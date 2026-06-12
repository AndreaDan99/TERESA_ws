#!/usr/bin/env python3
"""
WBC Spot Navigator — point-to-point navigation for WBC APPROACHING + EXPOSURE.

Two navigation modes:
  1. Forward (X + yaw):  rotate → drive → stop, used during APPROACHING.
  2. Lateral  (Y):       P-controller on cmd_vel.linear.y, used during
     exposure scanning for walking Spot along the patient's body.

Subscribes to /wbc/ee_goal (forward) and ~/lateral_goal (lateral, from
body_pose_optimizer).  Enabled/disabled via /wbc/spot_control.
"""

import math

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


class _Params:
    def __init__(self, node: Node):
        p = lambda n: node.get_parameter(n).value
        self.goal_topic          = p('goal_topic')
        self.lateral_goal_topic  = p('lateral_goal_topic')
        self.cmd_vel_topic       = p('cmd_vel_topic')
        self.goal_tolerance      = float(p('goal_tolerance'))
        self.angular_speed_max   = float(p('angular_speed_max'))
        self.linear_speed_max    = float(p('linear_speed_max'))
        self.angle_threshold     = float(p('angle_threshold'))
        self.robot_frame         = p('robot_frame')
        self.odom_frame          = p('odom_frame')
        self.update_rate         = float(p('update_rate'))
        self.kp_lin              = float(p('kp_lin'))
        self.kp_ang              = float(p('kp_ang'))
        # Lateral (Y) navigation — matching test_exposure_poses.py
        self.nav_y_speed         = float(p('nav_y_speed'))
        self.nav_y_gain          = float(p('nav_y_gain'))
        self.nav_y_min_speed     = float(p('nav_y_min_speed'))
        self.nav_y_tolerance     = float(p('nav_y_tolerance'))
        self.nav_y_timeout       = float(p('nav_y_timeout'))


class WBCSpotNavigator(Node):

    def __init__(self):
        super().__init__('wbc_spot_navigator')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('goal_topic',          '/wbc/ee_goal')
        self.declare_parameter('lateral_goal_topic',  '/body_pose_optimizer/navigator_goal')
        self.declare_parameter('cmd_vel_topic',       '/my_spot/cmd_vel')
        self.declare_parameter('goal_tolerance',       0.15)
        self.declare_parameter('angular_speed_max',    0.4)
        self.declare_parameter('linear_speed_max',     0.3)
        self.declare_parameter('angle_threshold',      0.08)
        self.declare_parameter('robot_frame',          'my_spot/body')
        self.declare_parameter('odom_frame',           'my_spot/odom')
        self.declare_parameter('update_rate',          10.0)
        self.declare_parameter('kp_lin',               0.5)
        self.declare_parameter('kp_ang',               0.8)
        # Lateral Y-nav (matching test_exposure_poses.py defaults)
        self.declare_parameter('nav_y_speed',          0.15)
        self.declare_parameter('nav_y_gain',           0.3)
        self.declare_parameter('nav_y_min_speed',      0.12)
        self.declare_parameter('nav_y_tolerance',      0.05)
        self.declare_parameter('nav_y_timeout',        10.0)
        self._p = _Params(self)

        # ── TF ─────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── Sub / Pub ──────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, self._p.goal_topic, self._cb_goal, 10)
        self.create_subscription(
            PoseStamped, self._p.lateral_goal_topic, self._cb_lateral_goal, 10)
        self.create_subscription(
            Bool, '/wbc/spot_control', self._cb_spot_control, 10)
        self._pub_vel = self.create_publisher(
            Twist, self._p.cmd_vel_topic, 10)

        # ── Forward-nav state ──────────────────────────────────────────
        self._state = 'IDLE'          # IDLE | ROTATING | DRIVING | STOPPED
        self._goal_odom: PoseStamped | None = None
        self._latest_goal: PoseStamped | None = None

        # ── Lateral-nav state ──────────────────────────────────────────
        self._lateral_target_y: float | None = None
        self._lateral_start_time: rclpy.time.Time | None = None
        self._lateral_active = False

        # ── Enable ─────────────────────────────────────────────────────
        self._spot_control = True  # enabled by default

        period = 1.0 / self._p.update_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'WBC Spot Navigator ready.  '
            f'forward={self._p.goal_topic}  lateral={self._p.lateral_goal_topic}  '
            f'cmd_vel={self._p.cmd_vel_topic}')

    # ═══════════════════════════════════════════════════════════════════
    #  Callbacks
    # ═══════════════════════════════════════════════════════════════════

    def _cb_goal(self, msg: PoseStamped) -> None:
        self._latest_goal = msg

    def _cb_lateral_goal(self, msg: PoseStamped) -> None:
        """Receive lateral navigation goal from body_pose_optimizer."""
        # Transform to odom if needed
        if msg.header.frame_id == self._p.odom_frame:
            self._lateral_target_y = msg.pose.position.y
        else:
            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id
            pt.header.stamp = rclpy.time.Time().to_msg()
            pt.point.x = msg.pose.position.x
            pt.point.y = msg.pose.position.y
            pt.point.z = msg.pose.position.z
            try:
                t = self._tf.transform(
                    pt, self._p.odom_frame,
                    timeout=rclpy.duration.Duration(seconds=0.2))
                self._lateral_target_y = t.point.y
            except TransformException:
                return
        self._lateral_active = True
        self._lateral_start_time = self.get_clock().now()
        self.get_logger().info(
            f'Lateral goal: target_y={self._lateral_target_y:.3f}')

    def _cb_spot_control(self, msg: Bool) -> None:
        self._spot_control = msg.data

    # ═══════════════════════════════════════════════════════════════════
    #  Forward navigation (X + yaw)
    # ═══════════════════════════════════════════════════════════════════

    def _update_goal_odom(self) -> None:
        goal_raw = self._latest_goal
        if goal_raw is None:
            return
        if goal_raw.header.frame_id == self._p.odom_frame:
            self._goal_odom = goal_raw
            return
        goal_pt = PointStamped()
        goal_pt.header.frame_id = goal_raw.header.frame_id
        goal_pt.header.stamp = rclpy.time.Time().to_msg()
        goal_pt.point.x = goal_raw.pose.position.x
        goal_pt.point.y = goal_raw.pose.position.y
        goal_pt.point.z = goal_raw.pose.position.z
        try:
            pt = self._tf.transform(
                goal_pt, self._p.odom_frame,
                timeout=rclpy.duration.Duration(seconds=0.2))
            self._goal_odom = PoseStamped()
            self._goal_odom.header = pt.header
            self._goal_odom.pose.position.x = pt.point.x
            self._goal_odom.pose.position.y = pt.point.y
            self._goal_odom.pose.position.z = pt.point.z
        except TransformException:
            pass

    def _tick_forward_nav(self, twist: Twist) -> bool:
        """Run forward navigation. Returns True if active (skip lateral)."""
        self._update_goal_odom()
        goal = self._goal_odom
        if goal is None:
            return False

        gb_pt = PointStamped()
        gb_pt.header.frame_id = self._p.odom_frame
        gb_pt.header.stamp = rclpy.time.Time().to_msg()
        gb_pt.point.x = goal.pose.position.x
        gb_pt.point.y = goal.pose.position.y
        gb_pt.point.z = goal.pose.position.z
        try:
            pt = self._tf.transform(
                gb_pt, self._p.robot_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            self._pub_vel.publish(Twist())
            return False

        dx = pt.point.x
        dy = pt.point.y
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        if self._state == 'IDLE':
            self._state = 'ROTATING'

        if self._state == 'ROTATING':
            if abs(angle) < self._p.angle_threshold:
                self._state = 'DRIVING'
            else:
                wz = float(max(
                    -self._p.angular_speed_max,
                    min(self._p.angular_speed_max, self._p.kp_ang * angle)))
                twist.angular.z = wz
                self._pub_vel.publish(twist)
                return True

        if self._state == 'DRIVING':
            if dist < self._p.goal_tolerance:
                self._state = 'STOPPED'
                self.get_logger().info('Forward goal reached — STOPPED')
                self._pub_vel.publish(Twist())
                return False
            twist.linear.x = float(max(
                0.0, min(self._p.linear_speed_max, self._p.kp_lin * dist)))
            wz = float(max(
                -0.5 * self._p.angular_speed_max,
                min(0.5 * self._p.angular_speed_max, self._p.kp_ang * angle)))
            twist.angular.z = wz
            self._pub_vel.publish(twist)
            return True

        if self._state == 'STOPPED':
            if dist > self._p.goal_tolerance:
                self._state = 'ROTATING'
                self.get_logger().info('New forward goal — restarting')
            else:
                self._pub_vel.publish(Twist())
            return False

        return False

    # ═══════════════════════════════════════════════════════════════════
    #  Lateral navigation (Y) — P-controller, matching test_exposure_poses.py
    # ═══════════════════════════════════════════════════════════════════

    def _tick_lateral_nav(self, twist: Twist) -> None:
        """P-controller for lateral (Y) Spot movement."""
        if not self._lateral_active or self._lateral_target_y is None:
            return

        try:
            t = self._tf.lookup_transform(
                self._p.odom_frame, self._p.robot_frame, rclpy.time.Time())
            current_y = t.transform.translation.y
        except TransformException:
            self.get_logger().warn(
                'TF lookup failed during lateral nav — skipping')
            self._lateral_active = False
            self._pub_vel.publish(Twist())
            return

        dy = self._lateral_target_y - current_y
        now = self.get_clock().now()

        if abs(dy) < self._p.nav_y_tolerance:
            self.get_logger().info(
                f'✅ Lateral nav arrived: target={self._lateral_target_y:.3f}, actual={current_y:.3f}')
            self._lateral_active = False
            self._lateral_target_y = None
            self._pub_vel.publish(Twist())
        elif (self._lateral_start_time is not None
              and (now - self._lateral_start_time) > rclpy.duration.Duration(
                  seconds=self._p.nav_y_timeout)):
            self._lateral_active = False
            self._lateral_target_y = None
            self._pub_vel.publish(Twist())
            self.get_logger().warn(
                f'⏰ Lateral nav timeout after {self._p.nav_y_timeout}s: '
                f'target={self._lateral_target_y:.3f}, actual={current_y:.3f}')
        else:
            speed = min(abs(dy) * self._p.nav_y_gain, self._p.nav_y_speed)
            # Spot cmd_vel.linear.y convention is inverted (positive = right)
            twist.linear.y = -math.copysign(
                max(speed, self._p.nav_y_min_speed), dy)
            self._pub_vel.publish(twist)

    # ═══════════════════════════════════════════════════════════════════
    #  Main control loop
    # ═══════════════════════════════════════════════════════════════════

    def _control_loop(self) -> None:
        if not self._spot_control:
            self._pub_vel.publish(Twist())
            self._state = 'IDLE'
            self._lateral_active = False
            return

        twist = Twist()
        forward_active = self._tick_forward_nav(twist)
        if not forward_active:
            self._tick_lateral_nav(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WBCSpotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
