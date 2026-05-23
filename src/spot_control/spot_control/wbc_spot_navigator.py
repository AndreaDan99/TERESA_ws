#!/usr/bin/env python3
"""
WBC Spot Navigator — simplified point-to-point navigation for WBC APPROACHING.

Subscribes to /wbc/ee_goal (PoseStamped in odom frame) and drives Spot
to the target position using rotate → drive → stop.

No keyboard, no return-to-start, no sit/stand. Just pure navigation.
"""

import math

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


class _Params:
    def __init__(self, node: Node):
        p = lambda n: node.get_parameter(n).value
        self.goal_topic       = p('goal_topic')
        self.cmd_vel_topic    = p('cmd_vel_topic')
        self.goal_tolerance   = float(p('goal_tolerance'))
        self.angular_speed_max = float(p('angular_speed_max'))
        self.linear_speed_max  = float(p('linear_speed_max'))
        self.angle_threshold   = float(p('angle_threshold'))
        self.robot_frame       = p('robot_frame')
        self.odom_frame        = p('odom_frame')
        self.update_rate       = float(p('update_rate'))
        self.kp_lin            = float(p('kp_lin'))
        self.kp_ang            = float(p('kp_ang'))


class WBCSpotNavigator(Node):

    def __init__(self):
        super().__init__('wbc_spot_navigator')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('goal_topic',       '/wbc/ee_goal')
        self.declare_parameter('cmd_vel_topic',    '/my_spot/cmd_vel')
        self.declare_parameter('goal_tolerance',    0.15)
        self.declare_parameter('angular_speed_max', 0.4)
        self.declare_parameter('linear_speed_max',  0.3)
        self.declare_parameter('angle_threshold',   0.08)
        self.declare_parameter('robot_frame',       'my_spot/body')
        self.declare_parameter('odom_frame',        'my_spot/odom')
        self.declare_parameter('update_rate',       10.0)
        self.declare_parameter('kp_lin',            0.5)
        self.declare_parameter('kp_ang',            0.8)
        self._p = _Params(self)

        # ── TF ─────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── Sub / Pub ──────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, self._p.goal_topic, self._cb_goal, 10)
        self.create_subscription(
            Bool, '/wbc/spot_control', self._cb_spot_control, 10)
        self._pub_vel = self.create_publisher(
            Twist, self._p.cmd_vel_topic, 10)

        # ── State ──────────────────────────────────────────────────────
        self._state = 'IDLE'          # IDLE | ROTATING | DRIVING | STOPPED
        self._goal_odom: PoseStamped | None = None
        self._latest_goal: PoseStamped | None = None
        self._spot_control = True  # enabled by default

        period = 1.0 / self._p.update_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'WBC Spot Navigator ready.  '
            f'goal={self._p.goal_topic}  cmd_vel={self._p.cmd_vel_topic}')

    def _cb_goal(self, msg: PoseStamped) -> None:
        self._latest_goal = msg

    def _cb_spot_control(self, msg: Bool) -> None:
        self._spot_control = msg.data

    def _update_goal_odom(self) -> None:
        goal_raw = self._latest_goal
        if goal_raw is None:
            return
        if goal_raw.header.frame_id == self._p.odom_frame:
            self._goal_odom = goal_raw
            return
        # Transform to odom
        goal_stamped = PoseStamped()
        goal_stamped.header.frame_id = goal_raw.header.frame_id
        goal_stamped.header.stamp = rclpy.time.Time().to_msg()
        goal_stamped.pose = goal_raw.pose
        try:
            g = self._tf.transform(
                goal_stamped, self._p.odom_frame,
                timeout=rclpy.duration.Duration(seconds=0.2))
            self._goal_odom = g
        except TransformException:
            pass

    def _control_loop(self) -> None:
        if not self._spot_control:
            self._pub_vel.publish(Twist())
            return
        self._update_goal_odom()
        goal = self._goal_odom
        if goal is None:
            return

        # Transform goal to body frame for error computation
        gb_stamped = PoseStamped()
        gb_stamped.header.frame_id = self._p.odom_frame
        gb_stamped.header.stamp = rclpy.time.Time().to_msg()
        gb_stamped.pose = goal.pose
        try:
            gb = self._tf.transform(
                gb_stamped, self._p.robot_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            self._pub_vel.publish(Twist())
            return

        dx = gb.pose.position.x
        dy = gb.pose.position.y
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        twist = Twist()

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
                return

        if self._state == 'DRIVING':
            if dist < self._p.goal_tolerance:
                self._state = 'STOPPED'
                self.get_logger().info('Goal reached — STOPPED')
                self._pub_vel.publish(Twist())
                return
            vx = float(max(
                0.0,
                min(self._p.linear_speed_max, self._p.kp_lin * dist)))
            wz = float(max(
                -0.5 * self._p.angular_speed_max,
                min(0.5 * self._p.angular_speed_max, self._p.kp_ang * angle)))
            twist.linear.x = vx
            twist.angular.z = wz
            self._pub_vel.publish(twist)
            return

        if self._state == 'STOPPED':
            if dist > self._p.goal_tolerance:
                self._state = 'ROTATING'
                self.get_logger().info('New goal — restarting')
            else:
                self._pub_vel.publish(Twist())


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
