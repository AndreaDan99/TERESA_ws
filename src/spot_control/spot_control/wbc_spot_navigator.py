#!/usr/bin/env python3
"""
WBC Spot Navigator — point-to-point navigation with two modes:

  APPROACHING  — rotate → drive → stop (forward nav, used during approach)
  EXPOSURE     — direct P-controller on linear.x + linear.y (no rotation,
                 used during exposure scanning for small body-frame corrections)

Enabled/disabled via /wbc/spot_control.  Mode selected by parameter.
"""

import math

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


class _Params:
    def __init__(self, node: Node):
        p = lambda n: node.get_parameter(n).value
        self.goal_topic          = p('goal_topic')
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
        self.mode                = p('mode')
        self.nav_y_speed         = float(p('nav_y_speed'))
        self.nav_y_gain          = float(p('nav_y_gain'))
        self.nav_y_min_speed     = float(p('nav_y_min_speed'))
        self.nav_y_tolerance     = float(p('nav_y_tolerance'))
        self.nav_y_timeout       = float(p('nav_y_timeout'))


class WBCSpotNavigator(Node):

    def __init__(self):
        super().__init__('wbc_spot_navigator')

        self.declare_parameter('goal_topic',          '/wbc/ee_goal')
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
        self.declare_parameter('mode',                 'approaching')
        self.declare_parameter('nav_y_speed',          0.15)
        self.declare_parameter('nav_y_gain',           0.3)
        self.declare_parameter('nav_y_min_speed',      0.12)
        self.declare_parameter('nav_y_tolerance',      0.05)
        self.declare_parameter('nav_y_timeout',        10.0)
        self._p = _Params(self)

        self._tf = Buffer()
        TransformListener(self._tf, self)

        self.create_subscription(
            PoseStamped, self._p.goal_topic, self._cb_goal, 10)
        self.create_subscription(
            Bool, '/wbc/spot_control', self._cb_spot_control, 10)
        self.create_subscription(
            String, '/wbc/nav_mode', self._cb_nav_mode, 10)
        self._pub_vel = self.create_publisher(
            Twist, self._p.cmd_vel_topic, 10)

        self._state = 'IDLE'
        self._goal_odom: PoseStamped | None = None
        self._latest_goal: PoseStamped | None = None
        self._exp_target_y: float | None = None
        self._exp_start_time: rclpy.time.Time | None = None
        self._exp_active = False
        self._spot_control = True

        period = 1.0 / self._p.update_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'WBC Spot Navigator ready — mode={self._p.mode}  '
            f'goal={self._p.goal_topic}')

    def _cb_goal(self, msg: PoseStamped) -> None:
        self._latest_goal = msg

    def _cb_spot_control(self, msg: Bool) -> None:
        self._spot_control = msg.data

    def _cb_nav_mode(self, msg: String) -> None:
        self._p.mode = msg.data
        self.get_logger().info(f'Nav mode switched to: {msg.data}')

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

    def _tick_approaching(self) -> None:
        self._update_goal_odom()
        goal = self._goal_odom
        if goal is None:
            return

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
            return

        dx = pt.point.x
        dy = pt.point.y
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
            twist.linear.x = float(max(
                0.0, min(self._p.linear_speed_max, self._p.kp_lin * dist)))
            wz = float(max(
                -0.5 * self._p.angular_speed_max,
                min(0.5 * self._p.angular_speed_max, self._p.kp_ang * angle)))
            twist.angular.z = wz
            self._pub_vel.publish(twist)
            return

        if self._state == 'STOPPED':
            if dist > self._p.goal_tolerance:
                self._state = 'ROTATING'
                self.get_logger().info('New goal — restarting')
            else:
                self._pub_vel.publish(Twist())

    def _tick_exposure(self) -> None:
        self._update_goal_odom()
        goal = self._goal_odom

        try:
            t = self._tf.lookup_transform(
                self._p.odom_frame, self._p.robot_frame, rclpy.time.Time())
            current_x = t.transform.translation.x
            current_y = t.transform.translation.y
        except TransformException:
            self._pub_vel.publish(Twist())
            return

        twist = Twist()

        if goal is not None:
            self._exp_target_y = goal.pose.position.y
            if not self._exp_active:
                self._exp_active = True
                self._exp_start_time = self.get_clock().now()

            dx_body = goal.pose.position.x - current_x
            dy_odom = goal.pose.position.y - current_y

            if abs(dx_body) > self._p.goal_tolerance:
                speed_x = min(abs(dx_body) * self._p.kp_lin, self._p.linear_speed_max)
                twist.linear.x = speed_x if dx_body > 0 else -speed_x

            if abs(dy_odom) > self._p.nav_y_tolerance:
                speed_y = min(abs(dy_odom) * self._p.nav_y_gain, self._p.nav_y_speed)
                twist.linear.y = -math.copysign(max(speed_y, self._p.nav_y_min_speed), dy_odom)

            if (abs(dx_body) <= self._p.goal_tolerance
                    and abs(dy_odom) <= self._p.nav_y_tolerance):
                self._exp_active = False
                self._goal_odom = None
                self.get_logger().info(
                    f'Exposure nav arrived: x={current_x:.3f}, y={current_y:.3f}')
                self._pub_vel.publish(Twist())
                return

        if self._exp_active and self._exp_start_time is not None:
            now = self.get_clock().now()
            if (now - self._exp_start_time) > rclpy.duration.Duration(
                    seconds=self._p.nav_y_timeout):
                self._exp_active = False
                self._goal_odom = None
                self.get_logger().warn(
                    f'Exposure nav timeout after {self._p.nav_y_timeout}s')
                self._pub_vel.publish(Twist())
                return

        self._pub_vel.publish(twist)

    def _control_loop(self) -> None:
        if not self._spot_control:
            self._pub_vel.publish(Twist())
            self._state = 'IDLE'
            return

        if self._p.mode == 'exposure':
            self._tick_exposure()
        else:
            self._tick_approaching()


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
