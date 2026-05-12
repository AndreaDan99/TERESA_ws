#!/usr/bin/env python3
"""
WBC Keyboard Controller — keyboard-driven Spot control with WBC integration.

Keys:
  s — start:  save start pose (first press only) + trigger WBC SEARCHING
  r — return: stand + navigate back to start pose + realign yaw
  q — restart: same as return (interrupt WBC, go back to start)
  u — update:  overwrite start pose with current position + yaw
  c — sit
  a — stand

Displays WBC state changes from /wbc/state.
"""
import math
import os
import sys
import termios
import threading
import tty
from enum import Enum, auto

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion


class KCState(Enum):
    IDLE       = auto()
    ROTATING   = auto()
    DRIVING    = auto()
    REALIGNING = auto()


class WBCKeyboardControllerNode(Node):

    def __init__(self):
        super().__init__('wbc_keyboard_controller')

        self.declare_parameter('cmd_vel_topic',     '/my_spot/cmd_vel')
        self.declare_parameter('goal_tolerance',     0.15)
        self.declare_parameter('angular_speed_max',  0.4)
        self.declare_parameter('linear_speed_max',   0.3)
        self.declare_parameter('angle_threshold',    0.08)
        self.declare_parameter('robot_frame',        'my_spot/body')
        self.declare_parameter('odom_frame',         'my_spot/odom')
        self.declare_parameter('update_rate',        10.0)

        p = lambda n: self.get_parameter(n).value
        self._cmd_vel_topic     = p('cmd_vel_topic')
        self._goal_tol          = float(p('goal_tolerance'))
        self._ang_speed_max     = float(p('angular_speed_max'))
        self._lin_speed_max     = float(p('linear_speed_max'))
        self._angle_threshold   = float(p('angle_threshold'))
        self._robot_frame       = p('robot_frame')
        self._odom_frame        = p('odom_frame')
        self._update_rate       = float(p('update_rate'))

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_pub      = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._restart_pub  = self.create_publisher(Bool, '/wbc/restart', 10)
        self._sit_client   = self.create_client(Trigger, '/my_spot/sit')
        self._stand_client = self.create_client(Trigger, '/my_spot/stand')

        self.create_subscription(String, '/wbc/state', self._cb_wbc_state, 10)

        self._kc_state: KCState               = KCState.IDLE
        self._start_odom:   PoseStamped | None = None
        self._start_yaw:    float              = 0.0
        self._start_has_goal: bool             = False  # True = we have a saved start
        self._goal_odom:    PoseStamped | None = None
        self._is_returning: bool               = False
        self._lock = threading.Lock()

        period = 1.0 / self._update_rate
        self._timer = self.create_timer(period, self._control_loop)

        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            'WBC Keyboard Controller ready.\n'
            '  s=start | r=return | q=restart | u=update start | c=sit | a=stand'
        )

    def _cb_wbc_state(self, msg: String) -> None:
        self.get_logger().info(f'WBC state → {msg.data}')

    def _keyboard_loop(self) -> None:
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self.get_logger().warn('Keyboard not available (stdin is not a TTY).')
            return
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == 's':
                    self._on_start()
                elif ch == 'r':
                    self._on_return()
                elif ch == 'q':
                    self._on_return()  # same as return
                elif ch == 'u':
                    self._on_update_start()
                elif ch == 'c':
                    self._call_trigger(self._sit_client, 'Sit')
                elif ch == 'a':
                    self._call_trigger(self._stand_client, 'Stand')
                elif ch == '\x03':  # Ctrl+C
                    rclpy.shutdown()
                    break
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _save_start_pose(self) -> bool:
        try:
            t = self._tf_buffer.lookup_transform(
                self._odom_frame, self._robot_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            start = PoseStamped()
            start.header.frame_id = self._odom_frame
            start.header.stamp = self.get_clock().now().to_msg()
            start.pose.position.x = t.transform.translation.x
            start.pose.position.y = t.transform.translation.y
            start.pose.position.z = 0.0
            start.pose.orientation = t.transform.rotation
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            with self._lock:
                self._start_odom = start
                self._start_yaw = yaw
                self._start_has_goal = True
            self.get_logger().info(
                f'Start pose saved: ({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) '
                f'yaw={math.degrees(yaw):.1f}°')
            return True
        except TransformException as e:
            self.get_logger().warn(f'Cannot save start pose: {e}')
            return False

    def _on_start(self) -> None:
        if not self._start_has_goal:
            if not self._save_start_pose():
                return

        self.get_logger().info('Start → WBC SEARCHING')
        self._restart_pub.publish(Bool(data=True))

    def _on_update_start(self) -> None:
        self._save_start_pose()

    def _on_return(self) -> None:
        with self._lock:
            start = self._start_odom
        if start is None:
            self.get_logger().warn('No start pose — press "s" first.')
            return

        self.get_logger().info('Return: stopping WBC + navigating back to start ...')
        self._restart_pub.publish(Bool(data=False))

        if not self._stand_client.service_is_ready():
            self.get_logger().warn('Stand service not available — returning anyway')
            self._start_return_nav(start)
            return
        future = self._stand_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._start_return_nav(start))

    def _start_return_nav(self, start: PoseStamped) -> None:
        with self._lock:
            self._goal_odom = start
            self._is_returning = True
            self._kc_state = KCState.ROTATING
        self.get_logger().info(
            f'Navigating to start: ({start.pose.position.x:.2f}, {start.pose.position.y:.2f})')

    def _control_loop(self) -> None:
        with self._lock:
            state = self._kc_state
            goal_odom = self._goal_odom

        if state == KCState.IDLE or goal_odom is None:
            return

        goal_body_stamped = PoseStamped()
        goal_body_stamped.header.frame_id = self._odom_frame
        goal_body_stamped.header.stamp = rclpy.time.Time().to_msg()
        goal_body_stamped.pose = goal_odom.pose

        try:
            goal_body = self._tf_buffer.transform(
                goal_body_stamped, self._robot_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            return

        dx = goal_body.pose.position.x
        dy = goal_body.pose.position.y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        if state == KCState.ROTATING:
            if abs(angle_to_goal) < self._angle_threshold:
                with self._lock:
                    self._kc_state = KCState.DRIVING
                self.get_logger().info('DRIVING → start')
                state = KCState.DRIVING

        if state == KCState.DRIVING:
            if dist < self._goal_tol:
                self._cmd_pub.publish(Twist())
                self.get_logger().info('Start position reached — realigning yaw ...')
                with self._lock:
                    self._kc_state = KCState.REALIGNING
                return

        if state == KCState.REALIGNING:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._odom_frame, self._robot_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                q = t.transform.rotation
                _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            except TransformException:
                return

            with self._lock:
                target_yaw = self._start_yaw

            yaw_err = target_yaw - current_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            if abs(yaw_err) < self._angle_threshold:
                self._cmd_pub.publish(Twist())
                with self._lock:
                    self._kc_state = KCState.IDLE
                    self._goal_odom = None
                    self._is_returning = False
                self.get_logger().info('Yaw realigned — waiting for "s".')
                return

            twist = Twist()
            raw_ang = 1.0 * yaw_err
            twist.angular.z = float(max(-self._ang_speed_max,
                                        min(self._ang_speed_max, raw_ang)))
            self._cmd_pub.publish(twist)
            return

        twist = Twist()
        if state == KCState.ROTATING:
            raw_ang = 1.0 * angle_to_goal
            twist.angular.z = float(max(-self._ang_speed_max,
                                        min(self._ang_speed_max, raw_ang)))
        elif state == KCState.DRIVING:
            raw_lin = 0.5 * dist
            twist.linear.x = float(max(0.0, min(self._lin_speed_max, raw_lin)))
            raw_ang = 1.0 * angle_to_goal
            half = self._ang_speed_max / 2.0
            twist.angular.z = float(max(-half, min(half, raw_ang)))

        self._cmd_pub.publish(twist)

    def _call_trigger(self, client, name: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} service not available')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{name}: {"OK" if f.result().success else "FAILED"}'
            )
        )

    def destroy_node(self) -> None:
        self._cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WBCKeyboardControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
