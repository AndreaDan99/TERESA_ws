#!/usr/bin/env python3
import math
import sys
import threading
from enum import Enum, auto

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support


class NavState(Enum):
    IDLE     = auto()
    ROTATING = auto()
    DRIVING  = auto()
    STOPPED  = auto()


def compute_cmd_vel(dx: float, dy: float, state: NavState, params) -> Twist:
    """Pure function: compute Twist from goal offset in body frame.

    Args:
        dx: goal x in body frame (forward = positive)
        dy: goal y in body frame (left = positive)
        state: current NavState
        params: object with angular_speed_max, linear_speed_max, kp_ang, kp_lin

    Returns:
        geometry_msgs/Twist
    """
    twist = Twist()

    if state not in (NavState.ROTATING, NavState.DRIVING):
        return twist  # zero twist for IDLE / STOPPED

    angle_to_goal = math.atan2(dy, dx)
    dist          = math.hypot(dx, dy)

    if state == NavState.ROTATING:
        raw_ang = params.kp_ang * angle_to_goal
        twist.angular.z = float(max(-params.angular_speed_max,
                                    min(params.angular_speed_max, raw_ang)))

    elif state == NavState.DRIVING:
        raw_lin = params.kp_lin * dist
        twist.linear.x = float(max(0.0, min(params.linear_speed_max, raw_lin)))

        raw_ang = params.kp_ang * angle_to_goal
        half    = params.angular_speed_max / 2.0
        twist.angular.z = float(max(-half, min(half, raw_ang)))

    return twist


# ── ROS2 helpers ──────────────────────────────────────────────────────────────

class _Params:
    """Holds node parameters as plain attributes for use with compute_cmd_vel."""

    def __init__(self, node: Node):
        self.cmd_vel_topic     = node.get_parameter('cmd_vel_topic').value
        self.goal_tolerance    = float(node.get_parameter('goal_tolerance').value)
        self.angular_speed_max = float(node.get_parameter('angular_speed_max').value)
        self.linear_speed_max  = float(node.get_parameter('linear_speed_max').value)
        self.angle_threshold   = float(node.get_parameter('angle_threshold').value)
        self.robot_frame       = node.get_parameter('robot_frame').value
        self.odom_frame        = node.get_parameter('odom_frame').value
        self.update_rate       = float(node.get_parameter('update_rate').value)
        self.kp_ang            = 1.0
        self.kp_lin            = 0.5


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class SpotGoalNavigatorNode(Node):

    def __init__(self):
        super().__init__('spot_goal_navigator')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic',     '/cmd_vel')
        self.declare_parameter('goal_tolerance',     0.3)
        self.declare_parameter('angular_speed_max',  0.5)
        self.declare_parameter('linear_speed_max',   0.4)
        self.declare_parameter('angle_threshold',    0.15)
        self.declare_parameter('robot_frame',        'body')
        self.declare_parameter('odom_frame',         'odom')
        self.declare_parameter('update_rate',        10.0)

        self._p = _Params(self)

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Sub / Pub ──────────────────────────────────────────────────────────
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/laying_human/approach_point',
            self._cb_goal,
            10,
        )
        self._cmd_pub = self.create_publisher(Twist, self._p.cmd_vel_topic, 10)

        # ── State ──────────────────────────────────────────────────────────────
        # _latest_goal: raw approach point in camera frame (updated by sub)
        # _goal_odom:   goal transformed to odom frame at press of 's' (world-fixed)
        self._state: NavState                 = NavState.IDLE
        self._latest_goal: PoseStamped | None = None
        self._goal_odom:   PoseStamped | None = None
        self._lock = threading.Lock()

        # ── Control loop timer ─────────────────────────────────────────────────
        period = 1.0 / self._p.update_rate
        self._timer = self.create_timer(period, self._control_loop)

        # ── Keyboard thread ────────────────────────────────────────────────────
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            f'SpotGoalNavigator ready.\n'
            f'  cmd_vel → {self._p.cmd_vel_topic}\n'
            f'  robot_frame: {self._p.robot_frame}\n'
            f'Press "s" + Enter to start navigation to latest approach point.'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _cb_goal(self, msg: PoseStamped) -> None:
        """Store latest approach point (raw camera frame)."""
        with self._lock:
            self._latest_goal = msg

    def _keyboard_loop(self) -> None:
        """Blocking stdin reader — runs on daemon thread."""
        while rclpy.ok():
            try:
                line = sys.stdin.readline().strip()
            except EOFError:
                break
            if line == 's':
                self._on_start_key()

    def _on_start_key(self) -> None:
        with self._lock:
            goal_raw = self._latest_goal

        if goal_raw is None:
            self.get_logger().warn('No approach point received yet — cannot start.')
            return

        # Transform to odom (world-fixed) frame at press time.
        # Use stamp=Time() (zero) to request latest available TF.
        goal_stamped = PoseStamped()
        goal_stamped.header.frame_id = goal_raw.header.frame_id
        goal_stamped.header.stamp    = rclpy.time.Time().to_msg()
        goal_stamped.pose            = goal_raw.pose

        try:
            goal_odom = self._tf_buffer.transform(
                goal_stamped,
                self._p.odom_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e} — navigation not started.')
            return

        with self._lock:
            self._goal_odom = goal_odom
            self._state     = NavState.ROTATING

        self.get_logger().info(
            f'Navigation started → '
            f'({goal_odom.pose.position.x:.2f}, {goal_odom.pose.position.y:.2f}) [odom]'
        )

    # ── Control loop ───────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        with self._lock:
            state     = self._state
            goal_odom = self._goal_odom

        if state == NavState.IDLE or goal_odom is None:
            return

        # Re-transform goal from odom → body each tick so dx/dy reflect
        # current robot position (body frame moves with Spot).
        goal_body_stamped = PoseStamped()
        goal_body_stamped.header.frame_id = self._p.odom_frame
        goal_body_stamped.header.stamp    = rclpy.time.Time().to_msg()
        goal_body_stamped.pose            = goal_odom.pose

        try:
            goal_body = self._tf_buffer.transform(
                goal_body_stamped,
                self._p.robot_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF error in control loop: {e}')
            return

        dx = goal_body.pose.position.x
        dy = goal_body.pose.position.y
        dist          = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        # ── State transitions ──────────────────────────────────────────────────
        if state == NavState.ROTATING:
            if abs(angle_to_goal) < self._p.angle_threshold:
                with self._lock:
                    self._state = NavState.DRIVING
                self.get_logger().info('Phase 2: DRIVING')
                state = NavState.DRIVING

        if state == NavState.DRIVING:
            if dist < self._p.goal_tolerance:
                with self._lock:
                    self._state = NavState.STOPPED
                self._cmd_pub.publish(Twist())
                self.get_logger().info('Goal reached — STOPPED. Press "s" for next goal.')
                return

        if state == NavState.STOPPED:
            with self._lock:
                self._state = NavState.IDLE
            return

        # ── Publish velocity ───────────────────────────────────────────────────
        twist = compute_cmd_vel(dx, dy, state, self._p)
        self._cmd_pub.publish(twist)

    def destroy_node(self) -> None:
        """Publish zero Twist on shutdown to stop Spot."""
        self._cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpotGoalNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
