#!/usr/bin/env python3
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
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support


class NavState(Enum):
    IDLE     = auto()
    ROTATING = auto()
    DRIVING  = auto()
    STOPPED  = auto()


def compute_cmd_vel(dx: float, dy: float, state: NavState, params) -> Twist:
    twist = Twist()
    if state not in (NavState.ROTATING, NavState.DRIVING):
        return twist

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


class _Params:
    def __init__(self, node: Node):
        self.cmd_vel_topic     = node.get_parameter('cmd_vel_topic').value
        self.goal_tolerance    = float(node.get_parameter('goal_tolerance').value)
        self.angular_speed_max = float(node.get_parameter('angular_speed_max').value)
        self.linear_speed_max  = float(node.get_parameter('linear_speed_max').value)
        self.angle_threshold   = float(node.get_parameter('angle_threshold').value)
        self.robot_frame       = node.get_parameter('robot_frame').value
        self.odom_frame        = node.get_parameter('odom_frame').value
        self.update_rate       = float(node.get_parameter('update_rate').value)
        self.crouch_height     = float(node.get_parameter('crouch_height').value)
        self.kp_ang            = 1.0
        self.kp_lin            = 0.5


class SpotGoalNavigatorNode(Node):

    def __init__(self):
        super().__init__('spot_goal_navigator')

        self.declare_parameter('cmd_vel_topic',     '/my_spot/cmd_vel')
        self.declare_parameter('goal_tolerance',     0.3)
        self.declare_parameter('angular_speed_max',  0.5)
        self.declare_parameter('linear_speed_max',   0.4)
        self.declare_parameter('angle_threshold',    0.15)
        self.declare_parameter('robot_frame',        'my_spot/body')
        self.declare_parameter('odom_frame',         'my_spot/odom')
        self.declare_parameter('update_rate',        10.0)
        self.declare_parameter('crouch_height',     -0.10)  # metà altezza [m]

        self._p = _Params(self)

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._goal_sub = self.create_subscription(
            PoseStamped, '/laying_human/approach_point', self._cb_goal, 10)

        self._cmd_pub      = self.create_publisher(Twist, self._p.cmd_vel_topic, 10)
        self._sit_client   = self.create_client(Trigger, '/my_spot/sit')
        self._stand_client = self.create_client(Trigger, '/my_spot/stand')

        # SetStandHeight per altezza intermedia
        try:
            from spot_msgs.srv import SetStandHeight
            self._height_client = self.create_client(SetStandHeight, '/my_spot/set_stand_height')
            self._SetStandHeight = SetStandHeight
        except ImportError:
            self._height_client = None
            self._SetStandHeight = None
            self.get_logger().warn('spot_msgs non trovato — tasto h disabilitato')

        self._state: NavState                 = NavState.IDLE
        self._latest_goal: PoseStamped | None = None
        self._goal_odom:   PoseStamped | None = None
        self._start_odom:  PoseStamped | None = None
        self._is_returning: bool              = False  # True durante ritorno a start
        self._lock = threading.Lock()

        period = 1.0 / self._p.update_rate
        self._timer = self.create_timer(period, self._control_loop)

        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            f'SpotGoalNavigator ready — crouch_height={self._p.crouch_height}m\n'
            f'  "s" = naviga | "b" = alzati+torna | "h" = metà altezza | '
            f'"c" = siediti | "a" = alzati | ESC = stop'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_goal(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_goal = msg

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def _keyboard_loop(self) -> None:
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self.get_logger().warn(
                'Keyboard non disponibile (stdin non è TTY). '
                'Usa: ros2 run spot_control spot_goal_navigator'
            )
            return
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == 's':
                    self._on_start_key()
                elif ch == 'b':
                    self._on_return_key()
                elif ch == 'h':
                    self._set_height(self._p.crouch_height)
                elif ch == 'c':
                    self._call_trigger(self._sit_client, 'Sit')
                elif ch == 'a':
                    self._call_trigger(self._stand_client, 'Stand')
                elif ch == '\x1b':  # ESC
                    self._on_estop_key()
                elif ch == '\x03':  # Ctrl+C
                    rclpy.shutdown()
                    break
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _on_estop_key(self) -> None:
        with self._lock:
            self._state     = NavState.IDLE
            self._goal_odom = None
        self._cmd_pub.publish(Twist())
        self.get_logger().warn('EMERGENCY STOP — Spot fermato.')

    def _on_start_key(self) -> None:
        with self._lock:
            goal_raw = self._latest_goal

        if goal_raw is None:
            self.get_logger().warn('Nessun approach point ricevuto — avvia prima la percezione.')
            return

        goal_stamped = PoseStamped()
        goal_stamped.header.frame_id = goal_raw.header.frame_id
        goal_stamped.header.stamp    = rclpy.time.Time().to_msg()
        goal_stamped.pose            = goal_raw.pose

        try:
            goal_odom = self._tf_buffer.transform(
                goal_stamped, self._p.odom_frame,
                timeout=rclpy.duration.Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(f'TF fallita: {e} — navigazione non avviata.')
            return

        # Salva start pose
        try:
            t = self._tf_buffer.lookup_transform(
                self._p.odom_frame, self._p.robot_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            start = PoseStamped()
            start.header.frame_id  = self._p.odom_frame
            start.header.stamp     = self.get_clock().now().to_msg()
            start.pose.position.x  = t.transform.translation.x
            start.pose.position.y  = t.transform.translation.y
            start.pose.position.z  = 0.0
            start.pose.orientation = t.transform.rotation
            with self._lock:
                self._start_odom = start
            self.get_logger().info(
                f'Start pose salvata: ({start.pose.position.x:.2f}, {start.pose.position.y:.2f})')
        except TransformException as e:
            self.get_logger().warn(f'Impossibile salvare start pose: {e}')

        with self._lock:
            self._goal_odom    = goal_odom
            self._is_returning = False
            self._state        = NavState.ROTATING

        self.get_logger().info(
            f'Navigazione → ({goal_odom.pose.position.x:.2f}, {goal_odom.pose.position.y:.2f})')

    def _on_return_key(self) -> None:
        with self._lock:
            start = self._start_odom
        if start is None:
            self.get_logger().warn('Nessuna start pose — premi prima "s".')
            return
        self.get_logger().info('b: alzati poi torno a start...')
        # Prima si alza, poi quando il callback è done naviga indietro
        self._stand_then_return(start)

    def _stand_then_return(self, start: PoseStamped) -> None:
        if not self._stand_client.service_is_ready():
            self.get_logger().warn('Stand service non disponibile — torno comunque')
            self._start_return_nav(start)
            return
        future = self._stand_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._start_return_nav(start))

    def _start_return_nav(self, start: PoseStamped) -> None:
        with self._lock:
            self._goal_odom    = start
            self._is_returning = True
            self._state        = NavState.ROTATING
        self.get_logger().info(
            f'Ritorno a start: ({start.pose.position.x:.2f}, {start.pose.position.y:.2f})')

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        with self._lock:
            state     = self._state
            goal_odom = self._goal_odom

        if state == NavState.IDLE or goal_odom is None:
            return

        goal_body_stamped = PoseStamped()
        goal_body_stamped.header.frame_id = self._p.odom_frame
        goal_body_stamped.header.stamp    = rclpy.time.Time().to_msg()
        goal_body_stamped.pose            = goal_odom.pose

        try:
            goal_body = self._tf_buffer.transform(
                goal_body_stamped, self._p.robot_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TF error: {e}')
            return

        dx            = goal_body.pose.position.x
        dy            = goal_body.pose.position.y
        dist          = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        if state == NavState.ROTATING:
            if abs(angle_to_goal) < self._p.angle_threshold:
                with self._lock:
                    self._state = NavState.DRIVING
                self.get_logger().info('DRIVING')
                state = NavState.DRIVING

        if state == NavState.DRIVING:
            if dist < self._p.goal_tolerance:
                with self._lock:
                    is_ret = self._is_returning
                    self._state = NavState.IDLE
                self._cmd_pub.publish(Twist())
                if not is_ret:
                    self.get_logger().info('Arrivato — abbasso Spot a metà altezza')
                    self._set_height(self._p.crouch_height)
                else:
                    self.get_logger().info('Tornato a start — premi "s" per nuova missione')
                return

        twist = compute_cmd_vel(dx, dy, state, self._p)
        self._cmd_pub.publish(twist)

    # ── Service helpers ───────────────────────────────────────────────────────

    def _call_trigger(self, client, name: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} service non disponibile')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{name}: {"OK" if f.result().success else "FALLITO — " + f.result().message}'
            )
        )

    def _set_height(self, height: float) -> None:
        if self._height_client is None:
            self.get_logger().warn('set_stand_height non disponibile')
            return
        if not self._height_client.service_is_ready():
            self.get_logger().warn('set_stand_height service non pronto')
            return
        req = self._SetStandHeight.Request()
        req.height = float(height)
        future = self._height_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'set_stand_height({height}m): {"OK" if f.result().success else "FALLITO"}'
            )
        )

    def destroy_node(self) -> None:
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
