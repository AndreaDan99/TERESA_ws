#!/usr/bin/env python3
"""
WBC Coordinator — phase FSM

States:
  IDLE           waiting for LYING detection
  APPROACHING    Spot navigates + Z1 look-at via WBC QP
  CONFIRMING     arm moves for better Orbbec view angle
  HANDOFF        Orbbec lost → switch to RealSense, WBC stops
  SCANNING       z1_FSM active, WBC QP dormant, Spot stopped
  WS_EXTENSION   z1_FSM requested workspace help → QP micro-step

Transitions:
  IDLE         → APPROACHING    posture=LYING and confidence >= threshold
  APPROACHING  → CONFIRMING     confidence drops below threshold
  CONFIRMING   → APPROACHING    confidence recovers
  APPROACHING  → HANDOFF        confidence low AND Spot near patient
  HANDOFF      → SCANNING       z1_FSM enters APPROACHING (took over)
  SCANNING     → WS_EXTENSION   /wbc/ws_request received
  WS_EXTENSION → SCANNING       /ik_done received
  any          → IDLE           posture != LYING for > lying_timeout
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Float32
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


class CoordState:
    IDLE         = 'IDLE'
    APPROACHING  = 'APPROACHING'
    CONFIRMING   = 'CONFIRMING'
    HANDOFF      = 'HANDOFF'
    SCANNING     = 'SCANNING'
    WS_EXTENSION = 'WS_EXTENSION'


class WBCCoordinatorNode(Node):

    def __init__(self):
        super().__init__('wbc_coordinator')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('orbbec_confidence_threshold', 0.4)
        self.declare_parameter('approach_done_distance',      0.8)
        self.declare_parameter('odom_frame',                  'my_spot/odom')
        self.declare_parameter('body_frame',                  'my_spot/body')
        self.declare_parameter('posture_confidence_topic',    '/human_pose/posture_confidence')
        self.declare_parameter('approach_point_topic',        '/laying_human/approach_point')
        self.declare_parameter('z1_fsm_state_topic',          '/z1_fsm/state')
        self.declare_parameter('wbc_goal_topic',              '/wbc/ee_goal')
        self.declare_parameter('wbc_enable_topic',            '/wbc/enable')
        self.declare_parameter('lying_timeout',               3.0)

        p = lambda n: self.get_parameter(n).value
        self._conf_thr      = float(p('orbbec_confidence_threshold'))
        self._approach_dist = float(p('approach_done_distance'))
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._lying_timeout = float(p('lying_timeout'))

        # ── TF ────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── State ─────────────────────────────────────────────────────
        self._state                  = CoordState.IDLE
        self._posture                = 'UNKNOWN'
        self._confidence             = 0.0
        self._approach_point: PoseStamped | None = None
        self._last_lying_time        = None

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(String,      '/human_pose/posture',       self._cb_posture,  10)
        self.create_subscription(Float32,     p('posture_confidence_topic'), self._cb_conf,    10)
        self.create_subscription(PoseStamped, p('approach_point_topic'),   self._cb_approach, 10)
        self.create_subscription(String,      p('z1_fsm_state_topic'),     self._cb_z1_state, 10)
        self.create_subscription(Bool,        '/ik_done',                  self._cb_ik_done,  10)
        self.create_subscription(Bool,        '/wbc/ws_request',           self._cb_ws_req,   10)

        self._pub_goal   = self.create_publisher(PoseStamped, p('wbc_goal_topic'),   10)
        self._pub_enable = self.create_publisher(Bool,        p('wbc_enable_topic'), 10)
        self._pub_state  = self.create_publisher(String,      '/wbc/state',          10)

        self.create_timer(0.2, self._tick)   # 5 Hz FSM
        self.get_logger().info('WBC Coordinator ready.')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_posture(self, msg: String) -> None:
        self._posture = msg.data
        if msg.data == 'LYING':
            self._last_lying_time = self.get_clock().now()

    def _cb_conf(self, msg: Float32) -> None:
        self._confidence = float(msg.data)

    def _cb_approach(self, msg: PoseStamped) -> None:
        self._approach_point = msg

    def _cb_z1_state(self, msg: String) -> None:
        if self._state == CoordState.HANDOFF and 'APPROACHING' in msg.data:
            self._set_state(CoordState.SCANNING)
            self._set_wbc_enabled(False)

    def _cb_ik_done(self, msg: Bool) -> None:
        if self._state == CoordState.WS_EXTENSION and msg.data:
            self._set_state(CoordState.SCANNING)

    def _cb_ws_req(self, msg: Bool) -> None:
        if self._state == CoordState.SCANNING and msg.data:
            self._set_state(CoordState.WS_EXTENSION)
            self._set_wbc_enabled(True)

    # ── FSM tick ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._check_lying_timeout()

        if self._state == CoordState.IDLE:
            self._tick_idle()
        elif self._state == CoordState.APPROACHING:
            self._tick_approaching()
        elif self._state == CoordState.CONFIRMING:
            self._tick_confirming()
        elif self._state == CoordState.WS_EXTENSION:
            self._tick_ws_extension()

        s = String(); s.data = self._state
        self._pub_state.publish(s)

    def _check_lying_timeout(self) -> None:
        if self._state == CoordState.IDLE:
            return
        if self._posture != 'LYING' and self._last_lying_time is not None:
            elapsed = (self.get_clock().now() - self._last_lying_time).nanoseconds * 1e-9
            if elapsed > self._lying_timeout:
                self.get_logger().warn('LYING timeout → IDLE')
                self._set_state(CoordState.IDLE)
                self._set_wbc_enabled(False)

    def _tick_idle(self) -> None:
        if (self._posture == 'LYING'
                and self._confidence >= self._conf_thr
                and self._approach_point is not None):
            self._set_state(CoordState.APPROACHING)
            self._set_wbc_enabled(True)

    def _tick_approaching(self) -> None:
        if self._approach_point is None:
            return

        self._pub_goal.publish(self._approach_point)

        if self._confidence < self._conf_thr:
            dist = self._distance_to_patient()
            if dist is not None and dist < self._approach_dist * 2:
                self._set_state(CoordState.HANDOFF)
                self._set_wbc_enabled(False)
            else:
                self._set_state(CoordState.CONFIRMING)

    def _tick_confirming(self) -> None:
        if self._confidence >= self._conf_thr:
            self._set_state(CoordState.APPROACHING)
            return
        if self._approach_point is not None:
            self._pub_goal.publish(self._approach_point)

    def _tick_ws_extension(self) -> None:
        # Goal published directly by z1_FSM via /wbc/ee_goal.
        # Transition back to SCANNING handled by _cb_ik_done.
        pass

    # ── Helpers ───────────────────────────────────────────────────────

    def _distance_to_patient(self) -> float | None:
        if self._approach_point is None:
            return None
        try:
            pt_body = self._tf.transform(
                self._approach_point, self._body_frame,
                timeout=Duration(seconds=0.1))
            return math.hypot(pt_body.pose.position.x, pt_body.pose.position.y)
        except TransformException:
            return None

    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            self.get_logger().info(f'WBC FSM: {self._state} → {new_state}')
            self._state = new_state

    def _set_wbc_enabled(self, enabled: bool) -> None:
        msg = Bool(); msg.data = enabled
        self._pub_enable.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WBCCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
