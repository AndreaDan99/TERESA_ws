#!/usr/bin/env python3
"""
WBC Coordinator — phase FSM

States:
  IDLE           waiting for LYING detection
  APPROACHING    Spot navigates + Z1 look-at via WBC QP
  HANDOFF        Spot reached patient → switch to RealSense, WBC stops
  SCANNING       z1_FSM active, WBC QP dormant, Spot stopped
  WS_EXTENSION   z1_FSM requested workspace help → QP micro-step

Transitions:
  IDLE         → APPROACHING    posture=LYING and confidence >= threshold
  APPROACHING  → HANDOFF        Spot within handoff_distance of approach_point
  HANDOFF      → SCANNING       z1_FSM enters APPROACHING (took over)
  SCANNING     → WS_EXTENSION   /wbc/ws_request received
  WS_EXTENSION → SCANNING       /ik_done received
  any          → IDLE           posture != LYING for > lying_timeout
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Bool, String, Float32
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


class _PositionKalman:
    """Constant-position 3D Kalman filter.

    Tracks approach point position and exposes tr(P_pos) as uncertainty.
    When measurements arrive regularly, P → small → robot moves fast.
    When measurements stop (predict-only), P grows → robot slows down.
    """

    def __init__(self, process_noise: float = 1e-3, measurement_noise: float = 2.5e-3):
        self._x = np.zeros(3)
        self._P = np.eye(3) * 1.0
        self._Q = np.eye(3) * process_noise
        self._R = np.eye(3) * measurement_noise
        self._initialized = False

    def predict(self) -> None:
        if not self._initialized:
            return
        self._P = self._P + self._Q

    def update(self, z: np.ndarray) -> None:
        if not self._initialized:
            self._x = z.copy()
            self._P = np.eye(3) * 0.1
            self._initialized = True
            return
        S = self._P + self._R
        K = self._P @ np.linalg.inv(S)
        self._x = self._x + K @ (z - self._x)
        self._P = (np.eye(3) - K) @ self._P

    def get_position(self) -> np.ndarray:
        return self._x.copy()

    def get_trace_cov(self) -> float:
        return float(np.trace(self._P))

    def get_sigma_max(self) -> float:
        """Max std dev: sqrt of largest eigenvalue of P_pos."""
        return float(np.sqrt(max(float(np.max(np.linalg.eigvalsh(self._P))), 0.0)))

    def reset(self) -> None:
        self._x = np.zeros(3)
        self._P = np.eye(3) * 1.0
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized


class CoordState:
    IDLE         = 'IDLE'
    APPROACHING  = 'APPROACHING'
    HANDOFF      = 'HANDOFF'
    SCANNING     = 'SCANNING'
    WS_EXTENSION = 'WS_EXTENSION'


class WBCCoordinatorNode(Node):

    def __init__(self):
        super().__init__('wbc_coordinator')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('orbbec_confidence_threshold',  0.4)
        self.declare_parameter('handoff_distance',            0.05)
        self.declare_parameter('odom_frame',                   'my_spot/odom')
        self.declare_parameter('body_frame',                   'my_spot/body')
        self.declare_parameter('posture_confidence_topic',     '/human_pose/posture_confidence')
        self.declare_parameter('approach_point_topic',         '/laying_human/approach_point')
        self.declare_parameter('z1_fsm_state_topic',           '/z1_fsm/state')
        self.declare_parameter('wbc_goal_topic',               '/wbc/ee_goal')
        self.declare_parameter('wbc_enable_topic',             '/wbc/enable')
        self.declare_parameter('lying_timeout',                3.0)
        self.declare_parameter('approach_kf_process_noise',    1e-3)
        self.declare_parameter('approach_kf_meas_noise',       2.5e-3)
        self.declare_parameter('ws_ext_fwd_limit',             0.20)
        self.declare_parameter('ws_ext_lat_limit',             0.20)
        self.declare_parameter('ws_ext_bwd_limit',             0.50)
        self.declare_parameter('handoff_body_height',         -0.15)  # [m] offset from nominal
        self.declare_parameter('min_body_height',             -0.20)
        self.declare_parameter('max_body_height',              0.0)

        p = lambda n: self.get_parameter(n).value
        self._conf_thr        = float(p('orbbec_confidence_threshold'))
        self._handoff_dist    = float(p('handoff_distance'))
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._lying_timeout   = float(p('lying_timeout'))
        self._ws_ext_fwd_lim  = float(p('ws_ext_fwd_limit'))
        self._ws_ext_lat_lim  = float(p('ws_ext_lat_limit'))
        self._ws_ext_bwd_lim  = float(p('ws_ext_bwd_limit'))
        self._handoff_body_height = float(p('handoff_body_height'))
        self._min_body_height     = float(p('min_body_height'))
        self._max_body_height     = float(p('max_body_height'))

        # ── Body height client (optional — needs spot_msgs) ───────────
        try:
            from spot_msgs.srv import SetStandHeight
            self._height_client = self.create_client(SetStandHeight, '/my_spot/set_stand_height')
            self._SetStandHeight = SetStandHeight
        except ImportError:
            self._height_client = None
            self._SetStandHeight = None
            self.get_logger().warn('spot_msgs not found — body height control disabled')

        # ── Approach point Kalman filter ───────────────────────────────
        self._kf_approach = _PositionKalman(
            process_noise=float(p('approach_kf_process_noise')),
            measurement_noise=float(p('approach_kf_meas_noise')),
        )

        # ── TF ────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── State ─────────────────────────────────────────────────────
        self._state                  = CoordState.IDLE
        self._posture                = 'UNKNOWN'
        self._confidence             = 0.0
        self._approach_point: PoseStamped | None = None
        self._last_lying_time        = None
        self._desired_yaw: float | None = None   # target yaw Spot [rad, odom frame]
        self._ws_ext_anchor: np.ndarray | None = None   # Spot odom position at WS_EXTENSION entry
        self._ws_ext_anchor_yaw: float = 0.0            # Spot yaw at WS_EXTENSION entry

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(String,       '/human_pose/posture',        self._cb_posture,    10)
        self.create_subscription(Float32,      p('posture_confidence_topic'), self._cb_conf,       10)
        self.create_subscription(PoseStamped,  p('approach_point_topic'),    self._cb_approach,   10)
        self.create_subscription(String,       p('z1_fsm_state_topic'),      self._cb_z1_state,   10)
        self.create_subscription(Bool,         '/ik_done',                   self._cb_ik_done,    10)
        self.create_subscription(Bool,         '/wbc/ws_request',            self._cb_ws_req,     10)
        self.create_subscription(Vector3Stamped, '/laying_human/body_axis',  self._cb_body_axis,  10)

        self._pub_goal    = self.create_publisher(PoseStamped, p('wbc_goal_topic'),        10)
        self._pub_enable  = self.create_publisher(Bool,        p('wbc_enable_topic'),     10)
        self._pub_state   = self.create_publisher(String,      '/wbc/state',              10)
        self._pub_uncert  = self.create_publisher(Float32,     '/wbc/target_uncertainty', 10)
        self._pub_yaw     = self.create_publisher(Float32,     '/wbc/desired_yaw',        10)

        self.create_timer(0.2, self._tick)   # 5 Hz FSM
        self.get_logger().info('WBC Coordinator ready.')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_posture(self, msg: String) -> None:
        self._posture = msg.data
        if msg.data == 'LYING':
            self._last_lying_time = self.get_clock().now()

    def _cb_conf(self, msg: Float32) -> None:
        self._confidence = float(msg.data)

    def _cb_body_axis(self, msg: Vector3Stamped) -> None:
        """Compute desired Spot yaw so that X_body ⊥ patient head-feet axis."""
        # Rotate body axis direction from camera frame to odom frame (rotation only).
        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, msg.header.frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return

        R = _quat_to_rot(tf.transform.rotation)
        axis_cam = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        axis_odom = R @ axis_cam
        axis_odom[2] = 0.0  # project onto XY plane (Spot is on flat ground)
        n = float(np.linalg.norm(axis_odom[:2]))
        if n < 0.1:
            return

        # body_axis points head → feet in odom XY.
        # Spot X must be ⊥ to body_axis → two candidates: ±90°
        θ_body = math.atan2(float(axis_odom[1]), float(axis_odom[0]))
        opt1 = _normalize_angle(θ_body + math.pi / 2)
        opt2 = _normalize_angle(θ_body - math.pi / 2)

        # Pick the option closest to current Spot yaw (minimum rotation).
        try:
            body_in_odom = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return

        θ_current = _yaw_from_quat(body_in_odom.transform.rotation)
        err1 = abs(_normalize_angle(opt1 - θ_current))
        err2 = abs(_normalize_angle(opt2 - θ_current))
        self._desired_yaw = opt1 if err1 <= err2 else opt2

        msg_out = Float32()
        msg_out.data = float(self._desired_yaw)
        self._pub_yaw.publish(msg_out)

    def _cb_approach(self, msg: PoseStamped) -> None:
        self._approach_point = msg
        z = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        self._kf_approach.update(z)

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

        # Propagate approach point uncertainty at FSM rate (5 Hz)
        self._kf_approach.predict()
        u = Float32()
        u.data = self._kf_approach.get_sigma_max()
        self._pub_uncert.publish(u)

        if self._state == CoordState.IDLE:
            self._tick_idle()
        elif self._state == CoordState.APPROACHING:
            self._tick_approaching()
        elif self._state == CoordState.WS_EXTENSION:
            self._tick_ws_extension()

        s = String(); s.data = self._state
        self._pub_state.publish(s)

    def _check_lying_timeout(self) -> None:
        # Once committed (handoff done), don't abort on Orbbec loss — RealSense is in charge.
        if self._state in (CoordState.IDLE,
                           CoordState.HANDOFF,
                           CoordState.SCANNING,
                           CoordState.WS_EXTENSION):
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

        self._pub_goal.publish(self._filtered_goal())

        # Handoff triggered purely by distance — when Spot reaches the
        # approach_point (within handoff_distance tolerance), hand control
        # to z1_FSM regardless of Orbbec confidence.
        dist = self._distance_to_patient()
        if dist is not None and dist < self._handoff_dist:
            self.get_logger().info(
                f'Handoff: dist={dist:.2f}m < {self._handoff_dist:.2f}m → HANDOFF')
            self._set_state(CoordState.HANDOFF)
            self._set_wbc_enabled(False)

    def _tick_ws_extension(self) -> None:
        # Goal published directly by z1_FSM via /wbc/ee_goal.
        # Transition back to SCANNING handled by _cb_ik_done.
        # Safety: enforce bounding box around anchor position.
        if self._ws_ext_anchor is None:
            return

        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return

        p_now = np.array([tf.transform.translation.x, tf.transform.translation.y])
        dp    = p_now - self._ws_ext_anchor
        c, s  = math.cos(self._ws_ext_anchor_yaw), math.sin(self._ws_ext_anchor_yaw)
        dp_fwd = float( c * dp[0] + s * dp[1])   # + = forward
        dp_lat = float(-s * dp[0] + c * dp[1])   # + = left

        violated = False
        if dp_fwd > self._ws_ext_fwd_lim:
            self.get_logger().warn(
                f'WS_EXT box violated: forward {dp_fwd:.2f}m > {self._ws_ext_fwd_lim:.2f}m')
            violated = True
        elif dp_fwd < -self._ws_ext_bwd_lim:
            self.get_logger().warn(
                f'WS_EXT box violated: backward {-dp_fwd:.2f}m > {self._ws_ext_bwd_lim:.2f}m')
            violated = True
        elif abs(dp_lat) > self._ws_ext_lat_lim:
            self.get_logger().warn(
                f'WS_EXT box violated: lateral {dp_lat:.2f}m > ±{self._ws_ext_lat_lim:.2f}m')
            violated = True

        if violated:
            self._set_wbc_enabled(False)
            self._set_state(CoordState.SCANNING)

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

    def _filtered_goal(self) -> PoseStamped:
        """Return approach_point with position replaced by Kalman estimate."""
        msg = PoseStamped()
        msg.header = self._approach_point.header
        msg.pose.orientation = self._approach_point.pose.orientation
        if self._kf_approach.initialized:
            p = self._kf_approach.get_position()
            msg.pose.position.x = float(p[0])
            msg.pose.position.y = float(p[1])
            msg.pose.position.z = float(p[2])
        else:
            msg.pose.position = self._approach_point.pose.position
        return msg

    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            self.get_logger().info(f'WBC FSM: {self._state} → {new_state}')
            if new_state == CoordState.IDLE:
                self._kf_approach.reset()
                self._set_body_height(0.0)   # ripristina altezza nominale
            if new_state == CoordState.HANDOFF:
                self._set_body_height(self._handoff_body_height)
            if new_state == CoordState.WS_EXTENSION:
                self._save_ws_ext_anchor()
            self._state = new_state

    def _save_ws_ext_anchor(self) -> None:
        """Save Spot odom position+yaw as anchor for WS_EXTENSION box constraint."""
        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            self._ws_ext_anchor = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
            ])
            self._ws_ext_anchor_yaw = _yaw_from_quat(tf.transform.rotation)
            self.get_logger().info(
                f'WS_EXT anchor saved: '
                f'p=[{self._ws_ext_anchor[0]:.2f},{self._ws_ext_anchor[1]:.2f}] '
                f'yaw={math.degrees(self._ws_ext_anchor_yaw):.1f}°'
            )
        except TransformException:
            self._ws_ext_anchor = None
            self.get_logger().warn('WS_EXT: could not save anchor (TF unavailable)')

    def _set_body_height(self, height: float) -> None:
        if self._height_client is None:
            return
        if not self._height_client.service_is_ready():
            self.get_logger().warn('set_stand_height service not ready — skipping')
            return
        req = self._SetStandHeight.Request()
        req.height = float(np.clip(height, self._min_body_height, self._max_body_height))
        future = self._height_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'body height → {req.height:.2f}m: '
                f'{"OK" if f.result() and f.result().success else "FAILED"}'
            )
        )

    def _set_wbc_enabled(self, enabled: bool) -> None:
        msg = Bool(); msg.data = enabled
        self._pub_enable.publish(msg)


def _quat_to_rot(q) -> np.ndarray:
    from tf_transformations import quaternion_matrix
    return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]


def _yaw_from_quat(q) -> float:
    from tf_transformations import euler_from_quaternion
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(yaw)


def _normalize_angle(a: float) -> float:
    """Wrap angle to (-π, π]."""
    return float((a + math.pi) % (2 * math.pi) - math.pi)


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
