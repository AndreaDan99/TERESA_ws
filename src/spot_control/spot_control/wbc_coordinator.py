#!/usr/bin/env python3
"""
WBC Coordinator — phase FSM (WBC is master, Z1 FSM waits for SCANNING)

States:
  SEARCHING      body lowered, waiting for LYING detection
  PRE_APPROACH   arm look-at toward target (Spot stationary)
  IDLE           passive fallback
  APPROACHING    Spot navigates + Z1 look-at via WBC QP
  SCANNING       Spot reached patient → WBC disables, z1_FSM takes over
  WS_EXTENSION   z1_FSM requested workspace help → QP micro-step

Transitions:
  SEARCHING     → PRE_APPROACH   posture=LYING and confidence >= threshold + approach_point
  SEARCHING     → IDLE           search timeout
  IDLE          → APPROACHING    posture=LYING and confidence >= threshold
  PRE_APPROACH  → APPROACHING    pre_approach duration elapsed (arm aligned)
  APPROACHING   → SCANNING       Spot within handoff_distance of approach_point
  SCANNING      → WS_EXTENSION   /wbc/ws_request received
  WS_EXTENSION  → SCANNING       /ik_done received
  any           → IDLE           posture != LYING for > lying_timeout
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped, Pose
from std_msgs.msg import Bool, String, Float32
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

from teresa_utils.orientation import quat_to_rot, normalize_angle


class _QualityMonitor:
    """Tracks approach point with best-confidence target and quality from posture_confidence.

    On init: collects N measurements in odom → target = mean.
    Target updated only when a measurement has significantly higher confidence.
    Quality = max_q * (1 - posture_confidence), grows linearly when confidence stops.
    Publish quality [m] — QP controller scales velocity proportionally.
    """

    def __init__(self, growth_rate: float = 0.05,
                 min_q: float = 0.01, max_q: float = 0.50,
                 buf_size: int = 3, conf_margin: float = 0.10):
        self._target: np.ndarray | None = None
        self._buf: list = []
        self._buf_size = buf_size
        self._quality = 0.0
        self._growth_rate = growth_rate
        self._min_q = min_q
        self._max_q = max_q
        self._conf_margin = conf_margin
        self._best_conf = 0.0
        self._last_conf_time = None
        self._initialized = False
        self._latest_meas: np.ndarray | None = None

    def try_init(self, z: np.ndarray, now: rclpy.time.Time) -> None:
        if self._initialized:
            self._latest_meas = z.copy()  # always keep latest for best-update
            return
        self._buf.append(z)
        self._latest_meas = z.copy()
        if len(self._buf) >= self._buf_size:
            self._target = np.mean(self._buf, axis=0)
            self._initialized = True
        self._last_conf_time = now

    def update_quality(self, conf: float, now: rclpy.time.Time) -> None:
        self._quality = max(self._min_q, min(self._max_q,
                            self._max_q * (1.0 - conf)))
        self._last_conf_time = now

    def try_best_update(self, conf: float, now: rclpy.time.Time) -> None:
        if not self._initialized or self._latest_meas is None:
            return
        if conf > self._best_conf + self._conf_margin:
            self._target = self._latest_meas.copy()
            self._best_conf = conf
            self._quality = max(self._min_q,
                               self._max_q * (1.0 - conf))

    def predict(self, now: rclpy.time.Time) -> None:
        if not self._initialized or self._last_conf_time is None:
            return
        dt = (now - self._last_conf_time).nanoseconds * 1e-9
        dt = max(dt, 0.0)
        self._quality = min(self._max_q,
                           self._quality + self._growth_rate * dt)

    def get_position(self) -> np.ndarray | None:
        return self._target.copy() if self._initialized else None

    def get_quality(self) -> float:
        return self._quality

    def reset(self) -> None:
        self._target = None
        self._buf = []
        self._quality = 0.0
        self._best_conf = 0.0
        self._last_conf_time = None
        self._latest_meas = None
        self._initialized = False

    def set_target(self, target: np.ndarray, best_conf: float = 0.85) -> None:
        self._target = target.copy()
        self._initialized = True
        self._best_conf = best_conf
        self._quality = 0.0
        self._latest_meas = target.copy()
        self._last_conf_time = None

    @property
    def initialized(self) -> bool:
        return self._initialized


class CoordState:
    SEARCHING     = 'SEARCHING'
    PRE_APPROACH  = 'PRE_APPROACH'
    IDLE          = 'IDLE'
    APPROACHING   = 'APPROACHING'
    HANDOFF       = 'HANDOFF'
    SCANNING      = 'SCANNING'
    WS_EXTENSION  = 'WS_EXTENSION'


class WBCCoordinatorNode(Node):

    def __init__(self):
        super().__init__('wbc_coordinator')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('handoff_distance',            0.05)
        self.declare_parameter('odom_frame',                   'my_spot/odom')
        self.declare_parameter('body_frame',                   'my_spot/body')
        self.declare_parameter('posture_confidence_topic',     '/human_pose/posture_confidence')
        self.declare_parameter('approach_point_topic',         '/laying_human/approach_point')
        self.declare_parameter('z1_fsm_state_topic',           '/z1_fsm/state')
        self.declare_parameter('wbc_goal_topic',               '/wbc/ee_goal')
        self.declare_parameter('wbc_enable_topic',             '/wbc/enable')
        self.declare_parameter('lying_timeout',                3.0)
        self.declare_parameter('confidence_margin',          0.10)
        self.declare_parameter('quality_growth',              0.05)
        self.declare_parameter('quality_min',                 0.01)
        self.declare_parameter('quality_max',                 0.50)
        self.declare_parameter('quality_buf_size',            3)
        self.declare_parameter('ws_ext_fwd_limit',             0.20)
        self.declare_parameter('ws_ext_lat_limit',             0.20)
        self.declare_parameter('ws_ext_bwd_limit',             0.50)
        self.declare_parameter('handoff_body_height',         -0.15)  # [m] offset from nominal
        self.declare_parameter('min_body_height',             -0.20)
        self.declare_parameter('max_body_height',              0.0)
        self.declare_parameter('search_body_height',          -0.20)  # [m] body lowered during search
        self.declare_parameter('search_pitch_angles',       [0.0])  # placeholder, overridden by YAML
        self.declare_parameter('search_yaw_offsets',        [0.0])  # placeholder, overridden by YAML
        self.declare_parameter('search_pause_per_point',    3.0)    # [s] pause per grid point
        self.declare_parameter('search_lock_confidence',    0.85)   # conf to lock and sample
        self.declare_parameter('search_lock_samples',       10)     # samples to average before exit
        self.declare_parameter('pre_approach_duration',        5.0)   # [s] arm look-at before Spot walks

        p = lambda n: self.get_parameter(n).value
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
        self._search_body_height    = float(p('search_body_height'))
        self._search_pitch_angles   = self._read_float_array('search_pitch_angles')
        self._search_yaw_offsets    = self._read_float_array('search_yaw_offsets')
        self._search_pause_per_point = float(p('search_pause_per_point'))
        self._search_lock_confidence = float(p('search_lock_confidence'))
        self._search_lock_samples   = int(p('search_lock_samples'))
        self._pre_approach_duration = float(p('pre_approach_duration'))

        # ── Body pose publisher (height + pitch via /my_spot/body_pose) ──
        self._pub_body_pose = self.create_publisher(Pose, '/my_spot/body_pose', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/my_spot/cmd_vel', 10)

        # ── Approach point quality monitor ────────────────────────────
        self._quality = _QualityMonitor(
            growth_rate=float(p('quality_growth')),
            min_q=float(p('quality_min')),
            max_q=float(p('quality_max')),
            buf_size=int(p('quality_buf_size')),
            conf_margin=float(p('confidence_margin')),
        )

        # ── TF ────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── State ─────────────────────────────────────────────────────
        self._state                  = None   # set via _set_state below
        self._posture                = 'UNKNOWN'
        self._confidence             = 0.0
        self._approach_point_odom: PoseStamped | None = None  # odom-frame (world-fixed)
        self._last_lying_time        = None
        self._desired_yaw: float | None = None   # target yaw Spot [rad, odom frame]
        self._ws_ext_anchor: np.ndarray | None = None   # Spot odom position at WS_EXTENSION entry
        self._ws_ext_anchor_yaw: float = 0.0            # Spot yaw at WS_EXTENSION entry
        self._search_start: rclpy.time.Time | None = None     # SEARCHING entry time
        self._pre_approach_start: rclpy.time.Time | None = None  # PRE_APPROACH entry time
        self._search_lock_buffer: list | None = None  # odom positions collected during lock
        self._search_timeline: list = []         # grid timeline: list of (t0, t1, pitch, yaw)
        self._search_total_duration: float = 0.0  # total grid duration [s]
        self._last_search_pitch: float = 0.0      # avoid redundant body_pose publishes

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(String,       '/human_pose/posture',        self._cb_posture,    10)
        self.create_subscription(Float32,      p('posture_confidence_topic'), self._cb_conf,       10)
        self.create_subscription(PoseStamped,  p('approach_point_topic'),    self._cb_approach,   10)
        self.create_subscription(String,       p('z1_fsm_state_topic'),      self._cb_z1_state,   10)
        self.create_subscription(Bool,         '/ik_done',                   self._cb_ik_done,    10)
        self.create_subscription(Bool,         '/wbc/ws_request',            self._cb_ws_req,     10)
        self.create_subscription(Vector3Stamped, '/laying_human/body_axis',  self._cb_body_axis,  10)
        self.create_subscription(Bool,           '/wbc/restart',             self._cb_restart,    10)

        self._pub_goal     = self.create_publisher(PoseStamped, p('wbc_goal_topic'),        10)
        self._pub_enable   = self.create_publisher(Bool,        p('wbc_enable_topic'),     10)
        self._pub_state    = self.create_publisher(String,      '/wbc/state',              10)
        self._pub_uncert   = self.create_publisher(Float32,     '/wbc/target_uncertainty', 10)
        self._pub_yaw      = self.create_publisher(Float32,     '/wbc/desired_yaw',        10)
        self._pub_spot_ctrl = self.create_publisher(Bool,       '/wbc/spot_control',       10)

        self.create_timer(0.2, self._tick)   # 5 Hz FSM
        self._set_state(CoordState.SEARCHING)
        self.get_logger().info(
            f'WBC Coordinator ready.\n'
            f'  FSM: 5 Hz  |  Search: 3×3 grid\n'
            f'  Lock: conf≥{self._search_lock_confidence}  |  samples: {self._search_lock_samples}')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_posture(self, msg: String) -> None:
        self._posture = msg.data
        if msg.data == 'LYING':
            self._last_lying_time = self.get_clock().now()

    def _cb_conf(self, msg: Float32) -> None:
        self._confidence = float(msg.data)
        self._quality.update_quality(self._confidence, self.get_clock().now())

    def _cb_body_axis(self, msg: Vector3Stamped) -> None:
        """Compute desired Spot yaw so that X_body ⊥ patient head-feet axis."""
        # Rotate body axis direction from camera frame to odom frame (rotation only).
        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, msg.header.frame_id,
                self.get_clock().now(), timeout=Duration(seconds=1.0))
        except TransformException:
            return

        R = quat_to_rot(tf.transform.rotation)
        axis_cam = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        axis_odom = R.T @ axis_cam   # R is odom→camera, we need camera→odom
        axis_odom[2] = 0.0  # project onto XY plane (Spot is on flat ground)
        n = float(np.linalg.norm(axis_odom[:2]))
        if n < 0.1:
            return

        # body_axis points head → feet in odom XY.
        # Spot X must be ⊥ to body_axis → two candidates: ±90°
        θ_body = math.atan2(float(axis_odom[1]), float(axis_odom[0]))
        opt1 = normalize_angle(θ_body + math.pi / 2)
        opt2 = normalize_angle(θ_body - math.pi / 2)

        # Pick the option closest to current Spot yaw (minimum rotation).
        try:
            body_in_odom = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                self.get_clock().now(), timeout=Duration(seconds=1.0))
        except TransformException:
            return

        θ_current = _yaw_from_quat(body_in_odom.transform.rotation)
        err1 = abs(normalize_angle(opt1 - θ_current))
        err2 = abs(normalize_angle(opt2 - θ_current))
        self._desired_yaw = opt1 if err1 <= err2 else opt2

        msg_out = Float32()
        msg_out.data = float(self._desired_yaw)
        self._pub_yaw.publish(msg_out)

    def _cb_approach(self, msg: PoseStamped) -> None:
        try:
            goal_odom = self._tf.transform(msg, self._odom_frame,
                                           timeout=Duration(seconds=1.0))
        except TransformException:
            return
        self._approach_point_odom = goal_odom
        if self._state != CoordState.SEARCHING:
            # In SEARCHING, target is set via lock + average; avoid polluting QualityMonitor.
            z = np.array([
                goal_odom.pose.position.x,
                goal_odom.pose.position.y,
                goal_odom.pose.position.z,
            ])
            self._quality.try_init(z, self.get_clock().now())

    def _cb_z1_state(self, msg: String) -> None:
        pass

    def _cb_ik_done(self, msg: Bool) -> None:
        if self._state == CoordState.WS_EXTENSION and msg.data:
            self._set_state(CoordState.SCANNING)
            self._set_wbc_enabled(False)

    def _cb_ws_req(self, msg: Bool) -> None:
        if self._state == CoordState.SCANNING and msg.data:
            self._set_state(CoordState.WS_EXTENSION)
            self._set_wbc_enabled(True)

    def _cb_restart(self, msg: Bool) -> None:
        if msg.data and self._state == CoordState.IDLE:
            self.get_logger().info('Keyboard restart → SEARCHING')
            self._set_state(CoordState.SEARCHING)
        elif not msg.data and self._state not in (CoordState.IDLE,):
            self.get_logger().info('Keyboard stop → IDLE')
            self._set_state(CoordState.IDLE)
            self._set_wbc_enabled(False)

    # ── FSM tick ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._check_lying_timeout()

        self._quality.try_best_update(self._confidence, self.get_clock().now())
        self._quality.predict(self.get_clock().now())
        u = Float32()
        u.data = self._quality.get_quality()
        self._pub_uncert.publish(u)

        if self._state == CoordState.IDLE:
            self._tick_idle()
        elif self._state == CoordState.SEARCHING:
            self._tick_searching()
        elif self._state == CoordState.PRE_APPROACH:
            self._tick_pre_approach()
        elif self._state == CoordState.APPROACHING:
            self._tick_approaching()
        elif self._state == CoordState.WS_EXTENSION:
            self._tick_ws_extension()

        s = String(); s.data = self._state
        self._pub_state.publish(s)

    def _check_lying_timeout(self) -> None:
        # Once committed (handoff done), don't abort on Orbbec loss — RealSense is in charge.
        if self._state in (CoordState.IDLE,
                            CoordState.SEARCHING,
                            CoordState.PRE_APPROACH,
                            CoordState.APPROACHING,
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
        pass  # IDLE is dead-end — restart requires external intervention

    def _tick_approaching(self) -> None:
        if self._approach_point_odom is None:
            return

        self._pub_goal.publish(self._filtered_goal())

        # Handoff triggered purely by distance — when Spot reaches the
        # approach_point (within handoff_distance tolerance), hand control
        # to z1_FSM regardless of Orbbec confidence.
        dist = self._distance_to_patient()
        if dist is not None and dist < self._handoff_dist:
            self.get_logger().info(
                f'Handoff: dist={dist:.2f}m < {self._handoff_dist:.2f}m → SCANNING')
            self._set_state(CoordState.SCANNING)
            self._set_wbc_enabled(False)

    def _tick_searching(self) -> None:
        lock_ok = (self._posture == 'LYING'
                   and self._confidence >= self._search_lock_confidence
                   and self._approach_point_odom is not None)

        # Phase 1: already locked — collecting samples
        if self._search_lock_buffer is not None:
            if lock_ok:
                z = np.array([
                    self._approach_point_odom.pose.position.x,
                    self._approach_point_odom.pose.position.y,
                    self._approach_point_odom.pose.position.z,
                ])
                self._search_lock_buffer.append(z)
                self.get_logger().info(
                    f'Lock: {len(self._search_lock_buffer)}/{self._search_lock_samples} samples',
                    throttle_duration_sec=1.0)
                if len(self._search_lock_buffer) >= self._search_lock_samples:
                    target = np.mean(self._search_lock_buffer, axis=0)
                    self._quality.set_target(target, self._search_lock_confidence)
                    self.get_logger().info(
                        f'Lock complete: {self._search_lock_samples} samples → PRE_APPROACH')
                    self._pre_approach_start = self.get_clock().now()
                    self._set_wbc_enabled(True)
                    self._pub_spot_ctrl.publish(Bool(data=False))
                    self._set_state(CoordState.PRE_APPROACH)
                return  # stay frozen (no rotation)
            else:
                self.get_logger().info('Lock lost — resuming search')
                self._search_lock_buffer = None
                # fall through to rotation

        # Phase 2: first lock trigger
        if lock_ok:
            self.get_logger().info(f'Lock: conf={self._confidence:.2f} — freezing')
            z = np.array([
                self._approach_point_odom.pose.position.x,
                self._approach_point_odom.pose.position.y,
                self._approach_point_odom.pose.position.z,
            ])
            self._search_lock_buffer = [z]
            return  # frozen, no rotation

        # Phase 3: grid search (no lock) — cycle through pitch×yaw points
        if self._search_start is not None:
            elapsed = (self.get_clock().now() - self._search_start).nanoseconds * 1e-9

            if elapsed >= self._search_total_duration:
                self.get_logger().warn('Search grid complete → IDLE')
                self._set_state(CoordState.IDLE)
                self._set_wbc_enabled(False)
                return

            # Find current grid point
            for t0, t1, pitch, yaw in self._search_timeline:
                if t0 <= elapsed < t1:
                    if abs(pitch - self._last_search_pitch) > 0.001:
                        self._last_search_pitch = pitch
                        self._set_body_pose(self._search_body_height, pitch, yaw)
                    break

    def _tick_pre_approach(self) -> None:
        self._pub_goal.publish(self._filtered_goal())

        if self._pre_approach_start is not None:
            elapsed = (self.get_clock().now() - self._pre_approach_start).nanoseconds * 1e-9
            if elapsed >= self._pre_approach_duration:
                self.get_logger().info(
                    f'PRE_APPROACH done ({self._pre_approach_duration:.1f}s) → APPROACHING')
                self._pub_spot_ctrl.publish(Bool(data=True))
                self._set_state(CoordState.APPROACHING)

    def _tick_ws_extension(self) -> None:
        # Goal published directly by z1_FSM via /wbc/ee_goal.
        # Transition back to SCANNING handled by _cb_ik_done.
        # Safety: enforce bounding box around anchor position.
        if self._ws_ext_anchor is None:
            return

        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                self.get_clock().now(), timeout=Duration(seconds=1.0))
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
        if self._approach_point_odom is None:
            return None
        try:
            body_in_odom = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                self.get_clock().now(), timeout=Duration(seconds=1.0))
        except TransformException:
            return None
        dx = body_in_odom.transform.translation.x - self._approach_point_odom.pose.position.x
        dy = body_in_odom.transform.translation.y - self._approach_point_odom.pose.position.y
        return math.hypot(dx, dy)

    def _filtered_goal(self) -> PoseStamped:
        """Return the fixed target position in odom frame."""
        msg = PoseStamped()
        msg.header.frame_id = self._odom_frame
        msg.header.stamp    = rclpy.time.Time().to_msg()
        msg.pose.orientation.w = 1.0  # identity — WBC QP recomputes orientation
        if self._quality.initialized:
            p = self._quality.get_position()
            msg.pose.position.x = float(p[0])
            msg.pose.position.y = float(p[1])
            msg.pose.position.z = float(p[2])
        elif self._approach_point_odom is not None:
            msg.pose.position = self._approach_point_odom.pose.position
        else:
            return PoseStamped()  # should not happen
        return msg

    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            self.get_logger().info(f'WBC FSM: {self._state} → {new_state}')
            if new_state == CoordState.IDLE:
                self._quality.reset()
                self._set_body_pose(0.0)   # ripristina altezza nominale
            if new_state == CoordState.SEARCHING:
                self._search_start = self.get_clock().now()
                self._search_lock_buffer = None
                self._search_timeline = self._build_search_timeline()
                self._search_total_duration = (self._search_timeline[-1][1]
                                               if self._search_timeline else 0.0)
                entry = self._search_timeline[0]
                self._last_search_pitch = entry[2]
                self._set_body_pose(self._search_body_height, entry[2], entry[3])
            if new_state == CoordState.SCANNING:
                self._set_body_pose(self._handoff_body_height)
            if new_state == CoordState.PRE_APPROACH:
                self._set_body_pose(0.0, 0.0)
            if new_state == CoordState.WS_EXTENSION:
                self._save_ws_ext_anchor()
            self._state = new_state

    def _save_ws_ext_anchor(self) -> None:
        """Save Spot odom position+yaw as anchor for WS_EXTENSION box constraint."""
        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                self.get_clock().now(), timeout=Duration(seconds=1.0))
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

    def _set_body_pose(self, height: float, pitch: float = 0.0, yaw: float = 0.0) -> None:
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, pitch, yaw)
        height_clamped = float(np.clip(height, self._min_body_height, self._max_body_height))
        pose = Pose()
        pose.position.z = height_clamped
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self._pub_body_pose.publish(pose)
        self._pub_cmd_vel.publish(Twist())
        self.get_logger().info(
            f'body_pose → height={height_clamped:.2f}m  pitch={math.degrees(pitch):.1f}°  yaw={math.degrees(yaw):.1f}°')

    def _read_float_array(self, param_name: str) -> list:
        val = self.get_parameter(param_name).value
        if isinstance(val, list):
            return [float(v) for v in val]
        return [float(val)]

    def _build_search_timeline(self) -> list:
        """Build timeline as list of (t_start, t_end, pitch, yaw).

        Entry: SEARCHING just entered. Capture current Spot yaw as reference,
        then build grid: for each yaw_offset, cycle through pitch angles.
        Each point: pause = search_pause_per_point [s].
        Returns timeline (may be empty if no params), sorted by t_start.
        """
        try:
            tf = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                self.get_clock().now(), timeout=Duration(seconds=1.0))
            ref_yaw = _yaw_from_quat(tf.transform.rotation)
        except TransformException:
            ref_yaw = 0.0
            self.get_logger().warn('SEARCH: could not get current yaw, using 0')

        timeline = []
        t = 0.0
        pause = self._search_pause_per_point
        for yaw_off in self._search_yaw_offsets:
            target_yaw = normalize_angle(ref_yaw + yaw_off)
            for pitch in self._search_pitch_angles:
                timeline.append((t, t + pause, pitch, target_yaw))
                t += pause
        return timeline

    def _set_wbc_enabled(self, enabled: bool) -> None:
        msg = Bool(); msg.data = enabled
        self._pub_enable.publish(msg)


def _yaw_from_quat(q) -> float:
    from tf_transformations import euler_from_quaternion
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(yaw)


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
