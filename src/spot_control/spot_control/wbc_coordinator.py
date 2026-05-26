#!/usr/bin/env python3
"""
WBC Coordinator — phase FSM (WBC is master, Z1 FSM waits for SCANNING)

States:
  WAITING_TF    waits for tf_monitor to confirm TF chains ready
  SEARCHING     Spot rotates + arm explores (QP SEARCH_GRID). Hybrid lock: Orbbec (full) + RealSense (semi-lock guidance)
  SEMI_LOCKING  RealSense found torso → Spot rotates+tilts toward it, arm freezes, Orbbec gets 3s clean window
  LOCKING       Orbbec confirmed LYING → arm goes home, collect 5 approach_point samples
  PRE_APPROACH  Spot upright, QP LOOKAT: arm points X_ee toward target
  APPROACHING   Spot navigator drives toward target, QP SCAN_SEQ: arm does 11-pose grid
  SCANNING      Spot at patient, body pose optimization, z1_FSM runs FAST cycle
  IDLE          passive fallback

Transitions:
  SEARCHING     → SEMI_LOCKING   RealSense detects torso (LOCKED)
  SEMI_LOCKING  → LOCKING        Orbbec confirms LYING within 3s
  SEMI_LOCKING  → SEARCHING      timeout (3s, Orbbec didn't detect)
  SEARCHING     → LOCKING        Orbbec detects LYING directly (full lock)
  LOCKING       → PRE_APPROACH   5 approach_point samples collected
  LOCKING       → SEARCHING      lock lost during collection
  SEARCHING     → IDLE           search sequence exhausted
  IDLE          → SEARCHING      external restart (keyboard)
  PRE_APPROACH  → APPROACHING    RealSense LOCKED ×5 or 5s timeout
  APPROACHING   → SCANNING       Spot within handoff_distance of approach_point
  any           → IDLE           TF loss or emergency
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3Stamped, Pose, Point, PoseArray
from std_msgs.msg import Bool, String, Float32, Int32
from visualization_msgs.msg import Marker
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
    WAITING_TF    = 'WAITING_TF'
    SEARCHING     = 'SEARCHING'
    SEMI_LOCKING  = 'SEMI_LOCKING'
    LOCKING       = 'LOCKING'
    PRE_APPROACH  = 'PRE_APPROACH'
    IDLE          = 'IDLE'
    APPROACHING   = 'APPROACHING'
    SCANNING      = 'SCANNING'


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
        self.declare_parameter('z1_mount_x',                   0.20)
        self.declare_parameter('z1_mount_y',                   0.0)
        self.declare_parameter('z1_mount_z',                   0.20)
        self.declare_parameter('handoff_body_height',         -0.15)  # [m] offset from nominal
        self.declare_parameter('soft_handoff_distance',      0.20)   # [m] pause for scanner
        self.declare_parameter('min_body_height',             -0.20)
        self.declare_parameter('max_body_height',              0.0)
        self.declare_parameter('search_body_height',           0.0)   # [m] altezza nominale
        self.declare_parameter('search_yaw_increment',         1.05)  # [rad] ≈60° passo Spot
        self.declare_parameter('search_yaw_steps',             6)     # posizioni = 360°
        self.declare_parameter('search_pitch_angles',       [0.0, 0.087, 0.17])  # 0°,5°,10°
        self.declare_parameter('search_dwell',                  15.0)  # [s] attesa per posizione
        self.declare_parameter('search_semi_lock_pause',       3.0)   # [s] attesa per Orbbec
        self.declare_parameter('search_lock_confidence',        0.85)
        self.declare_parameter('search_lock_samples',           5)

        # FAST body pose optimization
        self.declare_parameter('body_grid_heights',       [-0.20, -0.18, -0.15])
        self.declare_parameter('body_grid_pitches',       [0.0, 0.087, 0.17, 0.26])
        self.declare_parameter('body_sweet_spot',         [0.35, 0.0, 0.30])
        self.declare_parameter('body_settle_time',        1.5)
        self.declare_parameter('ws_ext_dx_steps',          5)     # number of lateral grid steps
        self.declare_parameter('ws_ext_dx_max',            0.20)  # [m] max lateral displacement
        self.declare_parameter('ws_ext_dy_fwd_max',        0.20)  # [m] max forward displacement
        self.declare_parameter('ws_ext_dy_bwd_max',        0.30)  # [m] max backward displacement
        self.declare_parameter('navigator_timeout',        5.0)   # [s] max wait for Spot to reach WS_EXT goal
        self.declare_parameter('pre_approach_duration',        5.0)   # [s] arm look-at before Spot walks

        p = lambda n: self.get_parameter(n).value
        self._handoff_dist    = float(p('handoff_distance'))
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._lying_timeout   = float(p('lying_timeout'))
        self._mount_x              = float(p('z1_mount_x'))
        self._mount_y              = float(p('z1_mount_y'))
        self._mount_z              = float(p('z1_mount_z'))
        self._handoff_body_height = float(p('handoff_body_height'))
        self._soft_handoff_dist   = float(p('soft_handoff_distance'))
        self._min_body_height     = float(p('min_body_height'))
        self._max_body_height     = float(p('max_body_height'))
        self._search_body_height    = float(p('search_body_height'))
        self._search_yaw_increment  = float(p('search_yaw_increment'))
        self._search_yaw_steps      = int(p('search_yaw_steps'))
        self._search_pitch_angles   = self._read_float_array('search_pitch_angles')
        self._search_dwell          = float(p('search_dwell'))
        self._search_semi_lock_pause = float(p('search_semi_lock_pause'))
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
        self._tf_ready = False

        # ── State ─────────────────────────────────────────────────────
        self._state                  = None   # set via _set_state below
        self._posture                = 'UNKNOWN'
        self._confidence             = 0.0
        self._approach_point_odom: PoseStamped | None = None  # odom-frame (world-fixed)
        self._last_lying_time        = None
        self._desired_yaw: float | None = None   # target yaw Spot [rad, odom frame]
        self._current_body_height: float = 0.0   # last applied body_pose height
        self._search_start: rclpy.time.Time | None = None     # SEARCHING entry time
        self._pre_approach_start: rclpy.time.Time | None = None  # PRE_APPROACH entry time
        self._search_lock_buffer: list | None = None  # odom positions collected during lock
        self._search_positions: list = []              # [{yaw, pitch}, ...]
        self._search_position_idx: int = 0
        self._search_position_start: rclpy.time.Time | None = None
        self._search_saved_idx: int = 0               # idx da riprendere dopo semi-lock
        self._torso_tracker_state: str = ''           # LOCKED / TRACKING / IDLE
        self._torso_detected_ticks: int = 0           # consecutive LOCKED ticks
        self._torso_pos: PoseStamped | None = None    # ultima posa torso da RealSense

        # FAST body pose optimization + WS_EXTENSION
        self._fast_points: PoseArray | None = None
        self._optimal_poses: list = []                # [(h, p, dist), ...] per point
        self._needs_ws_ext: set = set()               # point indices needing WS_EXT
        self._body_settle_start: float | None = None  # timestamp when settle started
        self._ws_ext_driving: bool = False            # True while navigator drives to WS_EXT goal
        self._ws_ext_drive_start: float | None = None # timestamp when drive started
        self._ws_ext_failed: set = set()              # point indices where WS_EXT failed

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(String,       '/human_pose/posture',        self._cb_posture,    10)
        self.create_subscription(Float32,      p('posture_confidence_topic'), self._cb_conf,       10)
        self.create_subscription(PoseStamped,  p('approach_point_topic'),    self._cb_approach,   10)
        self.create_subscription(String,       p('z1_fsm_state_topic'),      self._cb_z1_state,   10)
        self.create_subscription(Bool,         '/z1/fast_ready',             self._cb_fast_ready, 10)
        self.create_subscription(String,      '/torso_tracker_state',      self._cb_torso_state, 10)
        self.create_subscription(PoseStamped,  '/torso_target_ee',          self._cb_torso_pos,   10)
        self.create_subscription(Vector3Stamped, '/laying_human/body_axis',  self._cb_body_axis,  10)
        self.create_subscription(Bool,           '/wbc/restart',             self._cb_restart,    10)
        self.create_subscription(Bool,           '/wbc/tf_ready',            self._cb_tf_ready,    10)
        self.create_subscription(PoseArray,      '/z1/fast_points',          self._cb_fast_points, 10)
        self.create_subscription(Int32,          '/z1/next_point_idx',       self._cb_next_point,  10)

        self._pub_goal     = self.create_publisher(PoseStamped, p('wbc_goal_topic'),        10)
        self._pub_enable   = self.create_publisher(Bool,        p('wbc_enable_topic'),     10)
        self._pub_state    = self.create_publisher(String,      '/wbc/state',              10)
        self._pub_uncert   = self.create_publisher(Float32,     '/wbc/target_uncertainty', 10)
        self._pub_yaw      = self.create_publisher(Float32,     '/wbc/desired_yaw',        10)
        self._pub_spot_ctrl = self.create_publisher(Bool,       '/wbc/spot_control',       10)
        self._pub_dbg_marker = self.create_publisher(Marker, '/wbc/debug_marker', 10)
        self._pub_body_ready = self.create_publisher(Bool, '/wbc/body_ready', 10)

        self.create_timer(0.2, self._tick)   # 5 Hz FSM
        self._set_state(CoordState.WAITING_TF)
        self.get_logger().info(
            f'WBC Coordinator ready — in attesa TF da SpotCore.\n'
            f'  Attendo {self._odom_frame} → {self._body_frame} ...\n'
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
        tf = self._tf_lookup(self._odom_frame, msg.header.frame_id)
        if tf is None:
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
        body_in_odom = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_in_odom is None:
            return

        if not self._tf_ready:
            self._tf_ready = True
            self.get_logger().info(
                f'TF disponibile: {self._odom_frame} → {self._body_frame} OK. '
                f'SpotCore connesso via DDS.')

        θ_current = _yaw_from_quat(body_in_odom.transform.rotation)
        err1 = abs(normalize_angle(opt1 - θ_current))
        err2 = abs(normalize_angle(opt2 - θ_current))
        self._desired_yaw = opt1 if err1 <= err2 else opt2

        msg_out = Float32()
        msg_out.data = float(self._desired_yaw)
        self._pub_yaw.publish(msg_out)

    def _cb_approach(self, msg: PoseStamped) -> None:
        goal_odom = self._tf_transform(msg, self._odom_frame)
        if goal_odom is None:
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

    def _cb_fast_ready(self, msg: Bool) -> None:
        if msg.data and self._state == CoordState.SCANNING:
            self.get_logger().info('FAST ready — disabling WBC')
            self._set_wbc_enabled(False)

    def _cb_restart(self, msg: Bool) -> None:
        if msg.data and self._state == CoordState.IDLE:
            self.get_logger().info('Keyboard restart → SEARCHING')
            self._set_state(CoordState.SEARCHING)
        elif not msg.data and self._state not in (CoordState.IDLE,):
            self.get_logger().info('Keyboard stop → IDLE')
            self._set_state(CoordState.IDLE)
            self._set_wbc_enabled(False)

    def _cb_torso_state(self, msg: String) -> None:
        self._torso_tracker_state = msg.data

    def _cb_torso_pos(self, msg: PoseStamped) -> None:
        self._torso_pos = msg

    def _cb_tf_ready(self, msg: Bool) -> None:
        if msg.data and self._state == CoordState.WAITING_TF:
            self.get_logger().info(
                'TF SpotCore disponibile → IDLE.\n'
                '  Premi "s" sul keyboard controller per avviare la missione.')
            self._tf_ready = True
            self._set_state(CoordState.IDLE)
        elif not msg.data and self._state != CoordState.WAITING_TF:
            self.get_logger().error(
                '⚠️  TF perse — tornando in WAITING_TF. '
                'Spot e braccio fermati.')
            self._tf_ready = False
            self._set_state(CoordState.WAITING_TF)

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
        elif self._state == CoordState.WAITING_TF:
            self._tick_waiting_tf()
        elif self._state == CoordState.SEARCHING:
            self._tick_searching()
        elif self._state == CoordState.SEMI_LOCKING:
            self._tick_semi_locking()
        elif self._state == CoordState.LOCKING:
            self._tick_locking()
        elif self._state == CoordState.PRE_APPROACH:
            self._tick_pre_approach()
        elif self._state == CoordState.APPROACHING:
            self._tick_approaching()
        elif self._state == CoordState.SCANNING:
            self._tick_scannning()

        s = String(); s.data = self._state
        self._pub_state.publish(s)

        self._pub_debug_marker()
        self._tick_fast_settle()

    def _check_lying_timeout(self) -> None:
        # Once committed (handoff done), don't abort on Orbbec loss — RealSense is in charge.
        if self._state in (CoordState.WAITING_TF,
                            CoordState.IDLE,
                            CoordState.SEARCHING,
                            CoordState.PRE_APPROACH,
                            CoordState.APPROACHING,
                            CoordState.SCANNING):
            return
        if self._posture != 'LYING' and self._last_lying_time is not None:
            elapsed = (self.get_clock().now() - self._last_lying_time).nanoseconds * 1e-9
            if elapsed > self._lying_timeout:
                self.get_logger().warn('LYING timeout → IDLE')
                self._set_state(CoordState.IDLE)
                self._set_wbc_enabled(False)

    def _tick_idle(self) -> None:
        pass  # IDLE is dead-end — restart requires external intervention

    def _tick_waiting_tf(self) -> None:
        pass  # attesa passiva — _cb_tf_ready farà la transizione

    def _tick_scannning(self) -> None:
        pass  # passive — FSM handles FAST, body pose via settle tick

    def _tick_approaching(self) -> None:
        if self._approach_point_odom is None:
            return

        # Publish goal for spot_goal_navigator (Spot) and look-at (arm)
        self._pub_goal.publish(self._filtered_goal())

        dist = self._distance_to_patient()
        if dist is None:
            return

        # ── Soft handoff (20cm): pause Spot if scanner not done ─────
        if dist < self._soft_handoff_dist and dist >= self._handoff_dist:
            if self._fast_points is None:
                self._pub_spot_ctrl.publish(Bool(data=False))  # pause navigator
            else:
                self._pub_spot_ctrl.publish(Bool(data=True))   # scanner done, resume
        elif dist < self._handoff_dist:
            if self._fast_points is None:
                self._pub_spot_ctrl.publish(Bool(data=False))  # wait for scanner
            else:
                # Hard handoff (5cm)
                self.get_logger().info(
                    f'Handoff: dist={dist:.2f}m < {self._handoff_dist:.2f}m → SCANNING')
                self._pub_spot_ctrl.publish(Bool(data=False))
                self._set_state(CoordState.SCANNING)
        else:
            self._pub_spot_ctrl.publish(Bool(data=True))  # navigator active

    def _tick_searching(self) -> None:
        # FASE 1 — Lock in corso: raccolta campioni
        if self._search_lock_buffer is not None:
            lock_ok = (self._posture == 'LYING'
                       and self._confidence >= self._search_lock_confidence
                       and self._approach_point_odom is not None)
            if lock_ok:
                z = self._approach_point_odom_pos()
                self._search_lock_buffer.append(z)
                if len(self._search_lock_buffer) >= self._search_lock_samples:
                    target = np.mean(self._search_lock_buffer, axis=0)
                    self._quality.set_target(target, self._search_lock_confidence)
                    self.get_logger().info(
                        f'Lock complete: {self._search_lock_samples} samples → PRE_APPROACH')
                    self._pre_approach_start = self.get_clock().now()
                    self._set_state(CoordState.PRE_APPROACH)
                return
            else:
                self.get_logger().info('Lock lost — resuming search')
                self._search_lock_buffer = None
                self._set_state(CoordState.SEARCHING)
                return

        # FASE 2 — Check full lock da Orbbec
        if self._posture == 'LYING' \
                and self._confidence >= self._search_lock_confidence \
                and self._approach_point_odom is not None:
            z = self._approach_point_odom_pos()
            self._search_lock_buffer = [z]
            self.get_logger().info(f'Full lock (Orbbec): conf={self._confidence:.2f}')
            self._set_state(CoordState.LOCKING)
            return

        # FASE 3 — Check semi-lock da RealSense
        if self._torso_tracker_state == 'LOCKED' and self._torso_pos is not None:
            if self._check_realsense_guidance():
                return

        # FASE 4 — Position cycling
        self._tick_search_positions()

    def _tick_semi_locking(self) -> None:
        """Spot ruotato+inclinato, braccio fermo, Orbbec cerca."""
        elapsed = (self.get_clock().now() - self._search_position_start).nanoseconds * 1e-9 \
            if self._search_position_start is not None else 0.0

        if self._posture == 'LYING' \
                and self._confidence >= self._search_lock_confidence \
                and self._approach_point_odom is not None:
            z = self._approach_point_odom_pos()
            self._search_lock_buffer = [z]
            self.get_logger().info('Semi-lock → Full lock (Orbbec)')
            self._set_state(CoordState.LOCKING)
            return

        if elapsed >= self._search_semi_lock_pause:
            self.get_logger().info('Semi-lock timeout → resuming search')
            self._search_position_idx = self._search_saved_idx
            self._search_position_start = None
            self._set_state(CoordState.SEARCHING)

    def _tick_locking(self) -> None:
        """Braccio va in home (QP), coordinator raccoglie campioni in parallelo."""
        if self._posture == 'LYING' \
                and self._confidence >= self._search_lock_confidence \
                and self._approach_point_odom is not None:
            z = self._approach_point_odom_pos()
            if len(self._search_lock_buffer) < self._search_lock_samples:
                self._search_lock_buffer.append(z)
            if len(self._search_lock_buffer) >= self._search_lock_samples:
                target = np.mean(self._search_lock_buffer, axis=0)
                self._quality.set_target(target, self._search_lock_confidence)
                self.get_logger().info(
                    f'Locking complete: {self._search_lock_samples} samples → PRE_APPROACH')
                self._pre_approach_start = self.get_clock().now()
                self._set_state(CoordState.PRE_APPROACH)
            return

        # Lock perso
        self._search_lock_buffer = None
        self._set_state(CoordState.SEARCHING)

    def _check_realsense_guidance(self) -> bool:
        """Ruota e inclina Spot verso il torso rilevato dalla RealSense."""
        if self._torso_tracker_state != 'LOCKED' or self._torso_pos is None:
            return False

        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            return False

        # Trasforma torso da world/link00 a body frame
        torso_body = self._transform_to_body_frame(self._torso_pos)
        if torso_body is None:
            return False

        # Orbbec in body frame: (0.30, 0, 0.15)
        dir_vec = np.array([torso_body[0] - 0.30, torso_body[1], torso_body[2] - 0.15])
        horiz = math.sqrt(dir_vec[0]**2 + dir_vec[1]**2)

        target_pitch = float(np.clip(math.atan2(-dir_vec[2], horiz), 0.0, 0.26))
        target_yaw = math.atan2(dir_vec[1], dir_vec[0])

        self.get_logger().info(
            f'Semi-lock (RealSense): torso_body=({torso_body[0]:.2f},{torso_body[1]:.2f},{torso_body[2]:.2f}) '
            f'→ yaw={math.degrees(target_yaw):.1f}° pitch={math.degrees(target_pitch):.1f}°')

        self._search_saved_idx = self._search_position_idx
        self._search_position_start = self.get_clock().now()
        self._set_body_pose(self._search_body_height, float(target_pitch), float(target_yaw))
        self._set_state(CoordState.SEMI_LOCKING)
        return True

    def _tick_search_positions(self) -> None:
        if self._search_position_idx >= len(self._search_positions):
            self.get_logger().warn('Search sequence complete → IDLE')
            self._set_wbc_enabled(False)
            self._set_state(CoordState.IDLE)
            return

        if self._search_position_start is None:
            pos = self._search_positions[self._search_position_idx]
            self._set_body_pose(self._search_body_height, pos['pitch'], pos['yaw'])
            self._search_position_start = self.get_clock().now()
            self.get_logger().info(
                f'Search pos {self._search_position_idx+1}/{len(self._search_positions)}: '
                f'yaw={math.degrees(pos["yaw"]):.0f}° pitch={math.degrees(pos["pitch"]):.0f}°')
            return

        elapsed = (self.get_clock().now() - self._search_position_start).nanoseconds * 1e-9
        if elapsed >= self._search_dwell:
            self._search_position_idx += 1
            self._search_position_start = None

    def _approach_point_odom_pos(self) -> np.ndarray:
        p = self._approach_point_odom.pose.position
        return np.array([p.x, p.y, p.z])

    def _transform_to_body_frame(self, pose: PoseStamped) -> np.ndarray | None:
        """Trasforma una posa da world/link00 al body frame via TF."""
        try:
            if pose.header.frame_id not in ('world', 'link00'):
                pose_world = PoseStamped()
                pose_world.header.frame_id = 'world'
                pose_world.pose = pose.pose
            else:
                pose_world = pose
            body_frame = self._tf_transform(pose_world, self._body_frame)
            if body_frame is None:
                return None
            return np.array([body_frame.pose.position.x,
                             body_frame.pose.position.y,
                             body_frame.pose.position.z])
        except Exception:
            return None

    def _tick_pre_approach(self) -> None:
        self._pub_goal.publish(self._filtered_goal())

        elapsed = (self.get_clock().now() - self._pre_approach_start).nanoseconds * 1e-9 \
            if self._pre_approach_start is not None else 0.0

        # Wait for 5 consecutive RealSense LOCKED ticks, max 5 seconds
        if self._torso_tracker_state == 'LOCKED':
            self._torso_detected_ticks += 1
            if self._torso_detected_ticks >= 5:
                self.get_logger().info('RealSense LOCKED ×5 → APPROACHING')
                self._pub_spot_ctrl.publish(Bool(data=False))
                self._set_state(CoordState.APPROACHING)
                return
        else:
            self._torso_detected_ticks = 0

        if elapsed > 5.0:
            self.get_logger().warn('PRE_APPROACH timeout (5s) → APPROACHING (fallback)')
            self._pub_spot_ctrl.publish(Bool(data=False))
            self._set_state(CoordState.APPROACHING)

    # ── Helpers ───────────────────────────────────────────────────────

    def _tf_lookup(self, source: str, target: str,
                   timeout_sec: float = 1.0) -> TransformStamped | None:
        try:
            return self._tf.lookup_transform(
                source, target, self.get_clock().now(),
                timeout=Duration(seconds=timeout_sec))
        except TransformException:
            if not self._tf_ready:
                self.get_logger().warn(
                    f'TF {source} → {target} non disponibile.\n'
                    f'  Diagnostica: ros2 topic list | grep tf',
                    throttle_duration_sec=5.0)
            return None

    def _tf_transform(self, pose: PoseStamped, target_frame: str,
                      timeout_sec: float = 1.0) -> PoseStamped | None:
        try:
            return self._tf.transform(
                pose, target_frame, timeout=Duration(seconds=timeout_sec))
        except TransformException:
            if not self._tf_ready:
                self.get_logger().warn(
                    f'TF {pose.header.frame_id} → {target_frame} non disponibile.\n'
                    f'  Diagnostica: ros2 topic list | grep tf',
                    throttle_duration_sec=5.0)
            return None

    def _distance_to_patient(self) -> float | None:
        if self._approach_point_odom is None:
            return None
        body_in_odom = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_in_odom is None:
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

    # ── FAST body pose optimization ────────────────────────────────────

    def _cb_fast_points(self, msg: PoseArray) -> None:
        """Receive FAST points from wbc_approach_scanner — run grid search."""
        if len(msg.poses) < 1:
            return
        self._fast_points = msg
        self.get_logger().info(f'FAST points received ({len(msg.poses)} poses)')
        self._optimize_body_poses()
        # Apply first body pose if already in SCANNING
        if self._state == CoordState.SCANNING:
            self._apply_fast_body_pose(0)

    def _cb_next_point(self, msg: Int32) -> None:
        """FSM signals next FAST point index."""
        idx = msg.data
        if idx < 0:
            self.get_logger().info('FAST done — restoring handoff height')
            self._set_body_pose(self._handoff_body_height, 0.0)
            self._pub_body_ready.publish(Bool(data=True))
            return
        if self._state == CoordState.SCANNING:
            self._apply_fast_body_pose(idx)

    def _optimize_body_poses(self) -> None:
        """Grid search: for each FAST point, find optimal (h, p)."""
        if self._fast_points is None:
            return

        heights = self._body_grid_heights
        pitches = self._body_grid_pitches
        sweet = self._body_sweet_spot

        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            return

        body_yaw = _yaw_from_quat(body_tf.transform.rotation)
        body_pos = np.array([body_tf.transform.translation.x,
                             body_tf.transform.translation.y,
                             body_tf.transform.translation.z])

        self._optimal_poses = []
        self._needs_ws_ext.clear()

        for i, target_pose in enumerate(self._fast_points.poses):
            target_link00 = np.array([target_pose.position.x,
                                      target_pose.position.y,
                                      target_pose.position.z])

            best_h, best_p, best_dist = 0.0, 0.0, float('inf')
            for h in heights:
                for p in pitches:
                    link00_odom, _ = self._simulate_link00(body_pos, body_yaw, h, p)
                    target_odom = self._link00_to_odom_vec(body_tf, target_link00)
                    target_new_link00 = self._odom_to_link00_vec(target_odom, link00_odom, body_yaw, p)
                    dist = float(np.linalg.norm(target_new_link00 - sweet))
                    if dist < best_dist:
                        best_h, best_p, best_dist = h, p, dist

            self._optimal_poses.append((best_h, best_p, best_dist))
            self.get_logger().info(
                f'FAST pt[{i}]: best (h={best_h:.2f}m, p={math.degrees(best_p):.1f}°)'
                f' → sweet_dist={best_dist:.3f}m')

            if best_dist > self._max_workspace_reach():
                self._needs_ws_ext.add(i)  # will trigger WS_EXT on apply

    def _max_workspace_reach(self) -> float:
        """Estimate max reachable distance from link00 in meters.
        Uses Z1 reach + safety margin derived from workspacesafety_margin."""
        return 0.60  # Z1 arm reach ~0.65m, minus safety

    # ── Simulation helpers ────────────────────────────────────────────

    def _simulate_link00(self, body_pos: np.ndarray, body_yaw: float,
                         height: float, pitch: float) -> tuple:
        """Compute link00 position + rotation in odom for a given body config.

        Returns (link00_odom_xyz, R_body_odom).
        body_pos: current body [x,y,z] in odom (at current body_pose height).
        height: desired body_pose height offset (negative = lower).
        pitch: desired body_pose pitch [rad].
        """
        from tf_transformations import quaternion_matrix
        body_nominal_z = float(body_pos[2]) - self._current_body_height
        body_new_z = body_nominal_z + height
        t_body = np.array([float(body_pos[0]), float(body_pos[1]), body_new_z])

        q = np.array([0.0, float(np.sin(pitch / 2.0)), 0.0,
                      float(np.cos(pitch / 2.0))])  # Ry(pitch)
        R_yaw = np.array([[np.cos(body_yaw), -np.sin(body_yaw), 0.0],
                          [np.sin(body_yaw),  np.cos(body_yaw), 0.0],
                          [0.0, 0.0, 1.0]])
        R_pitch = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                            [0.0, 1.0, 0.0],
                            [-np.sin(pitch), 0.0, np.cos(pitch)]])
        R_body = R_yaw @ R_pitch

        mount = np.array([self._mount_x, self._mount_y, self._mount_z])
        link00_odom = t_body + R_body @ mount
        return link00_odom, R_body

    def _link00_to_odom_vec(self, body_tf: TransformStamped,
                             vec_link00: np.ndarray) -> np.ndarray:
        """Transform a vector from link00 frame to odom frame using current body TF."""
        R = quat_to_rot(body_tf.transform.rotation)
        link00_in_odom = np.array([body_tf.transform.translation.x,
                                    body_tf.transform.translation.y,
                                    body_tf.transform.translation.z]) \
                         + R @ np.array([self._mount_x, self._mount_y, self._mount_z])
        return link00_in_odom + R @ vec_link00

    def _odom_to_link00_vec(self, point_odom: np.ndarray,
                             link00_odom: np.ndarray,
                             body_yaw: float, pitch: float) -> np.ndarray:
        """Transform a point from odom to the link00 frame for given body config."""
        R_yaw = np.array([[np.cos(body_yaw), -np.sin(body_yaw), 0.0],
                          [np.sin(body_yaw),  np.cos(body_yaw), 0.0],
                          [0.0, 0.0, 1.0]])
        R_pitch = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                            [0.0, 1.0, 0.0],
                            [-np.sin(pitch), 0.0, np.cos(pitch)]])
        R_body = R_yaw @ R_pitch
        return R_body.T @ (point_odom - link00_odom)

    # ── WS_EXTENSION grid search ──────────────────────────────────────

    def _optimize_ws_extension(self, point_idx: int):
        """Grid search (h, p, dx, dy) for a single point that needs WS_EXT."""
        if self._fast_points is None or point_idx >= len(self._fast_points.poses):
            return None

        heights = self._body_grid_heights
        pitches = self._body_grid_pitches
        sweet = self._body_sweet_spot

        n = self._ws_ext_dx_steps
        dx_range = np.linspace(-self._ws_ext_dx_max, self._ws_ext_dx_max, n).tolist()
        dy_range = np.linspace(-self._ws_ext_dy_bwd_max, self._ws_ext_dy_fwd_max, n).tolist()

        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            return None

        body_yaw = _yaw_from_quat(body_tf.transform.rotation)
        body_pos = np.array([body_tf.transform.translation.x,
                             body_tf.transform.translation.y,
                             body_tf.transform.translation.z])

        target_pose = self._fast_points.poses[point_idx]
        target_link00 = np.array([target_pose.position.x,
                                  target_pose.position.y,
                                  target_pose.position.z])
        target_odom = self._link00_to_odom_vec(body_tf, target_link00)

        best = None
        for h in heights:
            for p in pitches:
                for dx in dx_range:
                    for dy in dy_range:
                        shifted = body_pos + np.array([dx, dy, 0.0])
                        link00_odom, _ = self._simulate_link00(shifted, body_yaw, h, p)
                        target_new = self._odom_to_link00_vec(target_odom, link00_odom, body_yaw, p)
                        dist = float(np.linalg.norm(target_new - sweet))
                        if best is None or dist < best[4]:
                            best = (h, p, dx, dy, dist)

        if best is None:
            return None

        h, p, dx, dy, best_dist = best
        self.get_logger().info(
            f'WS_EXT pt[{point_idx}]: best (h={h:.2f}m, p={math.degrees(p):.1f}°, '
            f'dx={dx:.2f}m, dy={dy:.2f}m) → sweet_dist={best_dist:.3f}m')
        return best

    # ── FAST body pose application + settle ───────────────────────────

    def _apply_fast_body_pose(self, idx: int) -> None:
        """Apply optimized body pose for FAST point idx."""
        if not self._optimal_poses or idx >= len(self._optimal_poses):
            self._pub_body_ready.publish(Bool(data=True))
            return

        h, p, dist = self._optimal_poses[idx]

        if idx in self._needs_ws_ext and idx not in self._ws_ext_failed:
            # Try WS_EXTENSION grid search
            ws = self._optimize_ws_extension(idx)
            if ws is not None:
                h, p, dx, dy, best_dist = ws
                if best_dist < self._max_workspace_reach():
                    # Drive Spot to WS_EXT position
                    self._drive_ws_ext_position(idx, h, p, dx, dy)
                    return
                else:
                    self.get_logger().warn(
                        f'WS_EXT pt[{idx}]: best sweet_dist={best_dist:.3f}m '
                        f'still > {self._max_workspace_reach():.3f}m → giving up')
                    self._ws_ext_failed.add(idx)

        # Normal body pose (h, p only) or WS_EXT gave up
        self._set_body_pose(h, p)
        self._body_settle_start = self.get_clock().now().nanoseconds * 1e-9
        self._ws_ext_driving = False

    def _drive_ws_ext_position(self, idx: int, h: float, p: float,
                                dx: float, dy: float) -> None:
        """Publish navigator goal for WS_EXT displacement and start drive monitor."""
        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            self._ws_ext_failed.add(idx)
            self._apply_fast_body_pose(idx)  # fallback to (h,p) only
            return

        body_yaw = _yaw_from_quat(body_tf.transform.rotation)
        # Target odom position: current body + dx,dy offset
        goal_x = body_tf.transform.translation.x + dx
        goal_y = body_tf.transform.translation.y + dy

        goal = PoseStamped()
        goal.header.frame_id = self._odom_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = body_tf.transform.translation.z
        goal.pose.orientation.w = 1.0

        self._pub_goal.publish(goal)
        self._pub_spot_ctrl.publish(Bool(data=True))  # enable navigator
        self._ws_ext_driving = True
        self._ws_ext_drive_start = self.get_clock().now().nanoseconds * 1e-9
        self._ws_ext_pending_h = h
        self._ws_ext_pending_p = p
        self._ws_ext_pending_idx = idx
        self._ws_ext_goal_pos = (goal_x, goal_y)
        self.get_logger().info(
            f'WS_EXT pt[{idx}]: driving Spot by (dx={dx:.2f}, dy={dy:.2f}) '
            f'→ goal=({goal_x:.2f}, {goal_y:.2f})')

    def _tick_fast_settle(self) -> None:
        """Monitor body pose settle time or WS_EXT drive completion."""
        if self._ws_ext_driving:
            self._tick_ws_ext_drive()
            return

        if self._body_settle_start is None:
            return

        elapsed = (self.get_clock().now().nanoseconds * 1e-9
                   - self._body_settle_start)
        if elapsed >= self._body_settle_time:
            self.get_logger().info(
                f'Body settle complete ({elapsed:.1f}s ≥ {self._body_settle_time:.1f}s)')
            self._body_settle_start = None
            self._pub_body_ready.publish(Bool(data=True))

    def _tick_ws_ext_drive(self) -> None:
        """Monitor navigator progress toward WS_EXT goal."""
        if self._ws_ext_drive_start is None:
            return

        elapsed = (self.get_clock().now().nanoseconds * 1e-9
                   - self._ws_ext_drive_start)

        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            return

        dx = body_tf.transform.translation.x - self._ws_ext_goal_pos[0]
        dy = body_tf.transform.translation.y - self._ws_ext_goal_pos[1]
        dist = math.hypot(dx, dy)

        if dist < 0.15:  # goal_tolerance from spot_navigator
            self.get_logger().info(
                f'WS_EXT pt[{self._ws_ext_pending_idx}]: goal reached '
                f'(dist={dist:.3f}m in {elapsed:.1f}s)')
            self._finish_ws_ext_drive()
        elif elapsed > self._navigator_timeout:
            self.get_logger().warn(
                f'WS_EXT pt[{self._ws_ext_pending_idx}]: timeout '
                f'({elapsed:.1f}s > {self._navigator_timeout:.1f}s, '
                f'dist={dist:.3f}m) → giving up')
            self._ws_ext_failed.add(self._ws_ext_pending_idx)
            self._finish_ws_ext_drive()
        else:
            # Still driving — republish goal to keep navigator alive
            goal = PoseStamped()
            goal.header.frame_id = self._odom_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self._ws_ext_goal_pos[0]
            goal.pose.position.y = self._ws_ext_goal_pos[1]
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            self._pub_goal.publish(goal)

    def _finish_ws_ext_drive(self) -> None:
        """After navigator reaches WS_EXT goal, apply body pose and settle."""
        self._pub_spot_ctrl.publish(Bool(data=False))  # disable navigator
        self._set_body_pose(self._ws_ext_pending_h, self._ws_ext_pending_p)
        self._body_settle_start = self.get_clock().now().nanoseconds * 1e-9
        self._ws_ext_driving = False

    def _pub_debug_marker(self) -> None:
        """Publish debug marker for current state visualization."""
        pass

    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            self.get_logger().info(f'WBC FSM: {self._state} → {new_state}')
            if new_state == CoordState.WAITING_TF:
                self._set_wbc_enabled(False)
                self._set_body_pose(0.0)
            if new_state == CoordState.IDLE:
                self._quality.reset()
                self._set_body_pose(0.0)   # ripristina altezza nominale
            if new_state == CoordState.SEARCHING:
                self._search_start = self.get_clock().now()
                self._search_lock_buffer = None
                self._search_positions = self._build_search_sequence()
                self._search_position_idx = 0
                self._search_position_start = None
                self._search_saved_idx = 0
                self._set_wbc_enabled(True)
                pos0 = self._search_positions[0]
                self._set_body_pose(self._search_body_height, pos0['pitch'], pos0['yaw'])
            if new_state == CoordState.SCANNING:
                self._set_body_pose(self._handoff_body_height)
            if new_state == CoordState.PRE_APPROACH:
                self._set_body_pose(0.0, 0.0)
                self._torso_detected_ticks = 0
            self._state = new_state

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
        self._current_body_height = height_clamped
        self.get_logger().info(
            f'body_pose → height={height_clamped:.2f}m  pitch={math.degrees(pitch):.1f}°  yaw={math.degrees(yaw):.1f}°')

    def _read_float_array(self, param_name: str) -> list:
        val = self.get_parameter(param_name).value
        if isinstance(val, list):
            return [float(v) for v in val]
        return [float(val)]

    def _build_search_sequence(self) -> list:
        """Build search sequence: [0°, +incr, -incr, +2*incr, ...] × [pitch].
        Returns list of {yaw, pitch} dicts."""
        incr = self._search_yaw_increment
        steps = self._search_yaw_steps
        yaw_list = [0.0]
        for i in range(1, steps):
            n = (i + 1) // 2
            if i % 2 != 0:
                yaw_list.append(n * incr)
            else:
                yaw_list.append(-n * incr)
        seq = []
        for yaw in yaw_list:
            for pitch in self._search_pitch_angles:
                seq.append({'yaw': yaw, 'pitch': pitch})
        return seq

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
