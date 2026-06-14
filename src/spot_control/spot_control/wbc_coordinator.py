#!/usr/bin/env python3
"""
WBC Coordinator — phase FSM (WBC is master, Z1 FSM waits for SCANNING)

States:
  WAITING_TF    waits for tf_monitor to confirm TF chains ready
  SEARCHING     Spot alternates pitch 0°/-5°/-10°, arm 7 poses each; after all: HOME + step forward,
               repeat. Hybrid lock: Orbbec (full) + RealSense (semi-lock guidance)
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
  SEARCHING     → IDLE           emergency / keyboard stop (otherwise loops ±30°)
  IDLE          → SEARCHING      external restart (keyboard)
  PRE_APPROACH  → APPROACHING    RealSense LOCKED ×5 or 5s timeout
  APPROACHING   → SCANNING       Spot within handoff_distance of approach_point
  any           → IDLE           TF loss or emergency
"""
import math
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy.time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3Stamped, Pose, Point, PoseArray
from std_msgs.msg import Bool, String, Float32, Int32
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

from teresa_utils.orientation import quat_to_rot, normalize_angle
from spot_perception.sml_pose_indices import SPINE1, SPINE2, SPINE3, PELVIS


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
    EXPOSURE_SCANNING = 'EXPOSURE_SCANNING'
    EXPOSURE_REVIEW   = 'EXPOSURE_REVIEW'
    WAITING_EXPOSURE  = 'WAITING_EXPOSURE'
    WAITING_FAST      = 'WAITING_FAST'


class WBCCoordinatorNode(Node):

    def __init__(self):
        super().__init__('wbc_coordinator')
        self._lock = threading.RLock()

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('handoff_distance',            0.05)
        self.declare_parameter('odom_frame',                   'my_spot/odom')
        self.declare_parameter('body_frame',                   'my_spot/body')
        self.declare_parameter('posture_confidence_topic',     '/human_pose/posture_confidence')
        self.declare_parameter('approach_point_topic',         '/laying_human/approach_point')
        self.declare_parameter('body_center_topic',            '/laying_human/body_center')
        self.declare_parameter('ik_done_topic',                '/ik_done')
        self.declare_parameter('z1_fsm_state_topic',           '/z1_fsm/state')
        self.declare_parameter('wbc_goal_topic',               '/wbc/spot_goal')
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
        self.declare_parameter('approach_timeout',              60.0)  # [s] max time to reach handoff
        self.declare_parameter('soft_handoff_distance',      0.20)   # [m] pause for scanner
        self.declare_parameter('min_body_height',             -0.20)
        self.declare_parameter('max_body_height',              0.0)
        self.declare_parameter('search_body_height',           0.0)   # [m] altezza nominale
        self.declare_parameter('search_step_forward',          0.50)  # [m] forward step after each pitch cycle
        self.declare_parameter('search_step_speed',            0.3)   # [m/s] forward step speed
        self.declare_parameter('search_yaw_kp',                    0.8)   # P-gain per rotazione yaw (semi-lock)
        self.declare_parameter('search_max_angular_vel',          0.5)   # [rad/s] max velocità angolare (semi-lock)
        self.declare_parameter('search_semi_lock_dwell',         3.0)   # [s] Orbbec dwell dopo settle
        self.declare_parameter('search_semi_lock_settle_timeout', 5.0)   # [s] timeout settle Spot
        self.declare_parameter('search_semi_lock_yaw_tol',        0.05)  # [rad] ≈3° tolleranza yaw
        self.declare_parameter('search_semi_lock_pitch_tol',      0.03)  # [rad] ≈2° tolleranza pitch
        self.declare_parameter('search_lock_confidence',          0.70)  # [0-1] soglia LYING per lock
        self.declare_parameter('search_lock_samples',           5)
        self.declare_parameter('search_arm_dwell',                30.0)  # [s] dwell time per yaw for arm scan

        # FAST body pose optimization
        self.declare_parameter('body_grid_heights',       [-0.20, -0.18, -0.15])
        self.declare_parameter('body_grid_pitches',       [0.0, 0.087, 0.17, 0.26])
        self.declare_parameter('body_sweet_spot',         [0.35, 0.0, 0.30])
        self.declare_parameter('body_settle_time',        1.5)
        self.declare_parameter('ws_ext_dx_steps',          5)     # number of lateral grid steps
        self.declare_parameter('ws_ext_dx_max',            0.20)  # [m] max lateral displacement
        self.declare_parameter('ws_ext_dy_fwd_max',        0.20)  # [m] max forward displacement
        self.declare_parameter('ws_ext_dy_bwd_max',        0.30)  # [m] max backward displacement
        self.declare_parameter('ws_ext_goal_tolerance',  0.15)   # [m] tolerance for WS_EXT drive arrival
        self.declare_parameter('max_workspace_reach',     0.60)   # [m] Z1 arm reach from link00
        self.declare_parameter('scan_timeout',           120.0)   # [s] max time in SCANNING phase
        self.declare_parameter('manual_scan_gate',     True)    # True = manual advance exposure→FAST
        self.declare_parameter('navigator_timeout',        5.0)   # [s] max wait for Spot to reach WS_EXT goal
        self.declare_parameter('pre_approach_duration',        5.0)   # [s] arm look-at before Spot walks
        self.declare_parameter('step_mode',                   False)  # gate automatic FSM transitions
        self.declare_parameter('nlf_coherence_threshold',     0.15)  # [m] YOLO−NLF delta for HIGH coherence
        self.declare_parameter('nlf_divergence_threshold',    0.30)  # [m] YOLO−NLF delta for MEDIUM coherence
        self.declare_parameter('nlf_excellent_confidence',    0.80)  # [0-1] NLF mean bbox_score for EXCELLENT tier

        p = lambda n: self.get_parameter(n).value
        self._handoff_dist    = float(p('handoff_distance'))
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._lying_timeout   = float(p('lying_timeout'))
        self._mount_x              = float(p('z1_mount_x'))
        self._mount_y              = float(p('z1_mount_y'))
        self._mount_z              = float(p('z1_mount_z'))
        self._handoff_body_height = float(p('handoff_body_height'))
        self._approach_timeout     = float(p('approach_timeout'))
        self._soft_handoff_dist   = float(p('soft_handoff_distance'))
        self._min_body_height     = float(p('min_body_height'))
        self._max_body_height     = float(p('max_body_height'))
        self._search_body_height    = float(p('search_body_height'))
        self._body_grid_heights = self._read_float_array('body_grid_heights')
        self._body_grid_pitches = self._read_float_array('body_grid_pitches')
        self._body_sweet_spot   = self._read_float_array('body_sweet_spot')
        self._search_step_forward   = float(p('search_step_forward'))
        self._search_step_speed     = float(p('search_step_speed'))
        self._search_yaw_kp         = float(p('search_yaw_kp'))
        self._search_max_angular_vel = float(p('search_max_angular_vel'))
        self._search_semi_lock_dwell          = float(p('search_semi_lock_dwell'))
        self._search_semi_lock_settle_timeout = float(p('search_semi_lock_settle_timeout'))
        self._search_semi_lock_yaw_tol        = float(p('search_semi_lock_yaw_tol'))
        self._search_semi_lock_pitch_tol      = float(p('search_semi_lock_pitch_tol'))
        self._search_lock_confidence = float(p('search_lock_confidence'))
        self._search_lock_samples   = int(p('search_lock_samples'))
        self._search_arm_dwell       = float(p('search_arm_dwell'))
        self._nlf_excellent_conf = float(p('nlf_excellent_confidence'))
        self._pre_approach_duration = float(p('pre_approach_duration'))
        self._body_settle_time      = float(p('body_settle_time'))
        self._step_mode          = bool(p('step_mode'))
        self._ws_ext_goal_tolerance = float(p('ws_ext_goal_tolerance'))
        self._max_reach_val          = float(p('max_workspace_reach'))
        self._scan_timeout          = float(p('scan_timeout'))
        self._manual_scan_gate      = bool(p('manual_scan_gate'))
        self._navigator_timeout      = float(p('navigator_timeout'))

        # ── Body pose publisher (height + pitch via /my_spot/body_pose) ──
        self._pub_body_pose = self.create_publisher(Pose, '/my_spot/body_pose', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/my_spot/cmd_vel', 10)

        # Perception enable publisher (transient_local — enables posture classifier + torso tracker)
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_perception_enable = self.create_publisher(Bool, '/wbc/perception_enable', qos_transient_local)

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
        self._step_pending_state: str | None = None  # gated transition waiting for confirm
        self._step_confirmed         = False
        self._posture                = 'UNKNOWN'
        self._confidence             = 0.0
        self._last_lying_time        = None
        self._current_body_height: float = 0.0   # last applied body_pose height
        self._search_start: rclpy.time.Time | None = None     # SEARCHING entry time
        self._pre_approach_start: rclpy.time.Time | None = None  # PRE_APPROACH entry time
        self._pre_approach_fast_start: rclpy.time.Time | None = None  # NLF fast-path safety gate timer
        self._approach_start: rclpy.time.Time | None = None      # APPROACHING entry time
        self._scan_start: rclpy.time.Time | None = None          # SCANNING entry time
        self._exposure_scan_start: rclpy.time.Time | None = None  # EXPOSURE_SCANNING entry time
        self._search_lock_buffer: list | None = None  # odom positions collected during lock
        self._search_positions: list = []              # [{yaw, pitch}, ...]
        self._search_position_idx: int = 0
        self._search_position_start: rclpy.time.Time | None = None
        self._search_saved_idx: int = 0               # idx da riprendere dopo semi-lock
        self._search_settling: bool = False            # True mentre attende che il body_pose si stabilizzi
        self._search_settle_start: rclpy.time.Time | None = None  # inizio attesa settle pitch
        self._search_semi_cooldown: int = 0           # tick cooldown after SEARCHING entry before semi-lock
        self._lock_lost_ticks: int = 0                # tick consecutivi senza Orbbec in LOCKING
        self._ik_done: bool = False                    # IK trajectory completion status
        self._search_ik_done_count: int = 0  # count ik_done events per search position
        self._search_home_phase: bool = False    # True while sending arm HOME between cycles
        self._search_scan_done: bool = False     # True when QP controller completes 7-pose scan
        self._search_step_phase: bool = False    # True while stepping Spot forward
        self._search_step_start: rclpy.time.Time | None = None  # forward step start time
        self._torso_tracker_state: str = ''           # LOCKED / TRACKING / IDLE
        self._torso_detected_ticks: list[bool] = []      # sliding window of recent (5) LOCKED/ESTIMATING ticks
        self._torso_pos: PoseStamped | None = None    # ultima posa torso da RealSense

        # SEMI_LOCKING settle tracking
        self._semi_lock_start_yaw: float = 0.0
        self._semi_lock_start_pitch: float = 0.0
        self._semi_lock_target_yaw: float = 0.0
        self._semi_lock_target_pitch: float = 0.0
        self._semi_lock_settle_done: bool = False
        self._semi_lock_dwell_start: rclpy.time.Time | None = None
        self._semi_lock_entry_time: float = 0.0      # timestamp ingresso per timeout assoluto

        # FAST body pose optimization
        self._fast_points: PoseArray | None = None
        self._optimizer_results: list = []            # [(h, p, dx, dy), ...] from optimizer
        self._body_settle_start: float | None = None  # timestamp when settle started

        # NLF prior (from /exposure/nlf_prior, 24 SMPL joints in odom)
        self._nlf_prior = None               # list[np.ndarray] | 'timeout' | None
        self._nlf_trigger_time = None         # rclpy.time.Time or None
        self._nlf_trigger_pending: bool = False  # True = trigger queued, waiting for 3s delay
        self._nlf_low_ticks = 0              # consecutive LOW-coherence ticks
        self._nlf_confidence = 0.0           # mean bbox_score from NLF burst

        # Exposure grid points (for body pose optimization)
        self._exposure_grid_points: list | None = None  # list of np.ndarray in world frame

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(String,       '/human_pose/posture',        self._cb_posture,    10)
        self.create_subscription(Float32,      p('posture_confidence_topic'), self._cb_conf,       10)
        self.create_subscription(Bool,         p('ik_done_topic'),           self._cb_ik_done,    10)
        self.create_subscription(String,       p('z1_fsm_state_topic'),      self._cb_z1_state,   10)
        self.create_subscription(Bool,         '/z1/fast_ready',             self._cb_fast_ready, 10)
        self.create_subscription(String,      '/torso_tracker_state',      self._cb_torso_state, 10)
        self.create_subscription(PoseStamped,  '/torso_target_ee',          self._cb_torso_pos,   10)
        self.create_subscription(Bool,           '/wbc/restart',             self._cb_restart,    10)
        self.create_subscription(Bool,           '/wbc/tf_ready',            self._cb_tf_ready,    10)
        self.create_subscription(PoseArray,      '/z1/fast_points',          self._cb_fast_points, 10)
        self.create_subscription(Int32,          '/z1/next_point_idx',       self._cb_next_point,  10)
        self.create_subscription(Bool,           '/exposure/ready',          self._cb_exposure_ready, 10)
        self.create_subscription(Bool,           '/exposure/terminate',      self._cb_terminate_exposure, 10)
        self.create_subscription(Bool,           '/wbc/set_manual_scan_gate', self._cb_manual_gate, 10)
        self.create_subscription(PoseArray,      '/exposure/grid_points',     self._cb_exposure_grid_points, 10)
        self.create_subscription(PoseArray,      '/exposure/nlf_prior',       self._cb_nlf_prior,         10)
        self.create_subscription(Float32,        '/exposure/nlf_confidence',  self._cb_nlf_confidence,    10)
        self.create_subscription(PoseArray,      '/human_pose/points_3d',     self._cb_skeleton_stream,   10)
        self.create_subscription(PoseArray,      '~/optimize_result',         self._cb_optimizer_result,  10)

        self._pub_goal     = self.create_publisher(PoseStamped, '/wbc/arm_goal',        10)  # arm look-at
        self._pub_spot_goal = self.create_publisher(PoseStamped, '/wbc/spot_goal',        10)  # Spot nav
        self._pub_nlf_trigger = self.create_publisher(Bool, '/nlf/trigger', 10)
        self._pub_enable   = self.create_publisher(Bool,        p('wbc_enable_topic'),     10)
        self._pub_ik_goal  = self.create_publisher(PoseStamped, '/wbc/ik_goal_pose',       10)
        self._pub_state    = self.create_publisher(String,      '/wbc/state',              10)
        self._pub_uncert   = self.create_publisher(Float32,     '/wbc/target_uncertainty', 10)
        self._pub_yaw      = self.create_publisher(Float32,     '/wbc/desired_yaw',        10)
        self._pub_spot_ctrl = self.create_publisher(Bool,       '/wbc/spot_control',       10)
        self._pub_nav_mode  = self.create_publisher(String,     '/wbc/nav_mode',            10)
        self._pub_dbg_marker = self.create_publisher(Marker, '/wbc/debug_marker', 10)
        self._pub_body_ready = self.create_publisher(Bool, '/wbc/body_ready', 10)
        self._pub_optimize_request = self.create_publisher(PoseArray, '~/optimize_request', 10)
        self._pub_step_pending = self.create_publisher(String, '/wbc/step_pending', 10)
        self._pub_guidance = self.create_publisher(Bool, '/tracker_guidance_mode', 10)
        self._pub_handoff  = self.create_publisher(Bool, '/wbc/handoff_reached', 10)
        self._pub_manual_gate = self.create_publisher(Bool, '/wbc/manual_scan_gate', 10)
        self.create_subscription(Bool, '/wbc/step_confirm', self._cb_step_confirm, 10)
        self.create_subscription(Bool, '/wbc/scan_done',    self._cb_scan_done,    10)

        self.create_timer(0.1, self._tick)   # 10 Hz FSM
        self._set_state(CoordState.WAITING_TF)
        self._pub_manual_gate.publish(Bool(data=self._manual_scan_gate))
        self.get_logger().info(
            f'WBC Coordinator ready — in attesa TF da SpotCore.\n'
            f'  Attendo {self._odom_frame} → {self._body_frame} ...\n'
            f'  FSM: 10 Hz  |  Search: pitch 0°/-5°/-10° × 7 arm poses × step forward\n'
            f'  Lock: conf≥{self._search_lock_confidence}  |  samples: {self._search_lock_samples}')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_posture(self, msg: String) -> None:
        with self._lock:
            self._posture = msg.data
            if msg.data == 'LYING':
                self._last_lying_time = self.get_clock().now()

    def _cb_conf(self, msg: Float32) -> None:
        with self._lock:
            self._confidence = float(msg.data)
            self._quality.update_quality(self._confidence, self.get_clock().now())

    def _get_body_yaw(self) -> float | None:
        """Return body yaw from patient_body TF, or None if unavailable."""
        try:
            t = self._tf.lookup_transform('my_spot/odom', 'patient_body', rclpy.time.Time())
            q = t.transform.rotation
            return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        except Exception:
            return None

    def _get_approach_odom(self) -> np.ndarray | None:
        """Return approach point in odom from patient_body TF."""
        try:
            t = self._tf.lookup_transform('my_spot/odom', 'patient_body', rclpy.time.Time())
            return np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        except Exception:
            return None

    def _cb_ik_done(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and not self._ik_done:
                self.get_logger().info('✅ ik_done received — arm home reached')
            self._ik_done = msg.data

    def _cb_z1_state(self, msg: String) -> None:
        pass

    def _cb_fast_ready(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and self._state == CoordState.SCANNING:
                self.get_logger().info('FAST ready — disabling WBC')
                self._set_wbc_enabled(False)

    def _cb_restart(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and self._state == CoordState.IDLE:
                self.get_logger().info('Keyboard restart → SEARCHING')
                self._set_state(CoordState.SEARCHING)
            elif not msg.data and self._state not in (CoordState.IDLE,):
                self.get_logger().info('Keyboard stop → IDLE')
                self._step_pending_state = None
                self._step_confirmed = False
                self._set_state(CoordState.IDLE, force=True)
                self._set_wbc_enabled(False)

    def _cb_step_confirm(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and self._step_pending_state is not None:
                self._step_confirmed = True
                self.get_logger().info(f'[STEP] Confermato passaggio a {self._step_pending_state}')

    def _cb_scan_done(self, msg: Bool) -> None:
        self._search_scan_done = True
        self.get_logger().info('📡 QP controller: scan completato')

    def _cb_step_confirm(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and self._step_pending_state is not None:
                self._step_confirmed = True
                self.get_logger().info(f'[STEP] Confermato passaggio a {self._step_pending_state}')
            if msg.data and self._state == CoordState.WAITING_EXPOSURE:
                self.get_logger().info('Step confirm → EXPOSURE_SCANNING')
                self._set_state(CoordState.EXPOSURE_SCANNING)
            elif msg.data and self._state == CoordState.WAITING_FAST:
                self.get_logger().info('Step confirm → SCANNING')
                self._set_state(CoordState.SCANNING)
            elif msg.data and self._state == CoordState.EXPOSURE_REVIEW:
                self._cb_terminate_exposure(Bool(data=True))

    def _cb_torso_state(self, msg: String) -> None:
        with self._lock:
            self._torso_tracker_state = msg.data

    def _cb_torso_pos(self, msg: PoseStamped) -> None:
        with self._lock:
            self._torso_pos = msg

    def _cb_tf_ready(self, msg: Bool) -> None:
        with self._lock:
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
                self._step_pending_state = None
                self._step_confirmed = False
                self._set_state(CoordState.WAITING_TF, force=True)

    # ── FSM tick ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        with self._lock:
            self._tick_locked()

    def _tick_locked(self) -> None:
        if self._step_mode and self._step_pending_state is not None:
            if self._step_confirmed:
                self.get_logger().info(
                    f'[STEP] Eseguo transizione → {self._step_pending_state}')
                new_state = self._step_pending_state
                self._step_pending_state = None
                self._step_confirmed = False
                self._do_set_state(new_state)
            return

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
        elif self._state == CoordState.EXPOSURE_SCANNING:
            self._tick_exposure()
        elif self._state in (CoordState.WAITING_EXPOSURE,
                             CoordState.WAITING_FAST,
                             CoordState.EXPOSURE_REVIEW):
            pass  # passive wait for step_confirm or click

        s = String(); s.data = self._state
        self._pub_state.publish(s)

        self._tick_fast_settle()
        self._tick_smooth_body_pose()

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
                self._set_state(CoordState.IDLE, force=True)
                self._set_wbc_enabled(False)

    def _tick_idle(self) -> None:
        pass  # IDLE is dead-end — restart requires external intervention

    def _tick_waiting_tf(self) -> None:
        pass  # attesa passiva — _cb_tf_ready farà la transizione

    def _tick_scannning(self) -> None:
        if self._scan_start is None:
            return
        elapsed = (self.get_clock().now() - self._scan_start).nanoseconds * 1e-9
        if elapsed >= self._scan_timeout:
            self.get_logger().error(
                f'SCANNING timeout ({self._scan_timeout:.0f}s) → IDLE')
            self._set_state(CoordState.IDLE)

    def _tick_exposure(self) -> None:
        """Passive tick — wait for exposure_scanner to finish.

        The exposure_scanner node handles all arm movement and
        per-point sequencing via /z1/next_point_idx and /wbc/body_ready.
        The coordinator only monitors for timeout and the /exposure/ready
        signal to transition to SCANNING (FAST).
        """
        if self._exposure_scan_start is None:
            return
        elapsed = (self.get_clock().now() - self._exposure_scan_start
                   ).nanoseconds * 1e-9
        if elapsed >= self._scan_timeout:
            self.get_logger().error(
                f'EXPOSURE_SCANNING timeout ({self._scan_timeout:.0f}s) → IDLE')
            self._set_state(CoordState.IDLE)

    def _cb_exposure_ready(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and self._state == CoordState.EXPOSURE_SCANNING:
                self.get_logger().info('Exposure scan complete → EXPOSURE_REVIEW')
                self._set_state(CoordState.EXPOSURE_REVIEW)

    def _cb_manual_gate(self, msg: Bool) -> None:
        with self._lock:
            self._manual_scan_gate = msg.data
            self._pub_manual_gate.publish(msg)
            mode = 'MANUAL' if msg.data else 'AUTO'
            self.get_logger().info(f'Scan gate set to {mode}')

    def _cb_exposure_grid_points(self, msg: PoseArray) -> None:
        with self._lock:
            if len(msg.poses) < 1:
                return
            points = []
            for pose in msg.poses:
                points.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
            self._exposure_grid_points = points
            self._pub_optimize_request.publish(msg)
            self.get_logger().info(
                f'Exposure grid points received ({len(points)}), published to optimizer')

    def _cb_terminate_exposure(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and self._state == CoordState.EXPOSURE_REVIEW:
                self.get_logger().info('Terminate exposure review')
                if self._manual_scan_gate:
                    self._set_state(CoordState.WAITING_FAST)
                else:
                    self._set_state(CoordState.SCANNING)

    def _tick_approaching(self) -> None:
        if self._get_approach_odom() is None:
            return

        # Timeout check: abort to IDLE if Spot can't reach goal
        if self._approach_start is not None:
            elapsed = (self.get_clock().now() - self._approach_start).nanoseconds * 1e-9
            if elapsed >= self._approach_timeout:
                self.get_logger().error(
                    f'APPROACHING timeout ({self._approach_timeout:.0f}s) '
                    f'— goal unreachable → IDLE')
                self._pub_spot_ctrl.publish(Bool(data=False))
                self._pub_cmd_vel.publish(Twist())
                self._set_state(CoordState.IDLE)
                return

        # Publish arm look-at goal + Spot navigation goal
        goal = self._filtered_goal()
        self._pub_goal.publish(goal)
        self._pub_spot_goal.publish(goal)

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
            # Always do exposure body scan first, then FAST ultrasound
            if self._manual_scan_gate:
                self.get_logger().info(
                    f'Handoff ({self._posture}): dist={dist:.2f}m → WAITING_EXPOSURE (manual gate)')
                self._pub_spot_ctrl.publish(Bool(data=False))
                self._pub_handoff.publish(Bool(data=True))
                self._set_state(CoordState.WAITING_EXPOSURE)
            else:
                self.get_logger().info(
                    f'Handoff ({self._posture}): dist={dist:.2f}m → EXPOSURE_SCANNING')
                self._pub_spot_ctrl.publish(Bool(data=False))
                self._pub_handoff.publish(Bool(data=True))
                self._set_state(CoordState.EXPOSURE_SCANNING)
        else:
            self._pub_spot_ctrl.publish(Bool(data=True))  # navigator active

    def _tick_searching(self) -> None:
        # Cooldown: prevent semi-lock from firing immediately on SEARCHING entry
        if self._search_semi_cooldown > 0:
            self._search_semi_cooldown -= 1

        # FASE 1 — Lock in corso: raccolta campioni
        if self._search_lock_buffer is not None:
            lock_ok = (self._posture == 'LYING'
                       and self._confidence >= self._search_lock_confidence
                       and (z := self._get_approach_odom()) is not None)
            if lock_ok:
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
                and (z := self._get_approach_odom()) is not None:
            self._search_lock_buffer = [z]
            self.get_logger().info(f'Full lock (Orbbec): conf={self._confidence:.2f}')
            self._set_wbc_enabled(False)
            self._set_state(CoordState.LOCKING)
            return

        # FASE 3 — Check semi-lock da RealSense (only between search positions,
        # not during pitch settling or active arm scan, and after cooldown)
        if self._search_semi_cooldown <= 0 \
                and self._search_position_start is None and not self._search_settling \
                and self._torso_tracker_state in ('GUIDING', 'ESTIMATING', 'LOCKED') \
                and self._torso_pos is not None:
            if self._check_realsense_guidance():
                return

        # FASE 4 — Position cycling
        self._tick_search_positions()

    def _tick_semi_locking(self) -> None:
        """Spot ruota+inclina verso corpo, braccio LOOKAT. Settle via TF → dwell Orbbec."""
        now_ns = self.get_clock().now().nanoseconds * 1e-9

        if self._torso_pos is not None:
            self._pub_goal.publish(self._torso_pos)

        # ── Fase A — Ruota Spot via cmd_vel.angular.z, attendi settle TF ──
        if not self._semi_lock_settle_done:
            body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
            if body_tf is not None:
                from tf_transformations import euler_from_quaternion
                q = body_tf.transform.rotation
                _, cur_pitch, cur_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                dy = cur_yaw - self._semi_lock_start_yaw
                dp = cur_pitch - self._semi_lock_start_pitch

                yaw_ok = abs(dy - self._semi_lock_target_yaw) < self._search_semi_lock_yaw_tol
                pitch_ok = abs(dp - self._semi_lock_target_pitch) < self._search_semi_lock_pitch_tol

                if yaw_ok and pitch_ok:
                    self._pub_cmd_vel.publish(Twist())
                    self._semi_lock_settle_done = True
                    self._semi_lock_dwell_start = self.get_clock().now()
                    self.get_logger().info(
                        f'Semi-lock: Spot settled (Δyaw={math.degrees(dy):.1f}° '
                        f'Δpitch={math.degrees(dp):.1f}°) → Orbbec dwell {self._search_semi_lock_dwell:.1f}s')
                    return
                else:
                    t = Twist()
                    if not yaw_ok:
                        error = normalize_angle(
                            (self._semi_lock_start_yaw + self._semi_lock_target_yaw)
                            - cur_yaw)
                        t.angular.z = float(np.clip(
                            self._search_yaw_kp * error,
                            -self._search_max_angular_vel, self._search_max_angular_vel))
                    self._pub_cmd_vel.publish(t)

            elapsed = now_ns - self._semi_lock_entry_time
            if elapsed >= self._search_semi_lock_settle_timeout:
                self._pub_cmd_vel.publish(Twist())
                self._semi_lock_settle_done = True
                self._semi_lock_dwell_start = self.get_clock().now()
                self.get_logger().warn(
                    f'Semi-lock: settle timeout ({self._search_semi_lock_settle_timeout:.1f}s) '
                    f'→ dwell anyway')
            return

        # ── Fase B — Spot è fermo e allineato, Orbbec cerca LYING ──
        if self._posture == 'LYING' \
                and self._confidence >= self._search_lock_confidence \
                and (z := self._get_approach_odom()) is not None:
            self._search_lock_buffer = [z]
            self.get_logger().info('Semi-lock → Full lock (Orbbec)')
            self._set_wbc_enabled(False)
            self._set_state(CoordState.LOCKING)
            return

        dwell_elapsed = now_ns - self._semi_lock_dwell_start.nanoseconds * 1e-9 \
            if self._semi_lock_dwell_start is not None else 0.0
        if dwell_elapsed >= self._search_semi_lock_dwell:
            self.get_logger().info('Semi-lock dwell timeout → resuming search')
            self._search_position_idx = self._search_saved_idx
            self._search_position_start = None
            self._set_state(CoordState.SEARCHING)

    def _tick_locking(self) -> None:
        """Braccio va in home (QP), coordinator raccoglie campioni in parallelo.
        Tollera fino a 10 tick (1s a 10 Hz) di assenza Orbbec prima di arrendersi."""

        # ── NLF trigger + timeout (runs regardless of LYING status) ──
        if self._nlf_trigger_pending:
            elapsed = (self.get_clock().now() - self._nlf_trigger_time).nanoseconds * 1e-9
            if elapsed >= 3.0:
                msg = Bool()
                msg.data = True
                self._pub_nlf_trigger.publish(msg)
                self._nlf_trigger_time = self.get_clock().now()
                self._nlf_trigger_pending = False
                self.get_logger().info('NLF trigger sent (after 3s delay)')
        if self._nlf_prior is None and self._nlf_trigger_time is not None:
            elapsed = (self.get_clock().now() - self._nlf_trigger_time).nanoseconds * 1e-9
            if elapsed > 30.0:
                self.get_logger().warn('NLF timeout (30s) — proceeding without prior')
                self._nlf_prior = 'timeout'

        if self._posture == 'LYING' \
                and self._confidence >= self._search_lock_confidence \
                and (z := self._get_approach_odom()) is not None:
            self._lock_lost_ticks = 0
            if self._search_lock_buffer is None:
                self._search_lock_buffer = []
            if len(self._search_lock_buffer) < self._search_lock_samples:
                self._search_lock_buffer.append(z)
            if len(self._search_lock_buffer) >= self._search_lock_samples \
                    and self._ik_done \
                    and (self._nlf_prior_valid() or self._nlf_prior == 'timeout'):
                target = np.mean(self._search_lock_buffer, axis=0)
                self._quality.set_target(target, self._search_lock_confidence)
                nlf_status = 'NLF prior valid' if self._nlf_prior_valid() else 'NLF timeout'
                self.get_logger().info(
                    f'Locking complete: {self._search_lock_samples} samples, '
                    f'{nlf_status} → PRE_APPROACH')
                self._pre_approach_start = self.get_clock().now()
                self._set_state(CoordState.PRE_APPROACH)
            else:
                # ── Debug: log what's blocking the LOCKING → PRE_APPROACH transition ──
                n = len(self._search_lock_buffer) if self._search_lock_buffer else 0
                if n < self._search_lock_samples:
                    self.get_logger().info(
                        f'🔒 LOCKING: samples {n}/{self._search_lock_samples}',
                        throttle_duration_sec=3.0)
                elif not self._ik_done:
                    self._lock_ik_ticks += 1
                    self.get_logger().info(
                        '🔒 LOCKING: waiting for arm home (ik_done) ...',
                        throttle_duration_sec=2.0)
                elif not self._nlf_prior_valid() and self._nlf_prior != 'timeout':
                    self._lock_nlf_ticks += 1
                    self.get_logger().info(
                        '🔒 LOCKING: waiting for NLF prior ...',
                        throttle_duration_sec=3.0)
            return

        # Orbbec persa — tolleranza prima di arrendersi
        self._lock_lost_ticks += 1
        if self._lock_lost_ticks >= 10:
            self.get_logger().warn(
                f'LOCKING: Orbbec lost for {self._lock_lost_ticks} ticks '
                f'(had {len(self._search_lock_buffer)}/{self._search_lock_samples} samples) '
                f'→ resuming search from current position')
            self._search_lock_buffer = None
            self._lock_lost_ticks = 0
            self._search_position_start = None  # riprendi dalla posizione corrente
            self._set_state(CoordState.SEARCHING)

    def _check_realsense_guidance(self) -> bool:
        """Ruota e inclina Spot verso il corpo rilevato dalla RealSense.
        Salva orientamento corrente e target per il TF-based settle in SEMI_LOCKING."""
        if self._torso_tracker_state not in ('GUIDING', 'ESTIMATING', 'LOCKED') \
                or self._torso_pos is None:
            return False

        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            self.get_logger().warn(
                'Semi-lock: TF odom→body non disponibile → skip',
                throttle_duration_sec=5.0)
            return False

        from tf_transformations import euler_from_quaternion
        q = body_tf.transform.rotation
        roll, start_pitch, start_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        torso_body = self._transform_to_body_frame(self._torso_pos)
        if torso_body is None:
            self.get_logger().warn(
                'Semi-lock: impossibile trasformare torso_pos in body frame → skip',
                throttle_duration_sec=5.0)
            return False

        dir_vec = np.array([torso_body[0] - 0.30, torso_body[1], torso_body[2] - 0.15])
        horiz = math.sqrt(dir_vec[0]**2 + dir_vec[1]**2)

        target_pitch = float(np.clip(math.atan2(-dir_vec[2], horiz), 0.0, 0.26))
        target_yaw = float(math.atan2(dir_vec[1], dir_vec[0]))

        self.get_logger().info(
            f'Semi-lock (RealSense): torso_body=({torso_body[0]:.2f},{torso_body[1]:.2f},{torso_body[2]:.2f}) '
            f'→ yaw={math.degrees(target_yaw):.1f}° pitch={math.degrees(target_pitch):.1f}°')

        self._search_saved_idx = self._search_position_idx
        self._search_position_start = self.get_clock().now()
        self._semi_lock_start_yaw = float(start_yaw)
        self._semi_lock_start_pitch = float(start_pitch)
        self._semi_lock_target_yaw = target_yaw
        self._semi_lock_target_pitch = target_pitch
        self._semi_lock_settle_done = False
        self._semi_lock_dwell_start = None
        self._semi_lock_entry_time = self.get_clock().now().nanoseconds * 1e-9
        self._set_body_pose(self._search_body_height, target_pitch)
        self._set_state(CoordState.SEMI_LOCKING)
        return True

    def _tick_search_positions(self) -> None:
        # ── HOME phase: waiting for arm to reach HOME after all pitches ─
        if self._search_home_phase:
            if self._ik_done:
                self._ik_done = False
                self.get_logger().info('HOME ik_done → stepping forward')
                self._search_home_phase = False
                self._search_step_phase = True
                self._search_step_start = self.get_clock().now()
                t = Twist()
                t.linear.x = float(self._search_step_speed)
                self._pub_cmd_vel.publish(t)
            return

        # ── STEP phase: Spot moving forward timed ─────────────────────
        if self._search_step_phase:
            elapsed = (self.get_clock().now() - self._search_step_start).nanoseconds / 1e9
            step_duration = self._search_step_forward / self._search_step_speed
            if elapsed >= step_duration:
                self._pub_cmd_vel.publish(Twist())
                self._search_step_phase = False
                self._search_step_start = None
                # Reset cycle: back to pitch position 0
                self._search_position_idx = 0
                self._search_position_start = None
                self._search_ik_done_count = 0
                self._search_settling = False
                self._set_wbc_enabled(True)
                self.get_logger().info(
                    f'Step done ({elapsed:.1f}s) → new search cycle')
            return

        # ── All pitches complete → send arm HOME (keep WBC enabled) ──
        if self._search_position_idx >= len(self._search_positions):
            self.get_logger().info('All pitches complete → sending arm HOME')
            self._search_home_phase = True
            self._search_ik_done_count = 0
            self._search_position_start = None
            self._search_settling = False
            # Publish HOME via ik_goal_mux (keep WBC enabled for next cycle)
            home_pose = PoseStamped()
            home_pose.header.frame_id = 'link00'
            home_pose.pose.position.x = 0.144
            home_pose.pose.position.y = -0.005
            home_pose.pose.position.z = 0.52
            home_pose.pose.orientation.x = 0.0182
            home_pose.pose.orientation.y = 0.1521
            home_pose.pose.orientation.z = -0.0217
            home_pose.pose.orientation.w = 0.9880
            self._pub_ik_goal.publish(home_pose)
            return

        # ── Waiting for QP controller scan to complete ──
        if not self._search_settling and self._search_position_start is not None:
            if self._search_scan_done:
                self._search_scan_done = False
                self.get_logger().info(
                    f'Search pos {self._search_position_idx+1}: scan done → next')
                self._search_position_idx += 1
                self._search_position_start = None
                return
            return

        # ── New pitch position: set body_pose and start settling ──
        if self._search_position_start is None and not self._search_settling:
            pos = self._search_positions[self._search_position_idx]
            self._set_body_pose(self._search_body_height, pos['pitch'])
            self._search_settling = True
            self._search_settle_start = self.get_clock().now()
            self.get_logger().info(
                f'Search pos {self._search_position_idx+1}/{len(self._search_positions)}: '
                f'pitch={math.degrees(pos["pitch"]):.0f}°')
            return

        # ── Settling: wait for body pose to stabilize (2s dwell) ──
        if self._search_settling:
            elapsed = (self.get_clock().now() - self._search_settle_start).nanoseconds / 1e9
            if elapsed >= 2.0:
                self._search_settling = False
                self._search_position_start = self.get_clock().now()
                self._search_scan_done = False
                self._ik_done = False
                self._search_ik_done_count = 0
                self.get_logger().info(
                    f'Search pos {self._search_position_idx+1}: pitch settled '
                    f'({elapsed:.1f}s) → arm 7 poses')
            return

    def _get_current_yaw(self) -> float | None:
        """Return current Spot yaw in odom frame [rad], or None if TF unavailable."""
        body_tf = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_tf is None:
            return None
        return _yaw_from_quat(body_tf.transform.rotation)

    def _approach_point_odom_pos(self) -> np.ndarray | None:
        """Return approach point from patient_body TF (legacy wrapper)."""
        return self._get_approach_odom()

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

    def _tick_pre_approach_legacy(self) -> None:
        goal = self._filtered_goal()
        goal.pose.position.z += 0.40  # supine torso ~40cm above ground
        self._pub_goal.publish(goal)

        # Wait for at least 1 RealSense ESTIMATING or LOCKED tick in last 5
        detected_now = self._torso_tracker_state in ('ESTIMATING', 'LOCKED')
        self._torso_detected_ticks.append(detected_now)
        if len(self._torso_detected_ticks) > 5:
            self._torso_detected_ticks = self._torso_detected_ticks[-5:]

        if any(self._torso_detected_ticks):
            positive = sum(self._torso_detected_ticks)
            self.get_logger().info(
                f'RealSense {self._torso_tracker_state} detected '
                f'({positive}/{len(self._torso_detected_ticks)} ticks) → APPROACHING')
            self._pub_spot_ctrl.publish(Bool(data=False))
            self._set_state(CoordState.APPROACHING)
            return

        # Timeout fallback: if RealSense never detects, proceed anyway
        elapsed = (self.get_clock().now() - self._pre_approach_start).nanoseconds * 1e-9
        if elapsed >= self._pre_approach_duration:
            self.get_logger().warn(
                f'PRE_APPROACH timeout ({self._pre_approach_duration:.1f}s) '
                f'— RealSense {self._torso_tracker_state}, proceeding anyway')
            self._pub_spot_ctrl.publish(Bool(data=False))
            self._set_state(CoordState.APPROACHING)
            return

    def _tick_pre_approach(self) -> None:
        """PRE_APPROACH: NLF fast-path (1s gate) or legacy sliding-window fallback."""
        if self._nlf_prior_valid():
            # ── FAST PATH: NLF prior available ──
            # Publish LOOKAT goal from NLF prior immediately
            if self._pre_approach_fast_start is None:
                # First tick: publish blended NLF+YOLO goal and start safety gate timer
                nlf_center = self._torso_center_from_prior()
                target = nlf_center.copy()

                if self._torso_tracker_state in ('ESTIMATING', 'LOCKED') \
                        and self._torso_pos is not None:
                    torso_yolo = np.array([self._torso_pos.pose.position.x,
                                           self._torso_pos.pose.position.y,
                                           self._torso_pos.pose.position.z])
                    quality_label, delta = self._check_nlf_delta(torso_yolo)
                    if quality_label == 'HIGH':
                        target = 0.7 * nlf_center + 0.3 * torso_yolo
                    elif quality_label == 'MEDIUM':
                        target = 0.5 * nlf_center + 0.5 * torso_yolo
                    else:  # LOW
                        target = torso_yolo
                    self.get_logger().info(
                        f'PRE_APPROACH (NLF+YOLO): quality={quality_label} '
                        f'δ={delta:.2f}m → goal published, waiting 1s safety gate')
                else:
                    self.get_logger().info(
                        'PRE_APPROACH (NLF): no RealSense — goal published, waiting 1s safety gate')

                goal = PoseStamped()
                goal.header.frame_id = self._odom_frame
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = float(target[0])
                goal.pose.position.y = float(target[1])
                goal.pose.position.z = float(target[2])
                goal.pose.orientation.w = 1.0
                self._pub_goal.publish(goal)
                self._pre_approach_fast_start = self.get_clock().now()
                return  # stay in PRE_APPROACH for safety gate

            elapsed = (self.get_clock().now() - self._pre_approach_fast_start).nanoseconds * 1e-9
            if elapsed < 1.0:
                return  # still in safety gate

            # Safety gate complete — coherence check (non-blocking)
            if self._torso_tracker_state in ('ESTIMATING', 'LOCKED') \
                    and self._torso_pos is not None:
                torso_yolo = np.array([self._torso_pos.pose.position.x,
                                       self._torso_pos.pose.position.y,
                                       self._torso_pos.pose.position.z])
                quality_label, delta = self._check_nlf_delta(torso_yolo)
                if quality_label == 'HIGH':
                    self.get_logger().info(
                        f'RealSense coherent with NLF prior: {delta:.2f}m')
                elif quality_label == 'MEDIUM':
                    self.get_logger().warn(
                        f'RealSense partially diverging from NLF: {delta:.2f}m')
                else:
                    self.get_logger().warn(
                        f'RealSense diverges from NLF: {delta:.2f}m')

                if quality_label == 'LOW':
                    self._nlf_low_ticks += 1
                    if self._nlf_low_ticks > 30:
                        self.get_logger().warn(
                            'Possible patient movement: NLF prior diverging from YOLO for >3s')
                else:
                    self._nlf_low_ticks = 0

            self._pub_spot_ctrl.publish(Bool(data=False))
            self._set_state(CoordState.APPROACHING)
            self._pre_approach_fast_start = None  # reset
        else:
            # ── FALLBACK: exact 6 June 2026 behavior ──
            self._tick_pre_approach_legacy()

    # ── NLF prior handlers ────────────────────────────────────────────

    def _cb_nlf_prior(self, msg: PoseArray) -> None:
        if self._nlf_prior == 'timeout':
            return
        if len(msg.poses) != 24:
            self.get_logger().warn(
                f'NLF prior: expected 24 joints, got {len(msg.poses)} → ignoring')
            return

        # Lookup TF: orbbec_color_optical_frame → odom
        from geometry_msgs.msg import TransformStamped
        transform: TransformStamped | None = None
        try:
            transform = self._tf.lookup_transform(
                'odom', 'orbbec_color_optical_frame', msg.header.stamp,
                timeout=Duration(seconds=0.5))
        except TransformException:
            self.get_logger().warn(
                'NLF prior: TF at msg stamp failed, trying latest')
            try:
                transform = self._tf.lookup_transform(
                    'odom', 'orbbec_color_optical_frame', rclpy.time.Time(),
                    timeout=Duration(seconds=0.5))
            except TransformException:
                self.get_logger().warn(
                    'NLF prior: TF odom←orbbec unavailable → cannot transform')

        if transform is None:
            return

        R = quat_to_rot(transform.transform.rotation)
        t = np.array([transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z])

        joints_odom = []
        for pose in msg.poses:
            p_cam = np.array([pose.position.x, pose.position.y, pose.position.z])
            joints_odom.append(R @ p_cam + t)

        self._nlf_prior = joints_odom
        self.get_logger().info('NLF prior received: 24 joints in odom')

        # Pause NLF streaming now that prior is captured
        msg = Bool()
        msg.data = False
        self._pub_nlf_trigger.publish(msg)
        self.get_logger().info('NLF streaming paused after prior capture')

    def _cb_nlf_confidence(self, msg: Float32) -> None:
        self._nlf_confidence = float(msg.data)

    def _cb_skeleton_stream(self, msg: PoseArray) -> None:
        pass  # stub: future Quality Monitor integration

    def _nlf_prior_valid(self) -> bool:
        if self._nlf_prior is None:
            return False
        if self._nlf_prior == 'timeout':
            return False
        if len(self._nlf_prior) != 24:
            return False
        valid_torso = sum(1 for j in [SPINE1, SPINE2, SPINE3, PELVIS]
                          if not np.any(np.isnan(self._nlf_prior[j])))
        return valid_torso >= 4

    def _torso_center_from_prior(self) -> np.ndarray | None:
        # Try NLF spine keypoints first
        if self._nlf_prior is not None:
            spine_indices = [SPINE1, SPINE2, SPINE3, PELVIS]
            points = []
            for idx in spine_indices:
                kp = self._nlf_prior[idx]
                if not any(np.isnan(kp)):
                    points.append(kp)
            if points:
                return np.mean(points, axis=0)
        # FALLBACK: use patient_body TF (works even with YOLO NaN)
        return self._get_approach_odom()

    def _check_nlf_delta(self, torso_yolo: np.ndarray) -> tuple:
        """Compare YOLO torso position against NLF prior. Returns (label, delta_m)."""
        if not self._nlf_prior_valid():
            return ('HIGH', None)

        nlf_center = self._torso_center_from_prior()
        delta = float(np.linalg.norm(torso_yolo[:3] - nlf_center[:3]))

        coherence = self.get_parameter('nlf_coherence_threshold').value
        divergence = self.get_parameter('nlf_divergence_threshold').value

        if delta < coherence:
            return ('HIGH', delta)
        elif delta < divergence:
            return ('MEDIUM', delta)
        else:
            return ('LOW', delta)

    # ── Helpers ───────────────────────────────────────────────────────

    def _tf_lookup(self, source: str, target: str,
                   timeout_sec: float = 10.0) -> TransformStamped | None:
        try:
            return self._tf.lookup_transform(
                source, target, rclpy.time.Time(),
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
        approach = self._get_approach_odom()
        if approach is None:
            return None
        body_in_odom = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_in_odom is None:
            return None
        dx = body_in_odom.transform.translation.x - approach[0]
        dy = body_in_odom.transform.translation.y - approach[1]
        return math.hypot(dx, dy)

    def _filtered_goal(self) -> PoseStamped:
        """Return the fixed target position in odom frame (NLF-blended when prior valid)."""
        msg = PoseStamped()
        msg.header.frame_id = self._odom_frame
        msg.header.stamp    = rclpy.time.Time().to_msg()
        msg.pose.orientation.w = 1.0  # identity — WBC QP recomputes orientation
        if self._quality.initialized:
            p = self._quality.get_position()
            # ── NLF prior blending ──────────────────────────────
            if self._nlf_prior_valid():
                # EXCELLENT tier: NLF confidence ≥ threshold → 100% NLF, skip delta blending
                if self._nlf_confidence >= self._nlf_excellent_conf:
                    nlf_center = self._torso_center_from_prior()
                    p = nlf_center.copy()
                    self._nlf_low_ticks = 0
                else:
                    quality_label, delta = self._check_nlf_delta(p)
                    nlf_center = self._torso_center_from_prior()
                    if quality_label == 'HIGH':
                        p = 0.7 * nlf_center + 0.3 * p
                    elif quality_label == 'MEDIUM':
                        p = 0.5 * nlf_center + 0.5 * p
                    # LOW → keep p as-is (YOLO/Orbbec 100%)

                if quality_label == 'LOW':
                    self._nlf_low_ticks += 1
                    if self._nlf_low_ticks > 30:
                        self.get_logger().warn(
                            'Possible patient movement: NLF prior diverging for >3s')
                else:
                    self._nlf_low_ticks = 0

            msg.pose.position.x = float(p[0])
            msg.pose.position.y = float(p[1])
            msg.pose.position.z = float(p[2])
        elif (approach := self._get_approach_odom()) is not None:
            msg.pose.position.x = approach[0]
            msg.pose.position.y = approach[1]
            msg.pose.position.z = approach[2]
        else:
            return PoseStamped()  # should not happen
        return msg

    # ── FAST body pose optimization ────────────────────────────────────

    def _cb_fast_points(self, msg: PoseArray) -> None:
        with self._lock:
            if len(msg.poses) < 1:
                return
            self._fast_points = msg
            self.get_logger().info(f'FAST points received ({len(msg.poses)} poses)')
            self._pub_optimize_request.publish(msg)
            self.get_logger().info(
                f'Published {len(msg.poses)} points to ~/optimize_request')

    def _cb_optimizer_result(self, msg: PoseArray) -> None:
        """Receive optimizer results: PoseArray where each Pose encodes (h, p, dx, dy).

        pose.position.x → h  (body height offset)
        pose.position.y → p  (body pitch)
        pose.position.z → dx (lateral displacement)
        pose.orientation.x → dy (forward/backward displacement)
        """
        with self._lock:
            self._optimizer_results = []
            for pose in msg.poses:
                h = float(pose.position.x)
                p = float(pose.position.y)
                dx = float(pose.position.z)
                dy = float(pose.orientation.x)
                self._optimizer_results.append((h, p, dx, dy))
            self.get_logger().info(
                f'Optimizer results received ({len(self._optimizer_results)} poses)')

    def _cb_next_point(self, msg: Int32) -> None:
        with self._lock:
            idx = msg.data
            if idx < 0:
                kind = 'EXPOSURE' if self._state == CoordState.EXPOSURE_SCANNING else 'FAST'
                self.get_logger().info(f'{kind} done — restoring handoff height')
                self._set_body_pose(self._handoff_body_height, 0.0)
                self._pub_body_ready.publish(Bool(data=True))
                return
            if self._optimizer_results and idx < len(self._optimizer_results):
                h, p, dx, dy = self._optimizer_results[idx]
                self.get_logger().info(
                    f'Point {idx} body pose from optimizer: '
                    f'h={h:.2f}m, p={math.degrees(p):.1f}°')
                self._set_body_pose(h, p)
                self._body_settle_start = self.get_clock().now().nanoseconds * 1e-9
            else:
                self._pub_body_ready.publish(Bool(data=True))

    def _max_workspace_reach(self) -> float:
        """Estimate max reachable distance from link00 in meters."""
        return self._max_reach_val

    def _tick_fast_settle(self) -> None:
        """Monitor body pose settle time."""
        if self._body_settle_start is None:
            return

        elapsed = (self.get_clock().now().nanoseconds * 1e-9
                   - self._body_settle_start)
        if elapsed >= self._body_settle_time:
            self.get_logger().info(
                f'Body settle complete ({elapsed:.1f}s ≥ {self._body_settle_time:.1f}s)')
            self._body_settle_start = None
            self._pub_body_ready.publish(Bool(data=True))

    def _set_state(self, new_state: str, force: bool = False) -> None:
        if new_state == self._state:
            return
        if self._step_mode and not force:
            skip_gate = (
                (self._state == CoordState.IDLE and new_state == CoordState.SEARCHING) or
                new_state in (CoordState.IDLE, CoordState.WAITING_TF) or
                (self._state == CoordState.SEMI_LOCKING and new_state == CoordState.SEARCHING) or
                (self._state == CoordState.LOCKING and new_state == CoordState.SEARCHING)
            )
            if not skip_gate:
                self._step_pending_state = new_state
                old_name = str(self._state)
                msg = String()
                msg.data = f'{old_name} → {new_state}'
                self._pub_step_pending.publish(msg)
                self.get_logger().info(
                    f'[STEP] Transizione in attesa: {old_name} → {new_state}. '
                    'Premi "n" sul keyboard controller per confermare.')
                return
        self._do_set_state(new_state)

    def _do_set_state(self, new_state: str) -> None:
        self.get_logger().info(f'WBC FSM: {self._state} → {new_state}')
        if new_state == CoordState.WAITING_TF:
            self._set_wbc_enabled(False)
            self._pub_cmd_vel.publish(Twist())
            self._pub_guidance.publish(Bool(data=False))
            self._set_body_pose(0.0)
        if new_state == CoordState.IDLE:
            self._quality.reset()
            self._pub_cmd_vel.publish(Twist())
            self._pub_guidance.publish(Bool(data=False))
            self._set_body_pose(0.0)   # ripristina altezza nominale
            self._pub_nav_mode.publish(String(data='approaching'))
            self._pub_perception_enable.publish(Bool(data=False))
        if new_state == CoordState.SEARCHING:
            self._pub_guidance.publish(Bool(data=True))
            self._pub_perception_enable.publish(Bool(data=True))
            old = self._state
            self._search_lock_buffer = None
            self._set_wbc_enabled(True)
            # Fresh start or re-entry: reset cooldown to allow pitch cycling before semi-lock
            self._search_semi_cooldown = 3  # 3 ticks (0.3s at 10Hz) before FASE 3 can fire
            if old == CoordState.IDLE:
                # Fresh start: full reset
                self._search_start = self.get_clock().now()
                self._search_positions = self._build_search_sequence()
                self._search_position_idx = 0
                self._search_position_start = None
                self._search_saved_idx = 0
                self._search_settling = False
                self._search_home_phase = False
                self._search_step_phase = False
                self._search_step_start = None
            else:
                # Re-entry: stop residual cmd_vel, reset phases, resume from current position
                self._pub_cmd_vel.publish(Twist())
                self._search_home_phase = False
                self._search_step_phase = False
                self._search_step_start = None
                self._search_scan_done = False
                self._search_settling = False
            # else: re-entry from LOCKING or SEMI_LOCKING — resume, don't rebuild
        if new_state == CoordState.SCANNING:
            self._set_body_pose(self._handoff_body_height)
            self._scan_start = self.get_clock().now()
        if new_state == CoordState.EXPOSURE_SCANNING:
            self._set_body_pose(self._handoff_body_height)
            self._exposure_scan_start = self.get_clock().now()
            self._pub_nav_mode.publish(String(data='exposure'))
            self.get_logger().info('Entering exposure body scan')
        if new_state == CoordState.WAITING_EXPOSURE:
            self._pub_step_pending.publish(String(data='EXPOSURE_SCANNING'))
            self.get_logger().info('WAITING_EXPOSURE — press n or click Start Exposure')
        if new_state == CoordState.WAITING_FAST:
            self._pub_step_pending.publish(String(data='SCANNING'))
            self.get_logger().info('WAITING_FAST — press n or click Start FAST')
        if new_state == CoordState.EXPOSURE_REVIEW:
            self._pub_step_pending.publish(String(data='EXPOSURE_REVIEW'))
            self.get_logger().info(
                'EXPOSURE_REVIEW — click grid points to re-inspect, '
                'n or Terminate to finish')
        if new_state == CoordState.LOCKING:
            self._pub_cmd_vel.publish(Twist())
            self._pub_guidance.publish(Bool(data=False))
            self._search_settling = False
            self._ik_done = False
            self._lock_ik_ticks = 0
            self._lock_nlf_ticks = 0
            self._set_wbc_enabled(True)
            # Trigger NLF prior if no valid prior already cached (debounce)
            if not self._nlf_prior_valid():
                self._nlf_prior = None
                self._nlf_trigger_time = self.get_clock().now()
                self._nlf_trigger_pending = True
                self.get_logger().info('NLF trigger queued (3s delay for model loading)')
        if new_state == CoordState.APPROACHING:
            self._pub_cmd_vel.publish(Twist())
            self._pub_guidance.publish(Bool(data=False))
            self._pub_spot_ctrl.publish(Bool(data=False))
            self._approach_start = self.get_clock().now()
        if new_state == CoordState.PRE_APPROACH:
            self._pub_cmd_vel.publish(Twist())
            self._pub_guidance.publish(Bool(data=False))
            self._set_body_pose(0.0, 0.0)
            self._torso_detected_ticks = []
            self._pre_approach_fast_start = None
        self._state = new_state

    def _set_body_pose(self, height: float, pitch: float = 0.0, yaw: float | None = None, smooth: bool = True) -> None:
        """Publish body_pose with optional smooth interpolation.

        When smooth=True (default), starts an async transition from current
        (h,p) to target over 0.8s, driven by _tick_smooth_body_pose() to
        avoid abrupt posture changes that destabilize Spot.
        """
        from tf_transformations import quaternion_from_euler
        if yaw is None:
            cur_yaw = self._get_current_yaw()
            yaw = cur_yaw if cur_yaw is not None else 0.0
        height_clamped = float(np.clip(height, self._min_body_height, self._max_body_height))

        if smooth and (abs(height_clamped - self._current_body_height) > 0.02
                       or abs(pitch - (self._smooth_pitch if hasattr(self, '_smooth_pitch') else 0.0)) > 0.01):
            self._smooth_start_h = self._current_body_height
            self._smooth_start_p = getattr(self, '_smooth_pitch', 0.0)
            self._smooth_target_h = height_clamped
            self._smooth_target_p = pitch
            self._smooth_target_yaw = yaw
            self._smooth_start_time = self.get_clock().now()
            self._smoothing_body = True
            self._smooth_duration = 0.8
        else:
            self._publish_body_pose_raw(height_clamped, pitch, yaw)
            self._current_body_height = height_clamped
            self._smooth_pitch = pitch

    def _tick_smooth_body_pose(self):
        """Called from timer callback: publish interpolated body pose."""
        if not getattr(self, '_smoothing_body', False):
            return
        elapsed = (self.get_clock().now() - self._smooth_start_time).nanoseconds * 1e-9
        if elapsed >= self._smooth_duration:
            self._publish_body_pose_raw(self._smooth_target_h, self._smooth_target_p, self._smooth_target_yaw)
            self._current_body_height = self._smooth_target_h
            self._smooth_pitch = self._smooth_target_p
            self._smoothing_body = False
        else:
            t = elapsed / self._smooth_duration
            h = self._smooth_start_h + (self._smooth_target_h - self._smooth_start_h) * t
            p = self._smooth_start_p + (self._smooth_target_p - self._smooth_start_p) * t
            self._publish_body_pose_raw(h, p, self._smooth_target_yaw)

    def _publish_body_pose_raw(self, height: float, pitch: float, yaw: float):
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, pitch, yaw)
        pose = Pose()
        pose.position.z = height
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self._pub_body_pose.publish(pose)
        # Workaround spot_ros2 bug: body_pose is accepted but not applied
        # until a cmd_vel (even zero) triggers the driver to actuate
        self._pub_cmd_vel.publish(Twist())

    def _read_float_array(self, param_name: str) -> list:
        val = self.get_parameter(param_name).value
        if isinstance(val, list):
            return [float(v) for v in val]
        return [float(val)]

    def _build_search_sequence(self) -> list:
        """Build pitch-based search sequence. Each step: set body pitch, then arm does 7 poses.
        Pitch values: +10°, +5°, 0° (nose-down first, then neutral) in radians.
        After all 3 pitches + 7 arm poses each: HOME → step forward 50cm."""
        pitch_angles = [math.radians(10.0), math.radians(5.0), 0.0]
        return [{'yaw': 0.0, 'pitch': p} for p in pitch_angles]

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
