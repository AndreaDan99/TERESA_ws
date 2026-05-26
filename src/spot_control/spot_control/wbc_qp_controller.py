#!/usr/bin/env python3
"""
WBC QP Controller — arm-only look-at + QP-based scanning + QP-based search.

Modes (selected automatically from /wbc/state):
  SEARCH_GRID — SEARCHING:    genera 7 pose esplorative dal null-space (loop infinito)
  LOOKAT      — PRE_APPROACH: ω_des orientamento + null-space joint centering
  SCAN_SEQ    — APPROACHING:  genera 11 pose dal null-space, le sequenzia,
                               raccoglie dati, pubblica /z1/fast_points

State transitions:
  SEARCHING     → _start_search()
  SEMI_LOCKING  → _pause_search()   (blocca il braccio, Orbbec cerca)
  LOCKING       → _end_search() + _send_home()
  SEARCHING (ripresa) → _resume_search()
"""

import math
import time

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float32MultiArray

from tf2_ros import Buffer, TransformListener, TransformException

from teresa_utils.orientation import (
    compute_ee_orientation, compute_ee_orientation_minrot,
)

from spot_control.wbc_math import damped_pinv, null_space_projector, manipulability
from z1_vision.workspace_checker import WorkspaceChecker
from z1_vision.body_search_scanner import BodySearchScanner, ScanAction, ScanTick

JOINT_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# ── Safe joint limits for SEARCH_GRID (more restrictive than Z1 native) ───────
# Restrict extreme poses to avoid hitting Spot body, ground, or unbalancing.
SAFE_Q_LOW  = [-0.5, -0.3, -1.8,  0.5, -0.5, -0.8]
SAFE_Q_HIGH = [ 0.5,  0.8, -0.3,  2.0,  0.5,  0.8]

# ── Scan parameters ────────────────────────────────────────────────────────────
SCAN_POINT_TIMEOUT = 4.0      # [s] tempo raccolta dati per posa (APPROACHING)
SCAN_MIN_FRAMES    = 5        # frame minimi per posa
SCAN_EARLY_STOP    = 0.95     # early-stop detection score
SCAN_STABILITY_K   = 10.0     # penalty stabilità 3D

# ── Search parameters ──────────────────────────────────────────────────────────
SEARCH_POINT_TIMEOUT = 2.0    # [s] tempo raccolta per posa (SEARCHING)
SEARCH_MIN_FRAMES    = 3      # frame minimi
SEARCH_EARLY_STOP    = 0.6    # early-stop liberale

HOME_POS = np.array([-0.09, 0.0, 0.44])
HOME_ORI = np.array([-0.0062, 0.4107, 0.0021, 0.9118])


def _make_pose_stamped(pos: np.ndarray, orientation: np.ndarray,
                        frame_id: str = 'world') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x    = float(pos[0])
    p.pose.position.y    = float(pos[1])
    p.pose.position.z    = float(pos[2])
    p.pose.orientation.x = float(orientation[0])
    p.pose.orientation.y = float(orientation[1])
    p.pose.orientation.z = float(orientation[2])
    p.pose.orientation.w = float(orientation[3])
    return p


class WBCQPControllerNode(Node):

    def __init__(self):
        super().__init__('wbc_qp_controller')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('dry_run',        False)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('odom_frame',    'my_spot/odom')
        self.declare_parameter('body_frame',    'my_spot/body')
        self.declare_parameter('z1_base_frame', 'world')
        self.declare_parameter('ee_frame',      'link06')
        self.declare_parameter('kp_ang',        1.5)
        self.declare_parameter('k_null',        0.3)
        self.declare_parameter('damping',       1e-3)
        self.declare_parameter('q_dot_max',     0.6)
        self.declare_parameter('search_delta',  0.15)
        self.declare_parameter('scan_delta',    0.12)
        self.declare_parameter('update_period', 0.1)
        self.declare_parameter('workspace_safety_margin', 0.05)
        self.declare_parameter('ik_goal_topic',      '/wbc/ik_goal_pose')
        self.declare_parameter('ik_enable_topic',    '/wbc/ik_enable')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('home_orientation', [-0.0062, 0.4107, 0.0021, 0.9118])
        self.declare_parameter('orientation_mode', 'minrot')

        p = lambda n: self.get_parameter(n).value
        self._dry_run       = bool(p('dry_run'))
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._z1_base_frame = p('z1_base_frame')
        self._ee_frame      = p('ee_frame')
        self._kp_ang        = float(p('kp_ang'))
        self._k_null        = float(p('k_null'))
        self._damping       = float(p('damping'))
        self._q_dot_max     = float(p('q_dot_max'))
        self._search_delta  = float(p('search_delta'))
        self._scan_delta    = float(p('scan_delta'))
        self._update_period = float(p('update_period'))
        self._home_orientation = np.array([float(x) for x in p('home_orientation')])
        self._orientation_mode = p('orientation_mode')

        # ── Pinocchio ─────────────────────────────────────────────────────
        urdf = p('urdf_path')
        if not urdf:
            import os
            try:
                from ament_index_python.packages import get_package_share_directory
                urdf = os.path.join(get_package_share_directory('z1_description'),
                                    'urdf', 'z1.urdf')
            except Exception:
                urdf = os.path.expanduser(
                    '~/Ros2_repositories/unitree_z1_ws/install/z1_description'
                    '/share/z1_description/urdf/z1.urdf')
        self.get_logger().info(f'URDF: {urdf}')
        self._model = pin.buildModelFromUrdf(urdf)
        self._data  = self._model.createData()
        self._ee_id = self._model.getFrameId(self._ee_frame)
        self._q_neutral = pin.neutral(self._model)

        self._ws_checker = WorkspaceChecker(
            urdf,
            ee_frame=self._ee_frame,
            safety_margin=float(p('workspace_safety_margin')),
        )

        # ── TF ────────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── State ─────────────────────────────────────────────────────────
        self._enabled        = False
        self._goal: PoseStamped | None = None
        self._q_meas: np.ndarray | None = None
        self._tf_ready       = False
        self._mode           = 'LOOKAT'   # 'LOOKAT' | 'SCAN_SEQ' | 'SEARCH_GRID'
        self._wbc_state      = ''
        self._search_paused  = False     # True when paused for SEMI_LOCKING

        # ── Scan state ────────────────────────────────────────────────────
        self._scan_scanner: BodySearchScanner | None = None
        self._scan_ik_done       = False
        self._scan_data_queue: list[list[float]] = []
        self._scan_torso_est: np.ndarray | None = None
        self._scan_poses: list[PoseStamped] = []

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Bool,        '/wbc/enable',          self._cb_enable,      10)
        self.create_subscription(PoseStamped, '/wbc/ee_goal',         self._cb_goal,        10)
        self.create_subscription(JointState,  p('joint_states_topic'), self._cb_joints,      50)
        self.create_subscription(String,      '/wbc/state',           self._cb_wbc_state,   10)
        self.create_subscription(Bool,        '/ik_done',             self._cb_ik_done,     10)
        self.create_subscription(Float32MultiArray, '/torso_scan_point',
                                  self._cb_scan_data, 10)

        # ── Publishers ────────────────────────────────────────────────────
        if self._dry_run:
            self._pub_ik  = self.create_publisher(PoseStamped, '/wbc/ik_goal_pose_debug', 10)
            self._pub_en  = self.create_publisher(Bool,        '/wbc/ik_enable_debug',    10)
            self.get_logger().warn('DRY_RUN mode')
        else:
            self._pub_ik  = self.create_publisher(PoseStamped, p('ik_goal_topic'),   10)
            self._pub_en  = self.create_publisher(Bool,        p('ik_enable_topic'), 10)

        self._pub_fast       = self.create_publisher(PoseArray, '/z1/fast_points', 10)
        self._pub_fast_ready = self.create_publisher(Bool,      '/z1/fast_ready',  10)
        self._pub_tracker_scan = self.create_publisher(Bool, '/tracker_scan_mode', 10)
        self._pub_tracker_reset = self.create_publisher(Bool, '/tracker_reset',    10)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(self._update_period, self._update)
        self.get_logger().info('WBC QP Controller ready (arm-only).')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_enable(self, msg: Bool) -> None:
        self._enabled = msg.data
        if not self._enabled:
            en = Bool(); en.data = False
            self._pub_en.publish(en)
            if self._mode == 'SCAN_SEQ':
                self._end_scan()
            elif self._mode == 'SEARCH_GRID':
                self._end_search()

    def _cb_goal(self, msg: PoseStamped) -> None:
        self._goal = msg

    def _cb_joints(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._q_meas = np.array([name_to_pos[j] for j in JOINT_ORDER])
        except KeyError:
            pass

    def _cb_wbc_state(self, msg: String) -> None:
        prev = self._wbc_state
        self._wbc_state = msg.data

        if msg.data == 'SEARCHING':
            if prev == 'SEMI_LOCKING':
                self._resume_search()
            elif prev != 'SEARCHING':
                self._start_search()
        elif msg.data == 'SEMI_LOCKING' and self._mode == 'SEARCH_GRID':
            self._pause_search()
        elif msg.data == 'LOCKING' and self._mode == 'SEARCH_GRID':
            self._end_search()
            self._send_home()
        elif msg.data == 'APPROACHING' and prev != 'APPROACHING':
            self._start_scan()
        elif msg.data not in ('SEARCHING', 'SEMI_LOCKING', 'LOCKING', 'APPROACHING'):
            if self._mode == 'SEARCH_GRID':
                self._end_search()
            if self._mode == 'SCAN_SEQ':
                self._end_scan()

    def _cb_ik_done(self, msg: Bool) -> None:
        self._scan_ik_done = msg.data

    def _cb_scan_data(self, msg: Float32MultiArray) -> None:
        self._scan_data_queue.append(list(msg.data))

    # ── TF helpers ────────────────────────────────────────────────────────

    def _tf_lookup(self, source: str, target: str,
                    timeout_sec: float = 1.0) -> TransformStamped | None:
        try:
            return self._tf.lookup_transform(
                source, target, rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec))
        except TransformException as e:
            if not self._tf_ready:
                self.get_logger().warn(
                    f'TF {source} → {target} non disponibile.',
                    throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(
                    f'TF {source} → {target} persa ({e})',
                    throttle_duration_sec=2.0)
            return None

    def _tf_transform(self, pose: PoseStamped, target_frame: str,
                       timeout_sec: float = 1.0) -> PoseStamped | None:
        try:
            return self._tf.transform(
                pose, target_frame, timeout=Duration(seconds=timeout_sec))
        except TransformException:
            return None

    # ── Mode dispatch ─────────────────────────────────────────────────────

    def _update(self) -> None:
        if self._mode == 'SCAN_SEQ':
            self._tick_scan()
            return
        if self._mode == 'SEARCH_GRID':
            if not self._search_paused:
                self._tick_search()
            return
        # LOOKAT mode
        self._tick_lookat()

    # ── LOOKAT mode (PRE_APPROACH) ────────────────────────────────────────

    def _tick_lookat(self) -> None:
        if not self._enabled or self._goal is None or self._q_meas is None:
            return

        # 1. EE pose in body frame
        ee_in_body = self._tf_lookup(self._body_frame, self._ee_frame)
        if ee_in_body is None:
            return

        # 2. Body position in odom
        body_in_odom = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_in_odom is None:
            return

        # 3. Compose EE in odom
        _q = body_in_odom.transform.rotation
        _qv = np.array([_q.x, _q.y, _q.z])
        _qw = float(_q.w)
        _p_eeb = np.array([ee_in_body.transform.translation.x,
                           ee_in_body.transform.translation.y,
                           ee_in_body.transform.translation.z])
        _p_eeb_rot = _p_eeb + 2.0 * np.cross(_qv, np.cross(_qv, _p_eeb) + _qw * _p_eeb)
        _p_eeodom = np.array([body_in_odom.transform.translation.x,
                              body_in_odom.transform.translation.y,
                              body_in_odom.transform.translation.z]) + _p_eeb_rot

        if not self._tf_ready:
            self._tf_ready = True
            self.get_logger().info('TF disponibile — SpotCore connesso via DDS.')

        # 4. Resolve goal to link00 frame
        goal_in = self._goal
        goal_frame = goal_in.header.frame_id
        if goal_frame in ('world', 'link00', self._z1_base_frame):
            goal_link00 = goal_in
        else:
            goal_stamped = PoseStamped()
            goal_stamped.header.frame_id = goal_frame
            goal_stamped.header.stamp = rclpy.time.Time().to_msg()
            goal_stamped.pose = goal_in.pose
            goal_link00 = self._tf_transform(goal_stamped, self._z1_base_frame)
            if goal_link00 is None:
                return

        # 5. FK + Jacobian at current joint config
        n_arm = self._q_meas.shape[0]
        q = self._q_neutral.copy()
        q[:n_arm] = self._q_meas
        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        T_ee = self._data.oMf[self._ee_id]
        J_arm_full = pin.getFrameJacobian(
            self._model, self._data, self._ee_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J_arm_full[:, :n_arm]

        # 6. Orientation error → ω_des (look-at: X_ee points to target)
        x_current = T_ee.rotation[:, 0]
        p_ee = T_ee.translation
        target_link00 = np.array([goal_link00.pose.position.x,
                                   goal_link00.pose.position.y,
                                   goal_link00.pose.position.z])
        x_desired = target_link00 - p_ee
        x_norm = float(np.linalg.norm(x_desired))
        if x_norm < 1e-6:
            x_desired = np.array([1.0, 0.0, 0.0])
        else:
            x_desired = x_desired / x_norm

        axis = np.cross(x_current, x_desired)
        sin_a = float(np.linalg.norm(axis))
        cos_a = float(np.clip(np.dot(x_current, x_desired), -1.0, 1.0))
        angle = math.atan2(sin_a, cos_a)

        if sin_a < 1e-6:
            ω_des = np.zeros(3)
        else:
            ω_des = self._kp_ang * angle * (axis / sin_a)

        # 7. Task Jacobian (angular part only, 3×6) + damped pseudo-inverse
        J_task = J_arm[:3, :]
        m = manipulability(J_arm)
        damp_adaptive = self._damping * (1.0 + 1.0 / (m + 1e-4))
        J_pinv = damped_pinv(J_task, damp_adaptive)

        # 8. Null-space projector + joint centering
        N = null_space_projector(J_task, J_pinv)
        q_low = self._model.lowerPositionLimit[:n_arm]
        q_high = self._model.upperPositionLimit[:n_arm]
        q_mid = (q_low + q_high) / 2.0
        q_dot_null = self._k_null * (q_mid - self._q_meas)
        q_dot_null = np.clip(q_dot_null, -self._q_dot_max, self._q_dot_max)

        # 9. Combined q_dot + FK prediction
        q_dot = J_pinv @ ω_des + N @ q_dot_null
        q_new = q.copy()
        q_new[:n_arm] = np.clip(q[:n_arm] + q_dot * self._update_period,
                                 q_low, q_high)
        pin.forwardKinematics(self._model, self._data, q_new)
        pin.updateFramePlacements(self._model, self._data)
        T_new = self._data.oMf[self._ee_id]

        # 10. Workspace clipping
        ws_pos = np.array([T_new.translation[0], T_new.translation[1],
                           T_new.translation[2]])
        clipped_pos, was_clipped, _ = self._ws_checker.clip_target(ws_pos)
        if was_clipped:
            self.get_logger().warn(
                f'WBC goal clipped: [{ws_pos[0]:.3f},{ws_pos[1]:.3f},{ws_pos[2]:.3f}] '
                f'→ [{clipped_pos[0]:.3f},{clipped_pos[1]:.3f},{clipped_pos[2]:.3f}]',
                throttle_duration_sec=3.0)

        # 11. Publish IK goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'
        goal_msg.pose.position.x = float(clipped_pos[0])
        goal_msg.pose.position.y = float(clipped_pos[1])
        goal_msg.pose.position.z = float(clipped_pos[2])

        quat = (compute_ee_orientation_minrot(x_desired, self._home_orientation.tolist())
                if self._orientation_mode == 'minrot'
                else compute_ee_orientation(x_desired, self._home_orientation.tolist()))
        goal_msg.pose.orientation.x = float(quat[0])
        goal_msg.pose.orientation.y = float(quat[1])
        goal_msg.pose.orientation.z = float(quat[2])
        goal_msg.pose.orientation.w = float(quat[3])

        en = Bool(); en.data = True
        self._pub_en.publish(en)
        self._pub_ik.publish(goal_msg)

        self.get_logger().info(
            f'LOOKAT: m={m:.3f} |ω|={np.linalg.norm(ω_des):.3f} '
            f'angle={math.degrees(angle):.1f}°',
            throttle_duration_sec=2.0)

    # ── SEARCH_GRID mode (SEARCHING) ──────────────────────────────────────

    def _start_search(self) -> None:
        self._mode = 'SEARCH_GRID'
        self._search_paused = False
        self._scan_ik_done = False
        self._scan_data_queue.clear()
        self._gen_and_start_search_scanner()

    def _gen_and_start_search_scanner(self) -> None:
        """Genera 7 pose esplorative dal null-space del 'guarda avanti'."""
        if self._q_meas is None:
            self.get_logger().warn('_gen_search_scanner: no joint state')
            return

        poses = self._gen_search_poses()
        if not poses:
            self.get_logger().warn('_gen_search_scanner: no poses generated')
            return

        self._scan_scanner = BodySearchScanner(
            scan_poses=poses,
            scan_point_timeout=SEARCH_POINT_TIMEOUT,
            scan_min_frames=SEARCH_MIN_FRAMES,
            early_stop_score=SEARCH_EARLY_STOP,
            logger=self.get_logger(),
            stability_k=SCAN_STABILITY_K,
        )
        self._scan_scanner.reset()
        self._pub_tracker_scan.publish(Bool(data=True))
        self.get_logger().info(f'SEARCH_GRID: {len(poses)} poses, delta={self._search_delta:.2f}')

    def _gen_search_poses(self) -> list[PoseStamped]:
        """Genera pose esplorative dal null-space di 'guarda avanti' (body X).
        Usa safe joint limits e delta di esplorazione più ampio.
        Solo 7 pose (home + ±δ×3, senza diagonali)."""
        if self._q_meas is None:
            self.get_logger().warn('_gen_search_poses: no joint state')
            return []

        n_arm = self._q_meas.shape[0]
        q = self._q_neutral.copy()
        q[:n_arm] = self._q_meas

        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        T_ee = self._data.oMf[self._ee_id]
        p_ee = T_ee.translation
        J_arm_full = pin.getFrameJacobian(
            self._model, self._data, self._ee_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J_arm_full[:, :n_arm]

        # Target virtuale: body X (avanti)
        x_desired = np.array([1.0, 0.0, 0.0])

        # Angular Jacobian (3×6) + null-space projector
        J_task = J_arm[:3, :]
        J_pinv = damped_pinv(J_task)
        N = null_space_projector(J_task, J_pinv)

        # SVD → basis of null-space
        _, S, Vt = np.linalg.svd(N)
        rank = int(np.sum(S > 1e-6))
        if rank < 1:
            self.get_logger().warn(f'_gen_search_poses: null-space rank={rank}')
            rank = min(3, n_arm)
        basis = Vt[:rank, :].T  # 6×rank

        q_safe_low = np.array(SAFE_Q_LOW[:n_arm])
        q_safe_high = np.array(SAFE_Q_HIGH[:n_arm])
        delta = self._search_delta
        n_dir = min(3, rank)

        poses = []

        # Home pose
        home_pose = _make_pose_stamped(p_ee, compute_ee_orientation(
            x_desired, HOME_ORI.tolist()))
        poses.append(home_pose)

        # ±δ along each basis direction (no diagonali)
        for i in range(n_dir):
            for sign in [-1.0, 1.0]:
                q_new = np.clip(q[:n_arm] + sign * delta * basis[:, i],
                                 q_safe_low, q_safe_high)
                pin.forwardKinematics(self._model, self._data, q_new)
                pin.updateFramePlacements(self._model, self._data)
                T_new = self._data.oMf[self._ee_id]
                pose = _make_pose_stamped(T_new.translation,
                                           compute_ee_orientation(
                                               x_desired, HOME_ORI.tolist()))
                poses.append(pose)

        self.get_logger().info(
            f'Search grid: {len(poses)} poses (null-space rank={rank}, delta={delta:.2f})',
            throttle_duration_sec=5.0)
        return poses

    def _tick_search(self) -> None:
        if self._scan_scanner is None:
            return

        for data in self._scan_data_queue:
            self._scan_scanner.feed_scan_data(data)
        self._scan_data_queue.clear()

        now = self.get_clock().now().nanoseconds * 1e-9
        st: ScanTick = self._scan_scanner.tick(ik_done=self._scan_ik_done, now=now)

        if st.action == ScanAction.SEND_IK and st.goal is not None:
            self._scan_ik_done = False
            self._pub_tracker_reset.publish(Bool(data=True))
            self._pub_ik.publish(st.goal)
            self._pub_en.publish(Bool(data=True))

        elif st.action in (ScanAction.EXIT_SCAN_MODE, ScanAction.DONE, ScanAction.FAILED):
            # Loop infinito: restart scanner con nuove pose (adattive alla q corrente)
            self._scan_scanner = None
            self._gen_and_start_search_scanner()

    def _pause_search(self) -> None:
        """Blocca il braccio durante SEMI_LOCKING."""
        self._search_paused = True
        self._pub_en.publish(Bool(data=False))

    def _resume_search(self) -> None:
        """Riprende dal punto in cui era stato messo in pausa."""
        self._search_paused = False

    def _end_search(self) -> None:
        """Esce da SEARCH_GRID mode."""
        self._mode = 'LOOKAT'
        self._search_paused = False
        self._pub_tracker_scan.publish(Bool(data=False))
        self._pub_en.publish(Bool(data=False))
        self._scan_scanner = None
        self._scan_data_queue.clear()

    def _send_home(self) -> None:
        """Pubblica la posa home all'IK solver."""
        home_pose = _make_pose_stamped(HOME_POS, HOME_ORI)
        self._pub_ik.publish(home_pose)
        self._pub_en.publish(Bool(data=True))
        self.get_logger().info('Home pose sent (LOCKING → home)')

    # ── SCAN_SEQ mode (APPROACHING) ───────────────────────────────────────

    def _start_scan(self) -> None:
        self._mode = 'SCAN_SEQ'
        self._scan_ik_done = False
        self._scan_data_queue.clear()

        poses = self._gen_scan_poses()
        if not poses:
            self.get_logger().warn('No WBC scan poses generated — DONE')
            self._publish_fast_points()
            self._mode = 'LOOKAT'
            return

        self._scan_poses = poses
        self._scan_scanner = BodySearchScanner(
            scan_poses=poses,
            scan_point_timeout=SCAN_POINT_TIMEOUT,
            scan_min_frames=SCAN_MIN_FRAMES,
            early_stop_score=SCAN_EARLY_STOP,
            logger=self.get_logger(),
            stability_k=SCAN_STABILITY_K,
        )
        self._scan_scanner.reset()

        # Enable tracker scan mode
        self._pub_tracker_scan.publish(Bool(data=True))

        self.get_logger().info(f'SCAN_SEQ: {len(poses)} WBC-generated poses')

    def _end_scan(self) -> None:
        self._mode = 'LOOKAT'
        self._pub_tracker_scan.publish(Bool(data=False))
        self._pub_en.publish(Bool(data=False))
        if self._scan_scanner is not None:
            torso = self._scan_scanner.fused_torso_xyz()
            if torso is not None:
                self._scan_torso_est = torso
            self._scan_scanner = None
        self._scan_poses.clear()

    def _tick_scan(self) -> None:
        if self._scan_scanner is None:
            return

        # Feed accumulated scan data
        for data in self._scan_data_queue:
            self._scan_scanner.feed_scan_data(data)
        self._scan_data_queue.clear()

        now = self.get_clock().now().nanoseconds * 1e-9
        st: ScanTick = self._scan_scanner.tick(ik_done=self._scan_ik_done, now=now)

        if st.action == ScanAction.SEND_IK and st.goal is not None:
            self._scan_ik_done = False
            self._pub_tracker_reset.publish(Bool(data=True))
            self._pub_ik.publish(st.goal)
            self._pub_en.publish(Bool(data=True))

        elif st.action in (ScanAction.EXIT_SCAN_MODE, ScanAction.DONE):
            self._pub_en.publish(Bool(data=False))
            self._pub_tracker_scan.publish(Bool(data=False))
            self._publish_fast_points()
            self._mode = 'LOOKAT'

        elif st.action == ScanAction.FAILED:
            self.get_logger().warn('Scan FAILED')
            self._pub_en.publish(Bool(data=False))
            self._pub_tracker_scan.publish(Bool(data=False))
            self._publish_fast_points()
            self._mode = 'LOOKAT'

    def _gen_scan_poses(self) -> list[PoseStamped]:
        """Generate scan poses via null-space of the look-at task."""
        if self._q_meas is None:
            self.get_logger().warn('_gen_scan_poses: no joint state')
            return []

        n_arm = self._q_meas.shape[0]
        q = self._q_neutral.copy()
        q[:n_arm] = self._q_meas

        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        T_ee = self._data.oMf[self._ee_id]
        p_ee = T_ee.translation
        J_arm_full = pin.getFrameJacobian(
            self._model, self._data, self._ee_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J_arm_full[:, :n_arm]

        # Look-at direction: from EE to goal target
        if self._goal is not None:
            target = np.array([self._goal.pose.position.x,
                               self._goal.pose.position.y,
                               self._goal.pose.position.z])
        else:
            target = p_ee + np.array([0.35, 0, 0])

        x_desired = target - p_ee
        x_desired = x_desired / max(float(np.linalg.norm(x_desired)), 1e-6)

        # Angular Jacobian (3×6) + null-space projector
        J_task = J_arm[:3, :]
        J_pinv = damped_pinv(J_task)
        N = null_space_projector(J_task, J_pinv)

        # SVD → basis of null-space
        _, S, Vt = np.linalg.svd(N)
        rank = int(np.sum(S > 1e-6))
        if rank < 1:
            self.get_logger().warn(f'_gen_scan_poses: null-space rank={rank} (arm singular?)')
            rank = min(3, n_arm)
        basis = Vt[:rank, :].T  # 6×rank

        q_low = self._model.lowerPositionLimit[:n_arm]
        q_high = self._model.upperPositionLimit[:n_arm]
        delta = self._scan_delta
        n_dir = min(3, rank)

        poses = []

        # Home pose
        home_pose = _make_pose_stamped(p_ee, compute_ee_orientation(
            x_desired, HOME_ORI.tolist()))
        poses.append(home_pose)

        # ±δ along each basis direction
        for i in range(n_dir):
            for sign in [-1.0, 1.0]:
                q_new = np.clip(q[:n_arm] + sign * delta * basis[:, i],
                                 q_low, q_high)
                pin.forwardKinematics(self._model, self._data, q_new)
                pin.updateFramePlacements(self._model, self._data)
                T_new = self._data.oMf[self._ee_id]
                pose = _make_pose_stamped(T_new.translation,
                                           compute_ee_orientation(
                                               x_desired, HOME_ORI.tolist()))
                poses.append(pose)

        # Diagonal combinations: v_i + v_j
        if n_dir >= 2:
            for i in range(min(n_dir - 1, 2)):
                j = i + 1
                for si, sj in [(1, 1), (1, -1)]:
                    q_new = np.clip(
                        q[:n_arm] + delta * (si * basis[:, i] + sj * basis[:, j]),
                        q_low, q_high)
                    pin.forwardKinematics(self._model, self._data, q_new)
                    pin.updateFramePlacements(self._model, self._data)
                    T_new = self._data.oMf[self._ee_id]
                    pose = _make_pose_stamped(T_new.translation,
                                               compute_ee_orientation(
                                                   x_desired, HOME_ORI.tolist()))
                    poses.append(pose)

        self.get_logger().info(
            f'Scan grid: {len(poses)} poses (null-space rank={rank}, delta={delta:.2f})')
        return poses

    # ── FAST point publishing ─────────────────────────────────────────────

    def _publish_fast_points(self) -> None:
        torso = self._scan_torso_est
        if torso is None:
            # Fallback: use approach goal position
            if self._goal is not None:
                torso = np.array([self._goal.pose.position.x,
                                  self._goal.pose.position.y,
                                  self._goal.pose.position.z])
            else:
                self.get_logger().warn('No torso estimate or goal — empty FAST')
                self._pub_fast.publish(PoseArray())
                self._pub_fast_ready.publish(Bool(data=True))
                return

        fast = PoseArray()
        fast.header.frame_id = self._z1_base_frame
        offsets = [
            (0.00, 0.00),          # Hub
            (0.00, -0.08),         # Subxiphoid
            (-0.05, -0.04),        # RUQ
            (0.05, -0.04),         # LUQ
            (0.00, 0.10),          # Suprapubic
        ]
        for dx, dy in offsets:
            pt = Pose()
            pt.position.x = float(torso[0] + dx)
            pt.position.y = float(torso[1] + dy)
            pt.position.z = float(torso[2])
            pt.orientation.w = 1.0
            fast.poses.append(pt)

        self._pub_fast.publish(fast)
        self._pub_fast_ready.publish(Bool(data=True))
        self.get_logger().info(
            f'FAST points published ({len(fast.poses)} pts, '
            f'torso=[{torso[0]:.2f},{torso[1]:.2f},{torso[2]:.2f}])')


def main(args=None):
    rclpy.init(args=args)
    node = WBCQPControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
