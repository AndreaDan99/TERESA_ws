#!/usr/bin/env python3
"""
WBC QP Controller — arm-only look-at + active-perception Cartesian scanning.

Modes (selected automatically from /wbc/state):
  ACTIVE_SEARCH  — SEARCHING:    6 pose ad arco attorno a HOME_POS (loop infinito)
  LOOKAT         — PRE_APPROACH: ω_des orientamento + null-space joint centering
  PERCEPTUAL_SCAN — APPROACHING:  6 Cartesian poses verso target, multi-angolo

State transitions:
  SEARCHING     → _start_active_search()
  SEMI_LOCKING  → _end_search() (braccio in LOOKAT, traccia torso mentre Spot ruota)
  LOCKING       → _end_search() + _send_home()
  SEARCHING (ripresa) → _start_active_search() (rigenera griglia)
"""

import math
import time

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import PointStamped, Pose, PoseStamped, PoseArray, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float32MultiArray

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

from teresa_utils.orientation import (
    compute_ee_orientation, compute_ee_orientation_minrot,
)

from spot_control.wbc_math import damped_pinv, null_space_projector, manipulability
from z1_vision.workspace_checker import WorkspaceChecker
from z1_vision.body_search_scanner import BodySearchScanner, ScanAction, ScanTick

JOINT_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# ── Scan parameters ────────────────────────────────────────────────────────────
SCAN_POINT_TIMEOUT = 4.0      # [s] unused (now from YAML: scan_timeout_per_point)
SCAN_MIN_FRAMES    = 5        # frame minimi per posa (sia search che scan)
SCAN_EARLY_STOP    = 0.95     # early-stop detection score
SCAN_STABILITY_K   = 10.0     # penalty stabilità 3D

# ── Search parameters ──────────────────────────────────────────────────────────
SEARCH_MIN_FRAMES  = 3        # frame minimi per search
SEARCH_EARLY_STOP  = 0.6      # early-stop liberale

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
        self.declare_parameter('cartesian_step',      0.12)
        self.declare_parameter('cartesian_step_wide', 0.20)
        self.declare_parameter('search_timeout_per_point', 2.0)
        self.declare_parameter('scan_timeout_per_point',   3.0)
        self.declare_parameter('scan_adaptive_iters',      3)
        self.declare_parameter('kp_confidence_ok',         0.4)
        self.declare_parameter('cartesian_x_advance',    0.10)
        self.declare_parameter('pre_scan_conf_thr',      0.6)
        self.declare_parameter('update_period', 0.1)
        self.declare_parameter('workspace_safety_margin', 0.05)
        self.declare_parameter('body_scan_reduced_ny', 2)
        self.declare_parameter('body_scan_reduced_nx', 2)
        self.declare_parameter('body_scan_reduced_wrist_ny', 2)
        self.declare_parameter('body_scan_reduced_wrist_nz', 2)
        self.declare_parameter('ik_goal_topic',      '/wbc/ik_goal_pose')
        self.declare_parameter('ik_enable_topic',    '/wbc/ik_enable')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('home_orientation', [-0.0062, 0.4107, 0.0021, 0.9118])
        self.declare_parameter('orientation_mode', 'minrot')
        self.declare_parameter('home_lock_z', 0.60)

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
        self._cartesian_step       = float(p('cartesian_step'))
        self._cartesian_step_wide  = float(p('cartesian_step_wide'))
        self._search_timeout_pp    = float(p('search_timeout_per_point'))
        self._scan_timeout_pp      = float(p('scan_timeout_per_point'))
        self._scan_adaptive_iters  = int(p('scan_adaptive_iters'))
        self._kp_confidence_ok     = float(p('kp_confidence_ok'))
        self._cartesian_x_advance  = float(p('cartesian_x_advance'))
        self._pre_scan_conf_thr    = float(p('pre_scan_conf_thr'))
        self._pre_scan_kp_conf     = [0.0, 0.0, 0.0, 0.0]
        self._update_period        = float(p('update_period'))
        self._home_orientation = np.array([float(x) for x in p('home_orientation')])
        self._orientation_mode = p('orientation_mode')
        self._home_lock_z      = float(p('home_lock_z'))

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
        self._mode           = 'LOOKAT'   # 'LOOKAT' | 'PERCEPTUAL_SCAN' | 'ACTIVE_SEARCH'
        self._wbc_state      = ''

        # ── Scan state ────────────────────────────────────────────────────
        self._scan_scanner: BodySearchScanner | None = None
        self._scan_ik_done       = False
        self._scan_data_queue: list[list[float]] = []
        self._scan_torso_est: np.ndarray | None = None
        self._scan_poses: list[PoseStamped] = []
        self._nlf_prior: list | None = None           # NLF prior from exposure scanner

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Bool,        '/wbc/enable',          self._cb_enable,      10)
        self.create_subscription(PoseStamped, '/wbc/ee_goal',         self._cb_goal,        10)
        self.create_subscription(JointState,  p('joint_states_topic'), self._cb_joints,      50)
        self.create_subscription(String,      '/wbc/state',           self._cb_wbc_state,   10)
        self.create_subscription(Bool,        '/ik_done',             self._cb_ik_done,     10)
        self.create_subscription(Float32MultiArray, '/torso_scan_point',
                                  self._cb_scan_data, 10)
        self.create_subscription(Float32MultiArray, '/torso_keypoint_conf',
                                  self._cb_kp_conf, 10)
        self.create_subscription(PoseArray, '/exposure/nlf_prior',
                                  self._cb_nlf_prior, 10)

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
        self._pub_grid_type     = self.create_publisher(String, '/wbc/scan_grid_type', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(self._update_period, self._update)
        self.get_logger().info('WBC QP Controller ready (arm-only).')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_enable(self, msg: Bool) -> None:
        self._enabled = msg.data
        if not self._enabled:
            en = Bool(); en.data = False
            self._pub_en.publish(en)
            if self._mode == 'PERCEPTUAL_SCAN':
                self._end_scan()
            elif self._mode == 'ACTIVE_SEARCH':
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
            if prev != 'SEARCHING':
                self._start_active_search()
        elif msg.data == 'SEMI_LOCKING' and self._mode == 'ACTIVE_SEARCH':
            self._end_search(re_enable=True)
        elif msg.data == 'LOCKING':
            self._end_search()
            self._send_home()
        elif msg.data == 'APPROACHING' and prev != 'APPROACHING':
            self._start_perceptual_scan()
        elif msg.data not in ('SEARCHING', 'SEMI_LOCKING', 'LOCKING', 'APPROACHING'):
            if self._mode == 'ACTIVE_SEARCH':
                self._end_search()
            if self._mode == 'PERCEPTUAL_SCAN':
                self._end_scan()

    def _cb_ik_done(self, msg: Bool) -> None:
        self._scan_ik_done = msg.data

    def _cb_scan_data(self, msg: Float32MultiArray) -> None:
        self._scan_data_queue.append(list(msg.data))

    def _cb_kp_conf(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            self._pre_scan_kp_conf = list(msg.data[:4])

    def _cb_nlf_prior(self, msg: PoseArray) -> None:
        """Store NLF body prior (24 SMPL joints in odom frame)."""
        joints_odom = []
        for pose in msg.poses:
            joints_odom.append(np.array([pose.position.x,
                                          pose.position.y,
                                          pose.position.z]))
        self._nlf_prior = joints_odom

    def _nlf_prior_valid(self) -> bool:
        """Check whether the NLF prior is usable for reduced scan."""
        if self._nlf_prior is None:
            return False
        if len(self._nlf_prior) != 24:
            return False
        from spot_perception.sml_pose_indices import SPINE1, SPINE2, SPINE3, PELVIS
        # At least one torso joint must be non-NaN
        for j in (SPINE1, SPINE2, SPINE3, PELVIS):
            if not np.any(np.isnan(self._nlf_prior[j])):
                return True
        return False

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
        pt = PointStamped()
        pt.header = pose.header
        pt.point.x = pose.pose.position.x
        pt.point.y = pose.pose.position.y
        pt.point.z = pose.pose.position.z
        try:
            tf_pt = self._tf.transform(
                pt, target_frame, timeout=Duration(seconds=timeout_sec))
            result = PoseStamped()
            result.header = tf_pt.header
            result.pose.position.x = tf_pt.point.x
            result.pose.position.y = tf_pt.point.y
            result.pose.position.z = tf_pt.point.z
            return result
        except TransformException:
            return None

    # ── Mode dispatch ─────────────────────────────────────────────────────

    def _update(self) -> None:
        if self._mode == 'PERCEPTUAL_SCAN':
            self._tick_perceptual_scan()
            return
        if self._mode == 'ACTIVE_SEARCH':
            self._tick_active_search()
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

    # ── ACTIVE_SEARCH mode (SEARCHING) ───────────────────────────────────

    def _gen_cartesian_search_grid(self) -> list[PoseStamped]:
        """
        7 hardcoded manual poses: position + quaternion.
        No tilt computation, no HOME_POS offset.
        """
        SEARCH_POSES = [
            ([0.144, -0.005,  0.530], [0.0182,  0.1521, -0.0217, 0.9880]),   # 1
            ([0.067, -0.070,  0.540], [0.0906,  0.1890, -0.3976, 0.8932]),   # 2
            ([0.057,  0.079,  0.538], [-0.0888, 0.1933,  0.4310, 0.8769]),   # 3
            ([-0.1001, -0.2671, 0.4404], [-0.1740, -0.1059, 0.9485, -0.2428]),  # 4 look-behind
            ([-0.2196, -0.1382, 0.5284], [-0.1314, 0.0163, 0.9893, 0.0608]),    # 5
            ([-0.1477,  0.1043, 0.5047], [-0.1151, 0.1665, 0.9711, 0.1263]),    # 6
            ([-0.0487,  0.0746, 0.3784], [-0.1336, 0.2679, 0.8804, 0.3678]),    # 7
        ]

        poses = []
        for pos, quat in SEARCH_POSES:
            pos_arr = np.array(pos)
            clipped, _, _ = self._ws_checker.clip_target(pos_arr)
            poses.append(_make_pose_stamped(clipped, quat))
        return poses

    def _start_active_search(self) -> None:
        if self._q_meas is None:
            self.get_logger().warn('_start_active_search: no joint state')
            return

        poses = self._gen_cartesian_search_grid()
        if not poses:
            self.get_logger().warn('_start_active_search: no poses generated')
            return

        self._mode = 'ACTIVE_SEARCH'
        self._scan_ik_done = False
        self._scan_data_queue.clear()
        self._scan_scanner = BodySearchScanner(
            scan_poses=poses,
            scan_point_timeout=self._search_timeout_pp,
            scan_min_frames=SEARCH_MIN_FRAMES,
            early_stop_score=SEARCH_EARLY_STOP,
            logger=self.get_logger(),
            stability_k=SCAN_STABILITY_K,
        )
        self._scan_scanner.reset()
        self.get_logger().info(
            f'ACTIVE_SEARCH: {len(poses)} wide poses '
            f'(HOME [0.50m], LEFT/RIGHT [±0.28m Y, +0.20m X, Z=0.42m], tilt -15°)')

    def _tick_active_search(self) -> None:
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
            self._scan_scanner = None
            self._start_active_search()

    def _end_search(self, re_enable: bool = False) -> None:
        self._mode = 'LOOKAT'
        self._pub_tracker_scan.publish(Bool(data=False))
        self._scan_scanner = None
        self._scan_data_queue.clear()
        self._pub_en.publish(Bool(data=re_enable))

    def _send_home(self) -> None:
        home_pos = np.array([0.144, -0.005, 0.530])
        home_quat = np.array([0.0182, 0.1521, -0.0217, 0.9880])
        home_pose = _make_pose_stamped(home_pos, home_quat)
        self._pub_ik.publish(home_pose)
        self._pub_en.publish(Bool(data=True))
        self.get_logger().info('Lock home sent: search pose 1')

    # ── PERCEPTUAL_SCAN mode (APPROACHING) ───────────────────────────────

    def _gen_cartesian_scan_grid(self, target: np.ndarray) -> list[PoseStamped]:
        """Generate 6-pose scan grid centered on torso estimate.
        NLF prior → tight offsets (4cm wrist, 6cm lateral)
        YOLO only → wide offsets (12cm wrist, 20cm lateral)
        """
        # Decide center and offsets
        if self._nlf_prior_valid():
            import numpy as np
            from spot_perception.sml_pose_indices import SPINE1, SPINE2, SPINE3, PELVIS
            torso_joints = [self._nlf_prior[j] for j in (SPINE1, SPINE2, SPINE3, PELVIS)
                            if not np.any(np.isnan(self._nlf_prior[j]))]
            center = np.mean(torso_joints, axis=0) if torso_joints else target
            wrist_step = 0.04
            lateral_step = 0.06
            grid_type = 'nlf'
        else:
            center = target
            wrist_step = 0.12
            lateral_step = 0.20
            grid_type = 'yolo'

        self._pub_grid_type.publish(String(data=grid_type))

        poses: list[PoseStamped] = []

        # Phase 1 — wrist sweep at center (2×2 = 4 poses)
        for wy in range(2):
            for wz in range(2):
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.pose.position.x = float(center[0])
                pose.pose.position.y = float(center[1]) + (wy - 0.5) * wrist_step
                pose.pose.position.z = float(center[2]) + (wz - 0.5) * wrist_step
                pose.pose.orientation.w = 1.0
                poses.append(pose)

        # Phase 2 — lateral parallax (±Y, 2 poses)
        for sign in [-1.0, 1.0]:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(center[0])
            pose.pose.position.y = float(center[1]) + sign * lateral_step
            pose.pose.position.z = float(center[2])
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        return poses

    def _start_perceptual_scan(self) -> None:
        if self._q_meas is None:
            self.get_logger().warn('_start_perceptual_scan: no joint state')
            self._publish_fast_points()
            return

        n_arm = self._q_meas.shape[0]
        q = self._q_neutral.copy()
        q[:n_arm] = self._q_meas
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        p_ee = self._data.oMf[self._ee_id].translation

        if self._goal is not None:
            target = np.array([self._goal.pose.position.x,
                               self._goal.pose.position.y,
                               self._goal.pose.position.z])
        else:
            target = p_ee + np.array([0.35, 0, 0])

        poses = self._gen_cartesian_scan_grid(target)
        if not poses:
            self.get_logger().warn('No Cartesian scan poses generated')
            self._publish_fast_points()
            return

        self._mode = 'PERCEPTUAL_SCAN'
        self._scan_ik_done = False
        self._scan_data_queue.clear()
        self._scan_poses = poses
        self._scan_scanner = BodySearchScanner(
            scan_poses=poses,
            scan_point_timeout=self._scan_timeout_pp,
            scan_min_frames=SCAN_MIN_FRAMES,
            early_stop_score=SCAN_EARLY_STOP,
            logger=self.get_logger(),
            stability_k=SCAN_STABILITY_K,
        )
        self._scan_scanner.reset()
        self._pub_tracker_scan.publish(Bool(data=True))
        self.get_logger().info(
            f'PERCEPTUAL_SCAN: {len(poses)} Cartesian poses')

    def _tick_perceptual_scan(self) -> None:
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

        elif st.action in (ScanAction.EXIT_SCAN_MODE, ScanAction.DONE):
            torso = self._scan_scanner.fused_torso_xyz()
            if torso is not None:
                self._scan_torso_est = torso
            self._pub_en.publish(Bool(data=False))
            self._pub_tracker_scan.publish(Bool(data=False))
            self._publish_fast_points()
            self._mode = 'LOOKAT'

        elif st.action == ScanAction.FAILED:
            self.get_logger().warn('Perceptual scan FAILED')
            self._pub_en.publish(Bool(data=False))
            self._pub_tracker_scan.publish(Bool(data=False))
            self._publish_fast_points()
            self._mode = 'LOOKAT'

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
