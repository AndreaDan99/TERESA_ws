#!/usr/bin/env python3
"""
WBC QP Controller
Reads EE goal from /wbc/ee_goal, arm joints from /joint_states and TF,
solves holistic WBC split, publishes:
  /ik_goal_pose  → z1_ik_to_jtc  (arm share, PoseStamped in 'world'/link00)
  /my_spot/cmd_vel → Spot         (base share, Twist)

Update rate: update_period (default 1.5s — respects z1_ik_to_jtc traj_min_time=1.0s).
Enabled/disabled via /wbc/enable (Bool).
"""
import math

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

from teresa_utils.orientation import (
    compute_ee_orientation, compute_ee_orientation_minrot,
    quat_to_rot, rot_to_quat, normalize_angle,
)

from spot_control.wbc_math import (
    compute_j_base,
    compute_j_holistic,
    manipulability,
    wbc_split,
    wbc_split_with_yaw,
)
from z1_vision.workspace_checker import WorkspaceChecker


JOINT_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class WBCQPControllerNode(Node):

    def __init__(self):
        super().__init__('wbc_qp_controller')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('dry_run',        False)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('odom_frame',    'my_spot/odom')
        self.declare_parameter('body_frame',    'my_spot/body')
        self.declare_parameter('z1_base_frame', 'world')
        self.declare_parameter('ee_frame',      'link06')
        self.declare_parameter('lam_arm',       1.0)
        self.declare_parameter('lam_base',      1.0)
        self.declare_parameter('damping',       1e-3)
        self.declare_parameter('kp_pos',        1.0)
        self.declare_parameter('kp_ang',        0.5)
        self.declare_parameter('quality_ref',   0.05)
        self.declare_parameter('v_min',         0.15)
        self.declare_parameter('k_yaw',         0.5)
        self.declare_parameter('vx_max',        0.4)
        self.declare_parameter('wz_max',        0.5)
        self.declare_parameter('q_dot_max',     0.6)
        self.declare_parameter('update_period', 1.5)
        self.declare_parameter('workspace_safety_margin', 0.05)
        self.declare_parameter('z1_mount_x',      0.20)
        self.declare_parameter('z1_mount_y',      0.0)
        self.declare_parameter('z1_mount_z',      0.20)
        self.declare_parameter('ik_goal_topic',      '/wbc/ik_goal_pose')
        self.declare_parameter('ik_enable_topic',    '/wbc/ik_enable')
        self.declare_parameter('cmd_vel_topic',      '/my_spot/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('home_orientation', [-0.0062, 0.4107, 0.0021, 0.9118])
        self.declare_parameter('orientation_mode', 'minrot')  # 'minrot' | 'gram_schmidt'

        p = lambda n: self.get_parameter(n).value
        self._dry_run       = bool(p('dry_run'))
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._z1_base_frame = p('z1_base_frame')
        self._ee_frame      = p('ee_frame')
        self._lam_arm       = float(p('lam_arm'))
        self._lam_base      = float(p('lam_base'))
        self._damping       = float(p('damping'))
        self._kp_pos        = float(p('kp_pos'))
        self._quality_ref   = float(p('quality_ref'))
        self._v_min         = float(p('v_min'))
        self._k_yaw         = float(p('k_yaw'))
        self._vx_max        = float(p('vx_max'))
        self._wz_max        = float(p('wz_max'))
        self._q_dot_max     = float(p('q_dot_max'))
        self._update_period = float(p('update_period'))
        self._home_orientation = np.array([float(x) for x in p('home_orientation')])
        self._orientation_mode = p('orientation_mode')

        # ── Pinocchio ─────────────────────────────────────────────────
        urdf = p('urdf_path')
        if not urdf:
            import os
            try:
                from ament_index_python.packages import get_package_share_directory
                urdf = os.path.join(get_package_share_directory('z1_description'), 'urdf', 'z1.urdf')
            except Exception:
                urdf = os.path.expanduser(
                    '~/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf'
                )
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

        # ── TF ────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._mount_x = float(p('z1_mount_x'))
        self._mount_y = float(p('z1_mount_y'))
        self._mount_z = float(p('z1_mount_z'))

        # ── State ─────────────────────────────────────────────────────
        self._enabled        = False
        self._goal: PoseStamped | None = None
        self._q_meas: np.ndarray | None = None
        self._sigma_max      = 0.0    # quality [m] from QualityMonitor (not std dev)
        self._desired_yaw: float | None = None  # target Spot yaw [rad, odom]
        self._spot_control = True  # cmd_vel enabled by default
        self._tf_ready   = False  # TF available flag

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(Bool,        '/wbc/enable',               self._cb_enable,      10)
        self.create_subscription(PoseStamped, '/wbc/ee_goal',              self._cb_goal,        10)
        self.create_subscription(JointState,  p('joint_states_topic'),     self._cb_joints,      50)
        self.create_subscription(Float32,     '/wbc/target_uncertainty',   self._cb_uncert,      10)
        self.create_subscription(Float32,     '/wbc/desired_yaw',          self._cb_desired_yaw, 10)
        self.create_subscription(Bool,        '/wbc/spot_control',         self._cb_spot_control, 10)

        if self._dry_run:
            self._pub_ik  = self.create_publisher(PoseStamped, '/wbc/ik_goal_pose_debug',  10)
            self._pub_en  = self.create_publisher(Bool,        '/wbc/ik_enable_debug',     10)
            self._pub_vel = self.create_publisher(Twist,       '/wbc/cmd_vel_debug',       10)
            self.get_logger().warn(
                'DRY_RUN mode — output on /wbc/*_debug topics, no robot movement')
        else:
            self._pub_ik  = self.create_publisher(PoseStamped, p('ik_goal_topic'),   10)
            self._pub_en  = self.create_publisher(Bool,        p('ik_enable_topic'), 10)
            self._pub_vel = self.create_publisher(Twist,       p('cmd_vel_topic'),   10)

        # ── Timer ─────────────────────────────────────────────────────
        self.create_timer(self._update_period, self._update)
        self.get_logger().info('WBC QP Controller ready.')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_enable(self, msg: Bool) -> None:
        self._enabled = msg.data
        if not self._enabled:
            self._pub_vel.publish(Twist())
            en = Bool(); en.data = False
            self._pub_en.publish(en)

    def _cb_goal(self, msg: PoseStamped) -> None:
        self._goal = msg

    def _cb_joints(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._q_meas = np.array([name_to_pos[j] for j in JOINT_ORDER])
        except KeyError:
            pass

    def _cb_uncert(self, msg: Float32) -> None:
        self._sigma_max = float(msg.data)

    def _cb_desired_yaw(self, msg: Float32) -> None:
        self._desired_yaw = float(msg.data)

    def _cb_spot_control(self, msg: Bool) -> None:
        self._spot_control = msg.data

    # ── TF helpers ────────────────────────────────────────────────────

    def _tf_lookup(self, source: str, target: str,
                   timeout_sec: float = 1.0) -> TransformStamped | None:
        try:
            return self._tf.lookup_transform(
                source, target,
                rclpy.time.Time(), timeout=Duration(seconds=timeout_sec))
        except TransformException as e:
            if not self._tf_ready:
                self.get_logger().warn(
                    f'TF {source} → {target} non disponibile.\n'
                    f'  Diagnostica: ros2 topic list | grep tf\n'
                    f'  Verifica: 1) spot_ros2 attivo su SpotCore?  '
                    f'2) ROS_DOMAIN_ID uguale?  '
                    f'3) spot_name=\'my_spot\'?',
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
        except TransformException as e:
            if not self._tf_ready:
                self.get_logger().warn(
                    f'TF {pose.header.frame_id} → {target_frame} non disponibile.\n'
                    f'  Diagnostica: ros2 topic list | grep tf',
                    throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(
                    f'TF {pose.header.frame_id} → {target_frame} persa ({e})',
                    throttle_duration_sec=2.0)
            return None

    # ── Main update ───────────────────────────────────────────────────

    def _update(self) -> None:
        if not self._enabled or self._goal is None or self._q_meas is None:
            return

        # 1. EE pose in body frame (PC-only, always reliable)
        ee_in_body = self._tf_lookup(self._body_frame, self._ee_frame)
        if ee_in_body is None:
            return

        # 2. body position in odom (single hop, more reliable than full chain)
        body_in_odom = self._tf_lookup(self._odom_frame, self._body_frame)
        if body_in_odom is None:
            return

        # 3. Compose ee_in_odom from body_in_odom * ee_in_body
        #    (avoids cross-machine TF chain that fails without clock sync)
        _q = body_in_odom.transform.rotation
        _qv = np.array([_q.x, _q.y, _q.z])
        _qw = float(_q.w)
        _p_eeb = np.array([
            ee_in_body.transform.translation.x,
            ee_in_body.transform.translation.y,
            ee_in_body.transform.translation.z])
        _p_eeb_rot = _p_eeb + 2.0 * np.cross(_qv, np.cross(_qv, _p_eeb) + _qw * _p_eeb)
        _p_eeodom = np.array([
            body_in_odom.transform.translation.x,
            body_in_odom.transform.translation.y,
            body_in_odom.transform.translation.z]) + _p_eeb_rot

        ee_in_odom = TransformStamped()
        ee_in_odom.header.frame_id = self._odom_frame
        ee_in_odom.child_frame_id = self._ee_frame
        ee_in_odom.transform.translation.x = float(_p_eeodom[0])
        ee_in_odom.transform.translation.y = float(_p_eeodom[1])
        ee_in_odom.transform.translation.z = float(_p_eeodom[2])

        # First successful TF lookup in this session → confirm connectivity
        if not self._tf_ready:
            self._tf_ready = True
            self.get_logger().info(
                f'TF disponibile: {self._odom_frame} → {self._body_frame} OK. '
                f'SpotCore connesso via DDS.')

        # 4. Resolve goal to both odom (for position error) and link00 (for look-at).
        #    Coordinator publishes in odom frame.
        #    Z1 FSM publishes in world/link00 frame (WS_EXTENSION path).
        goal_in = self._goal
        goal_frame = goal_in.header.frame_id

        # Goal position in odom frame (for dp comparison with EE in odom)
        if goal_frame in ('world', 'link00', self._z1_base_frame):
            goal_stamped = PoseStamped()
            goal_stamped.header.frame_id = self._z1_base_frame
            goal_stamped.header.stamp    = rclpy.time.Time().to_msg()
            goal_stamped.pose            = goal_in.pose
            goal_odom = self._tf_transform(goal_stamped, self._odom_frame)
            if goal_odom is None:
                return
            goal_link00 = goal_stamped
        else:
            goal_odom = goal_in
            goal_stamped = PoseStamped()
            goal_stamped.header.frame_id = goal_frame
            goal_stamped.header.stamp    = rclpy.time.Time().to_msg()
            goal_stamped.pose            = goal_in.pose
            goal_link00 = self._tf_transform(goal_stamped, self._z1_base_frame)
            if goal_link00 is None:
                return

        # 5. EE position error → desired spatial velocity (Pinocchio: [ang(3), lin(3)])
        dp = np.array([
            goal_odom.pose.position.x - ee_in_odom.transform.translation.x,
            goal_odom.pose.position.y - ee_in_odom.transform.translation.y,
            goal_odom.pose.position.z - ee_in_odom.transform.translation.z,
        ])
        dp_norm = float(np.linalg.norm(dp))
        v_des = np.zeros(6)
        v_des[3:6] = self._kp_pos * dp

        # 6. Pinocchio: J_arm in LOCAL_WORLD_ALIGNED (= odom-aligned)
        n_arm = self._q_meas.shape[0]
        q = self._q_neutral.copy()
        q[:n_arm] = self._q_meas
        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        J_arm_full = pin.getFrameJacobian(
            self._model, self._data, self._ee_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_arm = J_arm_full[:, :n_arm]

        # 7. J_base in body frame → rotate to odom frame to match J_arm
        p_ee_body = np.array([
            ee_in_body.transform.translation.x,
            ee_in_body.transform.translation.y,
            ee_in_body.transform.translation.z,
        ])
        J_base_body = compute_j_base(p_ee_body)
        R_body_to_odom = quat_to_rot(body_in_odom.transform.rotation)
        J_base_odom = np.zeros((6, 2))
        J_base_odom[:3, :] = R_body_to_odom @ J_base_body[:3, :]
        J_base_odom[3:, :] = R_body_to_odom @ J_base_body[3:, :]

        J_hol = compute_j_holistic(J_arm, J_base_odom)

        # 8. Manipulability + WBC split (with yaw task if target yaw is known)
        m = manipulability(J_arm)
        if self._desired_yaw is not None:
            from tf_transformations import euler_from_quaternion
            _, _, θ_cur = euler_from_quaternion([
                body_in_odom.transform.rotation.x,
                body_in_odom.transform.rotation.y,
                body_in_odom.transform.rotation.z,
                body_in_odom.transform.rotation.w,
            ])
            yaw_error = normalize_angle(self._desired_yaw - θ_cur)
            q_dot, vx, wz = wbc_split_with_yaw(
                J_hol, v_des, m,
                yaw_error=yaw_error,
                k_yaw=self._k_yaw,
                lam_arm=self._lam_arm, lam_base=self._lam_base,
                damping=self._damping,
                vx_max=self._vx_max, wz_max=self._wz_max,
                q_dot_max=self._q_dot_max,
            )
        else:
            yaw_error = 0.0
            q_dot, vx, wz = wbc_split(
                J_hol, v_des, m,
                lam_arm=self._lam_arm, lam_base=self._lam_base,
                damping=self._damping,
                vx_max=self._vx_max, wz_max=self._wz_max,
                q_dot_max=self._q_dot_max,
            )

        # 8.5. Quality-based velocity scaling — never zero, Spot always moves.
        # quality [m] = EMA(|new_meas - fixed_target|) + growth when Orbbec lost.
        # v_min = minimum velocity fraction (never stops before handoff).
        quality = self._sigma_max   # now quality [m], not standard deviation
        k = 1.0 / self._quality_ref
        v_scale = self._v_min + (1.0 - self._v_min) / (1.0 + k * quality)
        vx *= v_scale
        wz *= v_scale

        # 9. Integrate q_dot → q_new → FK → new EE pose in Pinocchio world (= link00)
        q_new = q.copy()
        q_new[:n_arm] = np.clip(
            self._q_meas + q_dot * self._update_period,
            self._model.lowerPositionLimit[:n_arm],
            self._model.upperPositionLimit[:n_arm],
        )
        pin.forwardKinematics(self._model, self._data, q_new)
        pin.updateFramePlacements(self._model, self._data)
        T_new = self._data.oMf[self._ee_id]

        # 9.5. Clip EE goal to safe workspace (max_reach - safety_margin)
        ws_pos = np.array([T_new.translation[0],
                           T_new.translation[1],
                           T_new.translation[2]])
        clipped_pos, was_clipped, _ = self._ws_checker.clip_target(ws_pos)
        if was_clipped:
            self.get_logger().warn(
                f'WBC goal clipped to workspace (safety_margin={self._ws_checker.safety_margin:.2f}m): '
                f'raw=[{ws_pos[0]:.3f},{ws_pos[1]:.3f},{ws_pos[2]:.3f}] → '
                f'clipped=[{clipped_pos[0]:.3f},{clipped_pos[1]:.3f},{clipped_pos[2]:.3f}]',
                throttle_duration_sec=3.0)

        # 10. Publish EE goal — position from FK, orientation from EE to target.
        # X_ee = direction from predicted EE (clipped_pos) to target (in link00 frame).
        # Consistent: position and orientation use same time horizon (q_new).
        goal_msg = PoseStamped()
        goal_msg.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'
        goal_msg.pose.position.x = float(clipped_pos[0])
        goal_msg.pose.position.y = float(clipped_pos[1])
        goal_msg.pose.position.z = float(clipped_pos[2])

        target_link00 = np.array([goal_link00.pose.position.x,
                                   goal_link00.pose.position.y,
                                   goal_link00.pose.position.z])
        x_ee = target_link00 - clipped_pos
        x_norm = float(np.linalg.norm(x_ee))
        if x_norm < 1e-6:
            x_ee = np.array([1.0, 0.0, 0.0])
        else:
            x_ee = x_ee / x_norm

        quat = (compute_ee_orientation_minrot(x_ee, self._home_orientation.tolist())
                if self._orientation_mode == 'minrot'
                else compute_ee_orientation(x_ee, self._home_orientation.tolist()))
        goal_msg.pose.orientation.x = float(quat[0])
        goal_msg.pose.orientation.y = float(quat[1])
        goal_msg.pose.orientation.z = float(quat[2])
        goal_msg.pose.orientation.w = float(quat[3])

        en = Bool(); en.data = True
        self._pub_en.publish(en)
        self._pub_ik.publish(goal_msg)

        # 11. Publish cmd_vel for Spot (suppressed when spot_control=False)
        if self._spot_control:
            twist = Twist()
            twist.linear.x  = float(vx)
            twist.angular.z = float(wz)
            self._pub_vel.publish(twist)

        prefix = '[DRY_RUN] ' if self._dry_run else ''
        self.get_logger().info(
            f'{prefix}WBC: m={m:.3f} vx={vx:.3f} wz={wz:.3f} '
            f'|dp|={dp_norm:.3f} q={quality:.3f} v_scale={v_scale:.2f} '
            f'yaw_err={math.degrees(yaw_error):.1f}°',
            throttle_duration_sec=2.0)


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
