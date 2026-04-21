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

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

from spot_control.wbc_math import (
    compute_j_base,
    compute_j_holistic,
    manipulability,
    wbc_split,
    wbc_split_with_yaw,
)


JOINT_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class WBCQPControllerNode(Node):

    def __init__(self):
        super().__init__('wbc_qp_controller')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('odom_frame',    'my_spot/odom')
        self.declare_parameter('body_frame',    'my_spot/body')
        self.declare_parameter('z1_base_frame', 'link00')
        self.declare_parameter('ee_frame',      'link06')
        self.declare_parameter('lam_arm',       1.0)
        self.declare_parameter('lam_base',      1.0)
        self.declare_parameter('damping',       1e-3)
        self.declare_parameter('kp_pos',        1.0)
        self.declare_parameter('kp_ang',        0.5)
        self.declare_parameter('z_delta',        1.96)
        self.declare_parameter('k_yaw',         0.5)
        self.declare_parameter('vx_max',        0.4)
        self.declare_parameter('wz_max',        0.5)
        self.declare_parameter('q_dot_max',     0.6)
        self.declare_parameter('update_period', 1.5)
        self.declare_parameter('ik_goal_topic',      '/wbc/ik_goal_pose')
        self.declare_parameter('ik_enable_topic',    '/wbc/ik_enable')
        self.declare_parameter('cmd_vel_topic',      '/my_spot/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')

        p = lambda n: self.get_parameter(n).value
        self._odom_frame    = p('odom_frame')
        self._body_frame    = p('body_frame')
        self._ee_frame      = p('ee_frame')
        self._lam_arm       = float(p('lam_arm'))
        self._lam_base      = float(p('lam_base'))
        self._damping       = float(p('damping'))
        self._kp_pos        = float(p('kp_pos'))
        self._z_delta       = float(p('z_delta'))
        self._k_yaw         = float(p('k_yaw'))
        self._vx_max        = float(p('vx_max'))
        self._wz_max        = float(p('wz_max'))
        self._q_dot_max     = float(p('q_dot_max'))
        self._update_period = float(p('update_period'))

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

        # ── TF ────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── State ─────────────────────────────────────────────────────
        self._enabled        = False
        self._goal: PoseStamped | None = None
        self._q_meas: np.ndarray | None = None
        self._sigma_max      = 0.0    # sqrt(lambda_max(P_pos)) from coordinator Kalman
        self._desired_yaw: float | None = None  # target Spot yaw [rad, odom]

        # ── Sub / Pub ─────────────────────────────────────────────────
        self.create_subscription(Bool,        '/wbc/enable',               self._cb_enable,      10)
        self.create_subscription(PoseStamped, '/wbc/ee_goal',              self._cb_goal,        10)
        self.create_subscription(JointState,  p('joint_states_topic'),     self._cb_joints,      50)
        self.create_subscription(Float32,     '/wbc/target_uncertainty',   self._cb_uncert,      10)
        self.create_subscription(Float32,     '/wbc/desired_yaw',          self._cb_desired_yaw, 10)

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

    # ── Main update ───────────────────────────────────────────────────

    def _update(self) -> None:
        if not self._enabled or self._goal is None or self._q_meas is None:
            return

        # 1. EE pose in odom (for error computation)
        try:
            ee_in_odom = self._tf.lookup_transform(
                self._odom_frame, self._ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TF ee→odom: {e}', throttle_duration_sec=2.0)
            return

        # 2. EE position in body frame (for J_base)
        try:
            ee_in_body = self._tf.lookup_transform(
                self._body_frame, self._ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TF ee→body: {e}', throttle_duration_sec=2.0)
            return

        # 3. Rotation body→odom (to align J_base with J_arm world frame)
        try:
            body_in_odom = self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TF body→odom: {e}', throttle_duration_sec=2.0)
            return

        # 4. Transform goal to odom frame
        try:
            goal_odom = self._tf.transform(self._goal, self._odom_frame,
                                           timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TF goal→odom: {e}', throttle_duration_sec=2.0)
            return

        # 5. EE position error → desired spatial velocity (Pinocchio: [ang(3), lin(3)])
        dp = np.array([
            goal_odom.pose.position.x - ee_in_odom.transform.translation.x,
            goal_odom.pose.position.y - ee_in_odom.transform.translation.y,
            goal_odom.pose.position.z - ee_in_odom.transform.translation.z,
        ])
        # Chance-constraint dead zone: robot stops when EE is already inside the
        # uncertainty ball (radius = z_delta * sigma_max) around the Kalman estimate.
        # Prob(true target inside ball) >= 1 - delta  (delta = 0.05 for z_delta=1.96).
        dp_norm = float(np.linalg.norm(dp))
        r_ball  = self._z_delta * self._sigma_max
        effective = max(dp_norm - r_ball, 0.0)
        v_des = np.zeros(6)
        v_des[3:6] = self._kp_pos * (effective / (dp_norm + 1e-6)) * dp

        # 6. Pinocchio: J_arm in LOCAL_WORLD_ALIGNED (= odom-aligned)
        q = self._q_neutral.copy()
        q[:6] = self._q_meas
        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        J_arm = pin.getFrameJacobian(
            self._model, self._data, self._ee_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # 7. J_base in body frame → rotate to odom frame to match J_arm
        p_ee_body = np.array([
            ee_in_body.transform.translation.x,
            ee_in_body.transform.translation.y,
            ee_in_body.transform.translation.z,
        ])
        J_base_body = compute_j_base(p_ee_body)
        R_body_to_odom = _quat_to_rot(body_in_odom.transform.rotation)
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
            yaw_error = _normalize_angle(self._desired_yaw - θ_cur)
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

        # 9. Integrate q_dot → q_new → FK → new EE pose in Pinocchio world (= link00)
        q_new = q.copy()
        q_new[:6] = np.clip(
            self._q_meas + q_dot * self._update_period,
            self._model.lowerPositionLimit[:6],
            self._model.upperPositionLimit[:6],
        )
        pin.forwardKinematics(self._model, self._data, q_new)
        pin.updateFramePlacements(self._model, self._data)
        T_new = self._data.oMf[self._ee_id]

        # 10. Publish EE goal — T_new is already in link00 frame ('world' for z1_ik_to_jtc)
        goal_msg = PoseStamped()
        goal_msg.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'
        goal_msg.pose.position.x = float(T_new.translation[0])
        goal_msg.pose.position.y = float(T_new.translation[1])
        goal_msg.pose.position.z = float(T_new.translation[2])
        quat = _rot_to_quat(T_new.rotation)
        goal_msg.pose.orientation.x = float(quat[0])
        goal_msg.pose.orientation.y = float(quat[1])
        goal_msg.pose.orientation.z = float(quat[2])
        goal_msg.pose.orientation.w = float(quat[3])

        en = Bool(); en.data = True
        self._pub_en.publish(en)
        self._pub_ik.publish(goal_msg)

        # 11. Publish cmd_vel for Spot
        twist = Twist()
        twist.linear.x  = float(vx)
        twist.angular.z = float(wz)
        self._pub_vel.publish(twist)

        self.get_logger().info(
            f'WBC: m={m:.3f} vx={vx:.3f} wz={wz:.3f} '
            f'|dp|={dp_norm:.3f} r_ball={r_ball:.3f} eff={effective:.3f} '
            f'yaw_err={math.degrees(yaw_error):.1f}°',
            throttle_duration_sec=2.0)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _normalize_angle(a: float) -> float:
    return float((a + math.pi) % (2 * math.pi) - math.pi)


def _quat_to_rot(q) -> np.ndarray:
    """geometry_msgs Quaternion → 3x3 rotation matrix."""
    from tf_transformations import quaternion_matrix
    R = quaternion_matrix([q.x, q.y, q.z, q.w])
    return R[:3, :3]


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix → [x, y, z, w] quaternion."""
    from tf_transformations import quaternion_from_matrix
    M = np.eye(4)
    M[:3, :3] = R
    return quaternion_from_matrix(M)


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
