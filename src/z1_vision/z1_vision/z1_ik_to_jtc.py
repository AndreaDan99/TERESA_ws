#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _quintic_blend(s: float) -> float:
    # 6s^5 - 15s^4 + 10s^3
    return (6*s**5) - (15*s**4) + (10*s**3)


class Z1IKToJTC(Node):
    """
    Subscribe:
      - /ik_goal_pose (PoseStamped): desired EE pose (world/base frame)
      - /joint_states (JointState): current joints
    Action:
      - /joint_trajectory_controller/follow_joint_trajectory
    Publish:
      - /ik_jtc/success (Bool)
      - /ik_jtc/status  (String)
    """

    def __init__(self):
        super().__init__('z1_ik_to_jtc')

        # ---------------- Parameters ----------------
        self.declare_parameter('robot_description_param', 'robot_description')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('ee_frame', 'link06')          # EE frame in URDF
        self.declare_parameter('controller_action', '/joint_trajectory_controller/follow_joint_trajectory')

        # joints used by controller (order must match controller joint_names)
        self.declare_parameter('joint_names', [
            'joint1','joint2','joint3','joint4','joint5','joint6'
        ])

        # trajectory
        self.declare_parameter('traj_duration', 3.0)           # seconds
        self.declare_parameter('traj_points', 40)              # discrete points

        # IK solver
        self.declare_parameter('ik_max_iter', 80)
        self.declare_parameter('ik_tol', 1e-4)
        self.declare_parameter('ik_damping', 1e-3)
        self.declare_parameter('ik_step', 0.8)                 # step scale
        self.declare_parameter('ik_joint_limit_margin', 1e-3)

        # orientation weight (0..1): if 0 -> position-only IK
        self.declare_parameter('use_orientation', True)

        # ---------------- Read parameters ----------------
        self.robot_description_param = self.get_parameter('robot_description_param').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.controller_action = self.get_parameter('controller_action').value
        self.joint_names = list(self.get_parameter('joint_names').value)

        self.traj_duration = float(self.get_parameter('traj_duration').value)
        self.traj_points = int(self.get_parameter('traj_points').value)

        self.ik_max_iter = int(self.get_parameter('ik_max_iter').value)
        self.ik_tol = float(self.get_parameter('ik_tol').value)
        self.ik_damping = float(self.get_parameter('ik_damping').value)
        self.ik_step = float(self.get_parameter('ik_step').value)
        self.ik_joint_limit_margin = float(self.get_parameter('ik_joint_limit_margin').value)

        self.use_orientation = bool(self.get_parameter('use_orientation').value)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- State ----------------
        self.last_joint_state = None
        self.robot = None
        self.model = None
        self.data = None
        self.ee_id = None

        # Action goal tracking
        self.goal_in_flight = False

        self.q_min = None
        self.q_max = None

        # ---------------- ROS interfaces ----------------
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/ik_goal_pose', self.cb_goal_pose, 10)

        self.pub_success = self.create_publisher(Bool, '/ik_jtc/success', 10)
        self.pub_status = self.create_publisher(String, '/ik_jtc/status', 10)
        self.pub_done = self.create_publisher(Bool, '/ik_jtc/done', 10)

        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller_action)

        # ---------------- Init robot model from URDF ----------------
        self._init_robot_from_urdf_param()

        self.get_logger().info(
            "🦾 z1_ik_to_jtc pronto\n"
            f"  base_frame: {self.base_frame}\n"
            f"  ee_frame:   {self.ee_frame}\n"
            f"  joints:     {self.joint_names}\n"
            f"  action:     {self.controller_action}\n"
            f"  IK: it={self.ik_max_iter} tol={self.ik_tol} damp={self.ik_damping} step={self.ik_step}"
        )

    # ---------------- Utilities ----------------
    def _status(self, text: str):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    def _success(self, ok: bool):
        m = Bool()
        m.data = bool(ok)
        self.pub_success.publish(m)

    def _done(self, done: bool):
        m = Bool()
        m.data = bool(done)
        self.pub_done.publish(m)

    def _init_robot_from_urdf_param(self):
        # Robot description must be available in this node namespace or global.
        # Commonly published by robot_state_publisher.
        urdf = None
        try:
            urdf = self.get_parameter(self.robot_description_param).value
        except Exception:
            urdf = None

        if not urdf:
            # try to declare and read (sometimes param not declared yet)
            try:
                self.declare_parameter(self.robot_description_param, '')
                urdf = self.get_parameter(self.robot_description_param).value
            except Exception:
                urdf = None

        if not urdf:
            self.get_logger().error(
                f"❌ robot_description non trovato come parametro '{self.robot_description_param}'. "
                "Assicurati che robot_state_publisher lo pubblichi e che questo nodo lo veda."
            )
            self._status("robot_description_missing")
            return

        try:
            # `urdf` is an XML string (robot_description). Build model from XML.
            self.model = pin.buildModelFromXML(urdf)
            self.data = self.model.createData()
            self.robot = None
            self.ee_id = self.model.getFrameId(self.ee_frame)

            # joint limits
            self.q_min = self.model.lowerPositionLimit.copy()
            self.q_max = self.model.upperPositionLimit.copy()

            self._status("robot_model_loaded")
        except Exception as e:
            self.get_logger().error(f"❌ Errore caricando URDF XML in Pinocchio: {e}")
            self._status("robot_model_error")

    def cb_joint_states(self, msg: JointState):
        self.last_joint_state = msg

    def _get_current_q(self):
        if self.last_joint_state is None:
            return None

        name_to_idx = {n: i for i, n in enumerate(self.last_joint_state.name)}
        q = []

        for jn in self.joint_names:
            if jn not in name_to_idx:
                return None
            q.append(self.last_joint_state.position[name_to_idx[jn]])

        q = np.array(q, dtype=np.float64)
        return q

    def _pose_to_SE3(self, pose_msg: PoseStamped):
        # PoseStamped -> pin.SE3
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        # quaternion to rotation matrix
        # pin uses (w,x,y,z)
        R = pin.Quaternion(q.w, q.x, q.y, q.z).toRotationMatrix()
        t = np.array([p.x, p.y, p.z], dtype=np.float64)
        return pin.SE3(R, t)

    def _transform_pose_to_base(self, pose_in: PoseStamped) -> PoseStamped:
        # ensure pose in base_frame
        if pose_in.header.frame_id == self.base_frame or pose_in.header.frame_id == '':
            return pose_in
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose_in.header.frame_id,
                rclpy.time.Time()
            )
            pose_out = do_transform_pose(pose_in, tf)
            pose_out.header.frame_id = self.base_frame
            return pose_out
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF non disponibile {pose_in.header.frame_id}->{self.base_frame}: {e}")
            return None

    def _ik_solve(self, target_SE3: pin.SE3, q0: np.ndarray):
        """
        Damped least squares IK on end-effector frame.
        Returns: q_sol, success(bool), final_err
        """
        if self.robot is None or self.model is None or self.data is None or self.ee_id is None:
            return None, False, None

        # Pinocchio expects q size = model.nq; but our q0 is only for controlled joints.
        # We assume the robot has exactly these joints in order in model (typical).
        # If your model has extra joints, serve mapping serio. Per ora: best-effort.
        if self.model.nq != len(q0):
            # attempt if model has same count of actuated joints as ours
            # otherwise fail loudly
            self.get_logger().error(
                f"❌ model.nq={self.model.nq} ma joint_names={len(q0)}. "
                "Serve mapping tra joint state e Pinocchio model."
            )
            return None, False, None

        q = q0.copy()
        damp = self.ik_damping
        step = self.ik_step

        # limits
        lo = self.q_min + self.ik_joint_limit_margin
        hi = self.q_max - self.ik_joint_limit_margin

        for it in range(self.ik_max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]
            # error in SE3 (log map)
            dMi = current.actInv(target_SE3)  # current^{-1} * target
            err6 = pin.log6(dMi).vector       # 6D error: [v, w]

            if not self.use_orientation:
                err6[3:] = 0.0  # ignore orientation

            err_norm = float(np.linalg.norm(err6))
            if err_norm < self.ik_tol:
                return q, True, err_norm

            # frame jacobian (LOCAL frame)
            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.ee_id, pin.ReferenceFrame.LOCAL)

            if not self.use_orientation:
                J6[3:, :] = 0.0

            # damped least squares: dq = - J^T (J J^T + λ^2 I)^{-1} err
            JJt = J6 @ J6.T
            A = JJt + (damp**2) * np.eye(6)
            try:
                y = np.linalg.solve(A, err6)
            except np.linalg.LinAlgError:
                y = np.linalg.lstsq(A, err6, rcond=None)[0]
            dq = - J6.T @ y

            q = q + step * dq

            # clamp
            q = np.array([_clamp(q[i], lo[i], hi[i]) for i in range(len(q))], dtype=np.float64)

        # not converged
        return q, False, float(np.linalg.norm(err6))

    def _build_trajectory(self, q_start: np.ndarray, q_goal: np.ndarray) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        T = max(0.5, self.traj_duration)
        N = max(2, self.traj_points)

        for k in range(N):
            s = k / (N - 1)
            b = _quintic_blend(s)
            qk = q_start + b * (q_goal - q_start)

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in qk.tolist()]
            pt.time_from_start.sec = int(math.floor(s * T))
            pt.time_from_start.nanosec = int((s * T - pt.time_from_start.sec) * 1e9)
            traj.points.append(pt)

        return traj

    # ---------------- Main callback ----------------
    def cb_goal_pose(self, goal_pose: PoseStamped):
        if self.goal_in_flight:
            self.get_logger().warn('⚠️ Nuovo goal ricevuto ma un goal JTC è già in corso: ignorato')
            self._status('goal_ignored_in_flight')
            self._success(False)
            return

        self._done(False)
        self._success(False)

        # basic readiness
        if self.robot is None:
            self._status("robot_model_not_ready")
            self._done(True)
            self._success(False)
            return

        q0 = self._get_current_q()
        if q0 is None:
            self.get_logger().warn("⚠️ joint_states non contiene tutti i joint_names richiesti.")
            self._status("joint_state_missing")
            self._done(True)
            self._success(False)
            return

        pose_base = self._transform_pose_to_base(goal_pose)
        if pose_base is None:
            self._status("tf_missing")
            self._done(True)
            self._success(False)
            return

        target = self._pose_to_SE3(pose_base)

        q_sol, ok, err = self._ik_solve(target, q0)
        if not ok:
            self.get_logger().warn(f"⚠️ IK non convergente. err={err}")
            self._status("ik_failed")
            self._done(True)
            self._success(False)
            return

        # build and send trajectory
        traj = self._build_trajectory(q0, q_sol)

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Action server JTC non disponibile.")
            self._status("jtc_action_missing")
            self._done(True)
            self._success(False)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self._status("sending_goal")
        self.goal_in_flight = True
        send_future = self.action_client.send_goal_async(goal)

        def _goal_response_cb(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("❌ Goal JTC rifiutato dal server.")
                self.goal_in_flight = False
                self._success(False)
                self._done(True)
                return

            def _result_cb(rf):
                res = rf.result()
                ok = (res is not None and res.status == 4)  # 4 = SUCCEEDED
                self._status('goal_succeeded' if ok else f'goal_failed_status_{res.status if res else "none"}')
                self._success(ok)
                self._done(True)
                self.goal_in_flight = False

            goal_handle.get_result_async().add_done_callback(_result_cb)

        send_future.add_done_callback(_goal_response_cb)


def main():
    rclpy.init()
    node = Z1IKToJTC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()