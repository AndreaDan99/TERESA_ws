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
        self.declare_parameter('urdf_path', '/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf')
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
        urdf_path = self.get_parameter('urdf_path').value
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
        self.model = None
        self.data = None
        self.ee_id = None

        # Action goal tracking
        self.goal_in_flight = False
        self.current_goal_handle = None
        self.goal_sent_time = None

        # Safety: if the controller never returns a result, unlock after timeout
        self.declare_parameter('goal_timeout_sec', 8.0)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self._goal_watchdog_timer = self.create_timer(0.2, self._goal_watchdog)

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
        
    def _unlock_goal(self, reason: str):
        # helper to consistently unlock goal state
        self.get_logger().warn(f"🔓 Unlock goal_in_flight ({reason})")
        self.goal_in_flight = False
        self.current_goal_handle = None
        self.goal_sent_time = None

    def _goal_watchdog(self):
        # If a goal is in flight but no result arrives, unlock after timeout
        if not self.goal_in_flight or self.goal_sent_time is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.goal_sent_time).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec:
            self._status('goal_timeout')
            self._success(False)
            self._done(True)
            # Try to cancel, but unlock regardless
            try:
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"⚠️ cancel_goal_async failed: {e}")
            self._unlock_goal('timeout')

    def _init_robot_from_urdf_param(self):
        urdf_path = self.get_parameter('urdf_path').value

        if not urdf_path:
            self.get_logger().error("❌ Parametro 'urdf_path' non impostato o vuoto.")
            self._status("urdf_path_missing")
            return

        try:
            self.model = pin.buildModelFromUrdf(urdf_path) 
            self.nq_ctrl = len(self.joint_names)                     
            self._to_full   = lambda qc: np.append(qc, 0.0)          
            self._from_full = lambda qf: qf[:self.nq_ctrl]   
            self.data = self.model.createData()
            self.ee_id = self.model.getFrameId(self.ee_frame)

            if self.ee_id >= self.model.nframes:
                self.get_logger().error(
                    f"❌ Frame '{self.ee_frame}' non trovato. "
                    f"Frames disponibili: {[self.model.frames[i].name for i in range(self.model.nframes)]}"
                )
                self._status("ee_frame_not_found")
                return

            self.q_min = self.model.lowerPositionLimit.copy()
            self.q_max = self.model.upperPositionLimit.copy()

            self.get_logger().info(
                f"✅ Modello URDF caricato da: {urdf_path} | nq={self.model.nq} | ee_id={self.ee_id}"
            )
            self._status("robot_model_loaded")
        except Exception as e:
            self.get_logger().error(f"❌ Errore caricando URDF da '{urdf_path}': {e}")
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
        if self.model is None or self.data is None or self.ee_id is None:
            return None, False, None

        self.get_logger().info(
            f"IK start: q0={q0.round(3).tolist()} "
            f"target_t={target_SE3.translation.round(3).tolist()}"
        )

        q_full = self._to_full(q0)
        lo = self.q_min[:self.nq_ctrl] + self.ik_joint_limit_margin
        hi = self.q_max[:self.nq_ctrl] - self.ik_joint_limit_margin
        err6 = np.ones(6)

        for _ in range(self.ik_max_iter):
            pin.forwardKinematics(self.model, self.data, q_full)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]
            dMi  = current.actInv(target_SE3)
            err6 = pin.log6(dMi).vector          # errore in frame LOCAL dell'EE

            # porta l'errore in world frame con la rotazione corrente
            R    = current.rotation
            err6_world = np.concatenate([R @ err6[:3], R @ err6[3:]])

            if not self.use_orientation:
                err6_world[3:] = 0.0

            if np.linalg.norm(err6_world) < self.ik_tol:
                return self._from_full(q_full), True, float(np.linalg.norm(err6_world))

            J = pin.computeFrameJacobian(self.model, self.data, q_full, self.ee_id,
                                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, :self.nq_ctrl]
            if not self.use_orientation:
                J[3:, :] = 0.0

            JJt = J @ J.T
            A   = JJt + (self.ik_damping**2) * np.eye(6)
            try:    y = np.linalg.solve(A, err6_world)
            except: y = np.linalg.lstsq(A, err6_world, rcond=None)[0]

            # segno POSITIVO: dq = J^T (J J^T + λI)^{-1} * err  (gradient descent verso target)
            qc = self._from_full(q_full) + self.ik_step * J.T @ y
            qc = np.array([_clamp(qc[i], lo[i], hi[i]) for i in range(self.nq_ctrl)])
            q_full = self._to_full(qc)


        # debug finale
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        current = self.data.oMf[self.ee_id]
        err_pos = np.linalg.norm(current.translation - target_SE3.translation)
        self.get_logger().warn(
            f"IK fail | err_pos={err_pos:.4f}m | err_rot={float(np.linalg.norm(err6[3:])):.4f}rad | "
            f"ee_pos={current.translation.round(3).tolist()}"
        )
        return self._from_full(q_full), False, float(np.linalg.norm(err6))



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
        self.get_logger().info(f"🎯 Nuovo goal IK ricevuto: pos= {goal_pose.header.frame_id}, p = {goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f}, {goal_pose.pose.position.z:.3f}")
        if self.goal_in_flight:
            self.get_logger().warn('⚠️ Nuovo goal ricevuto mentre un goal JTC è in corso: provo a cancellare il precedente')
            self._status('goal_preempt_requested')
            try:
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"⚠️ cancel_goal_async failed: {e}")
            # Unlock locally to allow the new goal to be sent
            self._unlock_goal('preempt')

        self._done(False)
        self._success(False)

        # basic readiness
        if self.model is None or self.data is None or self.ee_id is None:
            self._status("robot_model_not_ready")
            self.get_logger().error("❌ Modello robot non pronto." f"model_is_none = {self.model is None} data_is_none = {self.data is None} ee_id_is_none = {self.ee_id is None}" f"(ee_frame='{self.ee_frame}')") 
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

        if not self.use_orientation:
            # sostituisce la rotazione target con quella corrente dell'EE
            # così l'IK risolve solo posizione, errore rotazionale parte da 0
            pin.forwardKinematics(self.model, self.data, self._to_full(q0))
            pin.updateFramePlacements(self.model, self.data)
            R_current = self.data.oMf[self.ee_id].rotation.copy()
            target = pin.SE3(R_current, target.translation)


        q_sol, ok, err = self._ik_solve(target, q0)
        if not ok:
            self.get_logger().warn(f"⚠️ IK non convergente. err={err}")
            self._status("ik_failed")
            self._done(True)
            self._success(False)
            return
        
        self.get_logger().info(
            f"IK result: ok={ok}, err={err:.4g}, q_sol={q_sol.round(3).tolist() if q_sol is not None else 'None'}"
        )
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
        self.goal_sent_time = self.get_clock().now()
        send_future = self.action_client.send_goal_async(goal)

        def _goal_response_cb(fut):
            goal_handle = fut.result()
            self.current_goal_handle = goal_handle
            if not goal_handle.accepted:
                self.get_logger().error("❌ Goal JTC rifiutato dal server.")
                self._unlock_goal('rejected')
                self._success(False)
                self._done(True)
                return

            def _result_cb(rf):
                res = rf.result()
                ok = (res is not None and res.status == 4)  # 4 = SUCCEEDED
                self._status('goal_succeeded' if ok else f'goal_failed_status_{res.status if res else "none"}')
                self._success(ok)
                self._done(True)
                self._unlock_goal('result')

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