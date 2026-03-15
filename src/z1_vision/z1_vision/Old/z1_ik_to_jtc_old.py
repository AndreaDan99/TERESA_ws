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
from action_msgs.msg import GoalStatus

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
      - /ik_goal_pose (PoseStamped): desired EE pose (any frame)
      - /joint_states (JointState): current joints
    Action:
      - /joint_trajectory_controller/follow_joint_trajectory
    Publish:
      - /ik_jtc/done (Bool)
      - /ik_jtc/success (Bool)
      - /ik_jtc/status (String)
    """

    def __init__(self):
        super().__init__('z1_ik_to_jtc')

        # ---------------- Parameters ----------------
        self.declare_parameter('urdf_path', '/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf')

        # IMPORTANT: base_frame deve essere il frame base reale (quello root del robot nel TF).
        # NON usare 'world' se il robot nel TF è in 'link00'/'base' ecc.
        self.declare_parameter('base_frame', 'link00')

        self.declare_parameter('ee_frame', 'link06')
        self.declare_parameter('controller_action', '/joint_trajectory_controller/follow_joint_trajectory')

        self.declare_parameter('joint_names', ['joint1','joint2','joint3','joint4','joint5','joint6'])

        # trajectory
        self.declare_parameter('traj_duration', 3.0)
        self.declare_parameter('traj_points', 40)

        # IK
        self.declare_parameter('ik_max_iter', 80)
        self.declare_parameter('ik_tol', 1e-4)
        self.declare_parameter('ik_damping', 1e-3)
        self.declare_parameter('ik_step', 0.6)  # un po' più conservativo di 0.8
        self.declare_parameter('use_orientation', True)

        # Safety / preemption
        self.declare_parameter('goal_timeout_sec', 25.0)
        self.declare_parameter('min_goal_dist_to_send', 0.005)  # evita spam goals quasi uguali

        # ---------------- Read parameters ----------------
        self.urdf_path = self.get_parameter('urdf_path').value
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
        self.use_orientation = bool(self.get_parameter('use_orientation').value)

        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.min_goal_dist_to_send = float(self.get_parameter('min_goal_dist_to_send').value)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- State ----------------
        self.last_joint_state: JointState | None = None

        self.model_full = None
        self.model = None         # reduced model (solo joint del controller)
        self.data = None
        self.ee_id = None

        self.q_min = None
        self.q_max = None
        self.nq = None

        # action goal tracking
        self.goal_in_flight = False
        self.current_goal_handle = None
        self.goal_sent_time = None

        # last goal pose in base frame (to avoid spam)
        self._last_goal_pos = None  # np.array(3)

        # ---------------- ROS interfaces ----------------
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/ik_goal_pose', self.cb_goal_pose, 10)

        self.pub_success = self.create_publisher(Bool, '/ik_jtc/success', 10)
        self.pub_done = self.create_publisher(Bool, '/ik_jtc/done', 10)
        self.pub_status = self.create_publisher(String, '/ik_jtc/status', 10)

        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller_action)

        # watchdog
        self._goal_watchdog_timer = self.create_timer(0.2, self._goal_watchdog)

        # init pin model
        self._init_pinocchio_models()

        self.get_logger().info(
            "🦾 z1_ik_to_jtc (ROBUST) pronto\n"
            f"  base_frame: {self.base_frame}\n"
            f"  ee_frame:   {self.ee_frame}\n"
            f"  joints:     {self.joint_names}\n"
            f"  action:     {self.controller_action}\n"
            f"  IK: it={self.ik_max_iter} tol={self.ik_tol} damp={self.ik_damping} step={self.ik_step} use_ori={self.use_orientation}"
        )

    # ---------------- Pub helpers ----------------
    def _status(self, text: str):
        self.pub_status.publish(String(data=text))

    def _success(self, ok: bool):
        self.pub_success.publish(Bool(data=bool(ok)))

    def _done(self, done: bool):
        self.pub_done.publish(Bool(data=bool(done)))

    # ---------------- Joint states ----------------
    def cb_joint_states(self, msg: JointState):
        self.last_joint_state = msg

    def _get_current_q(self):
        """Return q vector ordered as self.joint_names (for reduced model)."""
        if self.last_joint_state is None:
            return None
        name_to_idx = {n: i for i, n in enumerate(self.last_joint_state.name)}
        q = []
        for jn in self.joint_names:
            if jn not in name_to_idx:
                return None
            q.append(self.last_joint_state.position[name_to_idx[jn]])
        return np.array(q, dtype=np.float64)

    # ---------------- TF ----------------
    def _transform_pose_to_base(self, pose_in: PoseStamped) -> PoseStamped | None:
        if pose_in.header.frame_id == '' or pose_in.header.frame_id == self.base_frame:
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

    # ---------------- Pinocchio init ----------------
    def _init_pinocchio_models(self):
        if not self.urdf_path:
            self.get_logger().error("❌ urdf_path vuoto")
            self._status("urdf_path_missing")
            return

        try:
            self.model_full = pin.buildModelFromUrdf(self.urdf_path)

            # Costruisci reduced model: lock di TUTTI i joint non controllati
            # (così nq == len(joint_names) e non devi fare hack tipo append(0.0))
            # pin joint ids: model_full.getJointId(name)
            ctrl_ids = []
            for jn in self.joint_names:
                jid = self.model_full.getJointId(jn)
                if jid == 0:
                    raise RuntimeError(f"Joint '{jn}' non trovato in URDF")
                ctrl_ids.append(jid)

            # lock list = tutti i joint (tranne universe=0) non in ctrl_ids
            lock_ids = []
            for jid in range(1, self.model_full.njoints):
                if jid not in ctrl_ids:
                    lock_ids.append(jid)

            q0_full = pin.neutral(self.model_full)
            self.model = pin.buildReducedModel(self.model_full, lock_ids, q0_full)
            self.data = self.model.createData()

            self.ee_id = self.model.getFrameId(self.ee_frame)
            if self.ee_id >= self.model.nframes:
                raise RuntimeError(f"Frame EE '{self.ee_frame}' non trovato nel reduced model")

            self.q_min = self.model.lowerPositionLimit.copy()
            self.q_max = self.model.upperPositionLimit.copy()
            self.nq = self.model.nq

            self.get_logger().info(
                f"✅ Pinocchio ready | full nq={self.model_full.nq} -> reduced nq={self.model.nq} | ee_id={self.ee_id}"
            )
            self._status("robot_model_loaded")

        except Exception as e:
            self.get_logger().error(f"❌ Errore init Pinocchio: {e}")
            self._status("robot_model_error")
            self.model = None
            self.data = None
            self.ee_id = None

    def _pose_to_SE3(self, pose_msg: PoseStamped) -> pin.SE3:
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        R = pin.Quaternion(q.w, q.x, q.y, q.z).toRotationMatrix()
        t = np.array([p.x, p.y, p.z], dtype=np.float64)
        return pin.SE3(R, t)

    # ---------------- IK core ----------------
    def _ik_solve(self, target: pin.SE3, q0: np.ndarray):
        if self.model is None or self.data is None or self.ee_id is None:
            return None, False, None

        lo = self.q_min + 1e-4
        hi = self.q_max - 1e-4

        q = q0.copy()
        last_norm = None

        for _ in range(self.ik_max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]
            dMi = current.actInv(target)
            err6 = pin.log6(dMi).vector  # LOCAL

            # convert to world-aligned error
            R = current.rotation
            err6_world = np.concatenate([R @ err6[:3], R @ err6[3:]])

            if not self.use_orientation:
                err6_world[3:] = 0.0

            nrm = float(np.linalg.norm(err6_world))
            if nrm < self.ik_tol:
                return q, True, nrm

            # Jacobian in LOCAL_WORLD_ALIGNED
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )

            if not self.use_orientation:
                J[3:, :] = 0.0

            A = (J @ J.T) + (self.ik_damping**2) * np.eye(6)
            y = np.linalg.solve(A, err6_world)
            dq = self.ik_step * (J.T @ y)

            q = q + dq
            q = np.array([_clamp(q[i], lo[i], hi[i]) for i in range(self.nq)], dtype=np.float64)

            # semplice “divergence guard”
            if last_norm is not None and nrm > last_norm * 1.5:
                # se esplode, stop prima
                break
            last_norm = nrm

        return q, False, last_norm

    # ---------------- Trajectory ----------------
    def _build_trajectory(self, q_start: np.ndarray, q_goal: np.ndarray) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        T = max(0.5, float(self.traj_duration))
        N = max(2, int(self.traj_points))

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

    # ---------------- Action helpers ----------------
    def _unlock_goal(self, reason: str):
        self.get_logger().warn(f"🔓 Unlock goal_in_flight ({reason})")
        self.goal_in_flight = False
        self.current_goal_handle = None
        self.goal_sent_time = None

    def _goal_watchdog(self):
        if not self.goal_in_flight or self.goal_sent_time is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.goal_sent_time).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec:
            self.get_logger().error(f"❌ goal_timeout > {self.goal_timeout_sec}s")
            self._status("goal_timeout")
            # IMPORTANT: qui pubblichiamo fallimento “pulito”
            self._success(False)
            self._done(True)
            try:
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._unlock_goal("timeout")

    # ---------------- Main callback ----------------
    def cb_goal_pose(self, goal_pose: PoseStamped):
        if self.model is None or self.data is None or self.ee_id is None:
            self._status("robot_model_not_ready")
            self._success(False)
            self._done(True)
            return

        q0 = self._get_current_q()
        if q0 is None or len(q0) != self.nq:
            self.get_logger().warn("⚠️ joint_states missing joints o dimensione mismatch")
            self._status("joint_state_missing")
            self._success(False)
            self._done(True)
            return

        pose_base = self._transform_pose_to_base(goal_pose)
        if pose_base is None:
            self._status("tf_missing")
            self._success(False)
            self._done(True)
            return

        # anti-spam goals quasi uguali
        p = pose_base.pose.position
        p_now = np.array([p.x, p.y, p.z], dtype=np.float64)
        if self._last_goal_pos is not None:
            if float(np.linalg.norm(p_now - self._last_goal_pos)) < self.min_goal_dist_to_send:
                # ignora goal troppo simile
                return
        self._last_goal_pos = p_now.copy()

        # Preempt (cancel old goal) ma senza far casino con status intermedi
        if self.goal_in_flight:
            self.get_logger().warn("⚠️ Preempt: cancello goal precedente")
            self._status("goal_preempt_requested")
            try:
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._unlock_goal("preempt")

        # reset outputs per questo goal
        self._done(False)
        self._success(False)

        target = self._pose_to_SE3(pose_base)

        # se orientation disabilitata: usa rotazione corrente dell’EE
        if not self.use_orientation:
            pin.forwardKinematics(self.model, self.data, q0)
            pin.updateFramePlacements(self.model, self.data)
            R_current = self.data.oMf[self.ee_id].rotation.copy()
            target = pin.SE3(R_current, target.translation)

        self.get_logger().info(
            f"🎯 Goal in {self.base_frame}: p={p_now.round(3).tolist()} use_ori={self.use_orientation}"
        )

        q_sol, ok, err = self._ik_solve(target, q0)
        if not ok:
            self.get_logger().error(f"❌ IK fail (err={err}) | goal={p_now.round(3).tolist()}")
            self._status("ik_failed")
            self._success(False)
            self._done(True)
            return

        traj = self._build_trajectory(q0, q_sol)

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ JTC action server non disponibile")
            self._status("jtc_action_missing")
            self._success(False)
            self._done(True)
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
                self.get_logger().error("❌ Goal rifiutato dal JTC")
                self._status("goal_rejected")
                self._success(False)
                self._done(True)
                self._unlock_goal("rejected")
                return

            def _result_cb(rf):
                res = rf.result()  # GetResult.Response
                status = res.status if res is not None else -1
                error_code = res.result.error_code if (res and res.result) else -1
                error_string = res.result.error_string if (res and res.result) else ""

                ok = (status == GoalStatus.STATUS_SUCCEEDED) and (error_code == 0)

                self.get_logger().info(
                    f"✅ JTC result: status={status} error_code={error_code} err='{error_string}' ok={ok}"
                )
                self._status("goal_succeeded" if ok else f"goal_failed_status_{status}_ecode_{error_code}")
                self._success(ok)
                self._done(True)
                self._unlock_goal("result")

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