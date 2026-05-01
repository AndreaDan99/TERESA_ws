#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import pinocchio as pin

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf_transformations import quaternion_matrix
from sensor_msgs.msg import JointState


class Z1IKToJTC(Node):

    def __init__(self):
        super().__init__("z1_ik_to_jtc")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ee_frame", "link06")

        self.declare_parameter("ik_max_iter", 100)
        self.declare_parameter("ik_tol", 1e-4)
        self.declare_parameter("ik_damping", 5e-3)

        # ---------------- FSM INTERFACE PARAMS ----------------
        self.declare_parameter("ik_goal_topic", "/ik_goal_pose")
        self.declare_parameter("ik_enable_topic", "/ik_enable")
        self.declare_parameter("ik_done_topic", "/ik_done")

        # Trajectory shaping (pulito)
        self.declare_parameter("max_joint_vel", 0.3)   # rad/s
        self.declare_parameter("traj_min_time", 1.0)    # s
        self.declare_parameter("traj_max_time", 10.0)   # s

        self.declare_parameter("ik_alpha", 0.50)
        self.declare_parameter("ik_rot_weight", 0.5)  # peso errore angolare (0=solo pos, 1=uguale a pos)

        urdf_path = self.get_parameter("urdf_path").value
        if not urdf_path:
            import os
            from ament_index_python.packages import get_package_share_directory
            urdf_path = os.path.join(get_package_share_directory('z1_description'), 'urdf', 'z1.urdf')
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.max_iter = self.get_parameter("ik_max_iter").value
        self.tol = self.get_parameter("ik_tol").value
        self.damping = self.get_parameter("ik_damping").value

        self.ik_goal_topic = self.get_parameter("ik_goal_topic").value
        self.ik_enable_topic = self.get_parameter("ik_enable_topic").value
        self.ik_done_topic = self.get_parameter("ik_done_topic").value

        self.max_joint_vel = float(self.get_parameter("max_joint_vel").value)
        self.traj_min_time = float(self.get_parameter("traj_min_time").value)
        self.traj_max_time = float(self.get_parameter("traj_max_time").value)

        self.ik_alpha      = float(self.get_parameter("ik_alpha").value)
        self.ik_rot_weight = float(self.get_parameter("ik_rot_weight").value)

        # FSM gating + goal latching
        self.ik_enabled = False
        self.busy = False
        self.last_goal: PoseStamped | None = None

        self.declare_parameter("traj_points", 25)  # >=3
        self.traj_points = int(self.get_parameter("traj_points").value)
        self.traj_points = max(self.traj_points, 3)


        # ---------------- PINOCCHIO MODEL ----------------
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self.ee_id = self.model.getFrameId(self.ee_frame)

        self.q = pin.neutral(self.model)

        self.get_logger().info(
            f"✅ URDF caricato: {urdf_path} | nq={self.model.nq} | ee_id={self.ee_id}"
        )
        self.get_logger().info("🧩 FSM interface:")
        self.get_logger().info(f"  goal:   {self.ik_goal_topic}")
        self.get_logger().info(f"  enable: {self.ik_enable_topic}")
        self.get_logger().info(f"  done:   {self.ik_done_topic}")

        # Joint state reale (per smoothness)
        self.q_meas = None  # np.ndarray shape (6,)
        self.have_js = False
        self.joint_order = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # ---------------- SUBSCRIBERS ----------------
        self.sub_goal = self.create_subscription(
            PoseStamped,
            self.ik_goal_topic,
            self.goal_callback,
            10
        )

        self.sub_enable = self.create_subscription(
            Bool,
            self.ik_enable_topic,
            self.enable_callback,
            10
        )
        self.sub_js = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            50
        )

        # ---------------- ACTION CLIENT ----------------
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.pub_done = self.create_publisher(Bool, self.ik_done_topic, 10)
    # ==========================================================
    # IK SOLVER
    # ==========================================================
    def solve_ik(self, target_pose):

        # Convert PoseStamped to SE3
        T = quaternion_matrix([
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        ])
        T[:3, 3] = [
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        ]

        target_SE3 = pin.SE3(T[:3, :3], T[:3, 3])

        q = self.q.copy()
        if self.have_js and self.q_meas is not None:
            q[:6] = self.q_meas
        for i in range(self.max_iter):

            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]

            err = pin.log(current.inverse() * target_SE3).vector
            err[:3] *= self.ik_rot_weight   # scala errore orientamento (primi 3 = ang, ultimi 3 = lin)

            if np.linalg.norm(err) < self.tol:
                self.get_logger().info(f"🎯 IK converged in {i} iter")
                self.q = q
                return q

            J = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_id,
                pin.ReferenceFrame.LOCAL
            )

            JJt = J @ J.T + self.damping * np.eye(6)
            v = J.T @ np.linalg.solve(JJt, err)

            q = pin.integrate(self.model, q, self.ik_alpha * v)

        self.get_logger().warn("⚠️ IK did NOT converge")
        return None

    def _wrap_to_pi(self, a: np.ndarray) -> np.ndarray:
        return (a + np.pi) % (2.0 * np.pi) - np.pi

    def _make_target_near(self, q_target: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
        """
        Sposta q_target di multipli di 2π per minimizzare |q_target - q_ref|.
        """
        dq = q_target - q_ref
        dq = self._wrap_to_pi(dq)
        return q_ref + dq
    # ==========================================================
    # SEND TRAJECTORY
    # ==========================================================

    def send_trajectory(self, q_target) -> bool:
        q0 = self.q_meas.copy()
        qf = np.array(q_target[:6], dtype=float)

        qf_old = qf.copy()
        qf = self._make_target_near(qf, q0)

        dq_before = float(np.max(np.abs(qf_old - q0)))
        dq_after  = float(np.max(np.abs(qf - q0)))
        if dq_after < dq_before - 1e-6:
            self.get_logger().warn(f"🔁 unwrap target: dq {dq_before:.3f} -> {dq_after:.3f} rad")
             
        dq = float(np.max(np.abs(qf - q0)))
        dq = max(dq, 1e-6)

        # tempo coerente con max velocità
        T = dq / max(self.max_joint_vel, 1e-6)
        T = float(np.clip(T, self.traj_min_time, self.traj_max_time))

        N = self.traj_points  # es. 11
        traj = JointTrajectory()
        traj.joint_names = self.joint_order
        traj.points = []

        # smoothstep: s(t)=3t^2-2t^3 (zero vel a inizio/fine, meno jerk percepito)
        for k in range(N):
            t = k / (N - 1)
            s = 10*t**3 - 15*t**4 + 6*t**5 # partito da s = 3*t**2 - 2*t**3
            qk = q0 + s * (qf - q0)

            p = JointTrajectoryPoint()
            p.positions = qk.tolist()
            p.time_from_start = Duration(seconds=T * t).to_msg()

            # velocità solo sull'ultimo punto per "arrivo morbido"
            if k == 0 or k == N - 1:
                p.velocities = [0.0] * 6

            traj.points.append(p)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ JTC action server non disponibile")
            return False

        self.get_logger().info(f"🚀 Sending smooth trajectory | N={N} | T={T:.2f}s | dq_max={dq:.3f} rad")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True
    # ==========================================================

    def joint_state_callback(self, msg: JointState):
        # mappa name->pos
        name_to_pos = dict(zip(msg.name, msg.position))

        try:
            q = np.array([name_to_pos[n] for n in self.joint_order], dtype=float)
        except KeyError:
            return  # non tutti i joint presenti

        self.q_meas = q
        self.have_js = True

    def enable_callback(self, msg: Bool):
        self.ik_enabled = bool(msg.data)

        if not self.ik_enabled:
            # disarm: richiedi una nuova posa + nuova enable
            self.last_goal = None
            self.busy = False
            self.get_logger().info("🛑 IK disabled (disarmed)")
            return

        self.get_logger().info("✅ IK enabled")
        self._try_start()

    def _try_start(self):
        # parte solo se: enabled + goal presente + non già in esecuzione
        if not self.have_js:
            self.get_logger().warn("⚠️ Aspetto /joint_states prima di partire (serve per smoothing)")
            return
        if not self.ik_enabled:
            return
        if self.busy:
            return
        if self.last_goal is None:
            return

        self.busy = True
        goal_msg = self.last_goal

        self.get_logger().info("🧠 Starting IK+JTC on latched goal")
        q_target = self.solve_ik(goal_msg)

        if q_target is None:
            # FAIL: non pubblichiamo /ik_done (la FSM tornerà waiting per freshness)
            self.get_logger().error("❌ IK fallita -> disarmo e attendo nuova posa")
            self.busy = False
            self.last_goal = None
            self.ik_enabled = False
            return

        ok = self.send_trajectory(q_target)
        if not ok:
            # FAIL: non pubblichiamo /ik_done
            self.get_logger().error("❌ Trajectory non inviata -> disarmo e attendo nuova posa")
            self.busy = False
            self.last_goal = None
            self.ik_enabled = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected -> disarmo e attendo nuova posa")
            self.busy = False
            self.last_goal = None
            self.ik_enabled = False
            return

        self.get_logger().info("✅ Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result().result
            # error_code == 0 → SUCCESSFUL
            if hasattr(result, 'error_code') and result.error_code != 0:
                self.get_logger().warn(
                    f"⚠️  JTC non convergente (error_code={result.error_code}: "
                    f"{getattr(result, 'error_string', '')}) "
                    f"→ ik_done=True per skip posa"
                )
            else:
                self.get_logger().info("🏁 Trajectory completed")
        except Exception as e:
            self.get_logger().warn(f"⚠️  result_callback exception: {e}")

        # Pubblica sempre ik_done=True (successo o fallimento) così
        # il scanner può avanzare alla posa successiva senza bloccarsi.
        self.pub_done.publish(Bool(data=True))

        # reset interno
        self.busy = False
        self.last_goal = None
        self.ik_enabled = False

    # ==========================================================
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("📍 Nuovo target ricevuto (latched)")
        self.last_goal = msg

        # se già abilitato, prova a partire
        self._try_start()


# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Z1IKToJTC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()