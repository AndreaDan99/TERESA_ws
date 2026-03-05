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
        self.declare_parameter("urdf_path",
            "/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ee_frame", "link06")

        self.declare_parameter("ik_max_iter", 100)
        self.declare_parameter("ik_tol", 1e-4)
        self.declare_parameter("ik_damping", 1e-3)

        # ---------------- FSM INTERFACE PARAMS ----------------
        self.declare_parameter("ik_goal_topic", "/ik_goal_pose")
        self.declare_parameter("ik_enable_topic", "/ik_enable")
        self.declare_parameter("ik_done_topic", "/ik_done")

        # Trajectory shaping (pulito)
        self.declare_parameter("max_joint_vel", 0.25)   # rad/s
        self.declare_parameter("traj_min_time", 1.0)    # s
        self.declare_parameter("traj_max_time", 10.0)   # s

        self.declare_parameter("ik_alpha", 0.3)

        urdf_path = self.get_parameter("urdf_path").value
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

        self.ik_alpha = float(self.get_parameter("ik_alpha").value)

        self.traj_time_scale = float(self.get_parameter("traj_time_scale").value)

        # FSM gating + goal latching
        self.ik_enabled = False
        self.busy = False
        self.last_goal: PoseStamped | None = None

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

        for i in range(self.max_iter):

            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]

            err = pin.log(current.inverse() * target_SE3).vector

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

    # ==========================================================
    # SEND TRAJECTORY
    # ==========================================================
    def send_trajectory(self, q_target) -> bool:
        # start reale dal robot
        q0 = self.q_meas.copy()  # (6,)
        q2 = np.array(q_target[:6], dtype=float)
        q1 = 0.5 * (q0 + q2)

        dq = float(np.max(np.abs(q2 - q0)))
        dq = max(dq, 1e-6)  # evita 0

        # tempo coerente con max velocità
        T = dq / max(self.max_joint_vel, 1e-6)
        T = float(np.clip(T, self.traj_min_time, self.traj_max_time))

        traj = JointTrajectory()
        traj.joint_names = self.joint_order

        p0 = JointTrajectoryPoint()
        p0.positions = q0.tolist()
        p0.time_from_start = Duration(seconds=0.0).to_msg()

        p1 = JointTrajectoryPoint()
        p1.positions = q1.tolist()
        p1.time_from_start = Duration(seconds=T * 0.5).to_msg()

        p2 = JointTrajectoryPoint()
        p2.positions = q2.tolist()
        p2.time_from_start = Duration(seconds=T).to_msg()
        p2.velocities = [0.0] * 6  # stop morbido in arrivo

        traj.points = [p0, p1, p2]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ JTC action server non disponibile")
            return False

        self.get_logger().info(f"🚀 Sending 3-point trajectory | T={T:.2f}s | dq_max={dq:.3f} rad")
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
        self.get_logger().info("🏁 Trajectory completed -> publishing /ik_done=True")
        self.pub_done.publish(Bool(data=True))

        # reset interno: attendo nuovo ciclo FSM (nuovo goal + enable)
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