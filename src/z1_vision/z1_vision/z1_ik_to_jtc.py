#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import pinocchio as pin

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.duration import Duration
from tf_transformations import quaternion_matrix


class Z1IKToJTC(Node):
    """
    Servo IK -> JointTrajectoryController (topic command)

    Input:
      - /ik_goal_pose (PoseStamped)  [pubblicato dalla FSM]
      - /target_lock_valid (Bool)    [dal tracker]

    Output:
      - /joint_trajectory_controller/joint_trajectory (JointTrajectory)
        con joint1..joint6 (q_target[:6])
    """

    def __init__(self):
        super().__init__("z1_ik_to_jtc")

        # ---------------- Params ----------------
        self.declare_parameter(
            "urdf_path",
            "/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf",
        )
        self.declare_parameter("ee_frame", "link06")

        # Servo
        self.declare_parameter("servo_rate", 10.0)     # Hz (lento e stabile)
        self.declare_parameter("traj_dt", 0.40)        # durata del micro-move (s)
        self.declare_parameter("max_q_step", 0.008)     # rad per step (clamp)

        # IK
        self.declare_parameter("ik_max_iter", 80)
        self.declare_parameter("ik_tol", 1e-4)
        self.declare_parameter("ik_damping", 1e-3)

        self.declare_parameter("slowdown_distance", 0.10)  # m
        self.declare_parameter("min_step_scale", 0.15)     # non scendere sotto questo fattore

        urdf_path = self.get_parameter("urdf_path").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.servo_rate = float(self.get_parameter("servo_rate").value)
        self.traj_dt = float(self.get_parameter("traj_dt").value)
        self.max_q_step = float(self.get_parameter("max_q_step").value)

        self.ik_max_iter = int(self.get_parameter("ik_max_iter").value)
        self.ik_tol = float(self.get_parameter("ik_tol").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)

        self.slowdown_distance = float(self.get_parameter("slowdown_distance").value)
        self.min_step_scale = float(self.get_parameter("min_step_scale").value)

        # ---------------- Pinocchio ----------------
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId(self.ee_frame)

        self.q = pin.neutral(self.model)

        self.get_logger().info(
            f"✅ URDF caricato: {urdf_path} | nq={self.model.nq} | ee_frame={self.ee_frame} (id={self.ee_id})"
        )

        # ---------------- State ----------------
        self.lock_valid = False
        self.goal_pose: PoseStamped | None = None

        # ---------------- I/O ----------------
        self.create_subscription(PoseStamped, "/ik_goal_pose", self.cb_goal_pose, 10)
        self.create_subscription(Bool, "/target_lock_valid", self.cb_lock, 10)

        # Topic command del JointTrajectoryController
        self.pub_cmd = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # Servo timer
        self.dt = 1.0 / self.servo_rate
        self.timer = self.create_timer(self.dt, self.servo_tick)

        self.get_logger().info(
            f"🦾 Servo IK attivo: rate={self.servo_rate:.1f}Hz, traj_dt={self.traj_dt:.2f}s, joints=joint1..joint6"
        )

    # ---------------- Callbacks ----------------
    def cb_lock(self, msg: Bool):
        self.lock_valid = bool(msg.data)

    def cb_goal_pose(self, msg: PoseStamped):
        self.goal_pose = msg

    # ---------------- IK ----------------
    def solve_ik(self, target_SE3: pin.SE3, q_init: np.ndarray) -> np.ndarray | None:
        q = q_init.copy()

        for _ in range(self.ik_max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]
            err = pin.log(current.inverse() * target_SE3).vector  # 6D

            if np.linalg.norm(err) < self.ik_tol:
                return q

            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id, pin.ReferenceFrame.LOCAL
            )
            JJt = J @ J.T + self.ik_damping * np.eye(6)
            v = J.T @ np.linalg.solve(JJt, err)

            q = pin.integrate(self.model, q, v)

        return None

    # ---------------- Servo loop ----------------
    def servo_tick(self):
        if not self.lock_valid:
            return
        if self.goal_pose is None:
            return

        # PoseStamped -> SE3
        T = quaternion_matrix([
            self.goal_pose.pose.orientation.x,
            self.goal_pose.pose.orientation.y,
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w
        ])
        T[:3, 3] = [
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y,
            self.goal_pose.pose.position.z
        ]
        target = pin.SE3(T[:3, :3], T[:3, 3])

        q_sol = self.solve_ik(target, self.q)
        if q_sol is None:
            self.get_logger().warn("⚠️ IK non converge (tick)", throttle_duration_sec=1.0)
            return

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        current = self.data.oMf[self.ee_id] 
        pos_err = np.linalg.norm(current.translation - target.translation)
        s = np.clip(pos_err / self.slowdown_distance, self.min_step_scale, 1.0)

        # Clamp step (per muoversi lentamente e senza scatti)
        dq = pin.difference(self.model, self.q, q_sol)
        dq = np.clip(dq, -self.max_q_step, self.max_q_step)
        dq = dq * s
        self.q = pin.integrate(self.model, self.q, dq)

        # Pubblica comando al controller SOLO joint1..joint6
        self.publish_jtc(self.q)

    def publish_jtc(self, q_cmd: np.ndarray):
        msg = JointTrajectory()
        msg.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]

        pt = JointTrajectoryPoint()
        pt.positions = q_cmd[:6].tolist()  # ✅ taglio 7° joint
        pt.time_from_start = Duration(seconds=self.traj_dt).to_msg()

        msg.points = [pt]
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Z1IKToJTC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()