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

        urdf_path = self.get_parameter("urdf_path").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.max_iter = self.get_parameter("ik_max_iter").value
        self.tol = self.get_parameter("ik_tol").value
        self.damping = self.get_parameter("ik_damping").value

        # ---------------- PINOCCHIO MODEL ----------------
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self.ee_id = self.model.getFrameId(self.ee_frame)

        self.q = pin.neutral(self.model)

        self.get_logger().info(
            f"✅ URDF caricato: {urdf_path} | nq={self.model.nq} | ee_id={self.ee_id}"
        )

        # ---------------- SUBSCRIBERS ----------------
        self.sub_goal = self.create_subscription(
            PoseStamped,
            "/ik_goal_pose",
            self.goal_callback,
            10
        )

        # ---------------- ACTION CLIENT ----------------
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.pub_done = self.create_publisher(Bool, "/ik_jtc/done", 10)

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

            q = pin.integrate(self.model, q, v)

        self.get_logger().warn("⚠️ IK did NOT converge")
        return None

    # ==========================================================
    # SEND TRAJECTORY
    # ==========================================================
    def send_trajectory(self, q_target):

        joint_names = self.model.names[1:]  # skip universe

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.time_from_start = Duration(seconds=3.0).to_msg()

        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ JTC action server non disponibile")
            return

        self.get_logger().info("🚀 Sending trajectory to JTC")

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # ==========================================================
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return

        self.get_logger().info("✅ Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("🏁 Trajectory completed")
        self.pub_done.publish(Bool(data=True))

    # ==========================================================
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("📍 Nuovo target ricevuto")

        q_target = self.solve_ik(msg)

        if q_target is None:
            self.get_logger().error("❌ IK fallita")
            return

        self.send_trajectory(q_target)


# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Z1IKToJTC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()