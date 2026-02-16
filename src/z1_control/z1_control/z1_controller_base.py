from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class Z1ControllerBase:
    def __init__(self, node, joint_names):
        self.node = node
        self.joint_names = joint_names

        self.client = ActionClient(
            node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.node.get_logger().info("Waiting for joint_trajectory_controller...")
        self.client.wait_for_server()
        self.node.get_logger().info("Controller ready.")

    def send_target(self, target_positions, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = target_positions
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        goal.trajectory.points.append(pt)
        self.client.send_goal_async(goal)

    def send_delta(self, current_positions, delta, duration=1.0):
        target = [q + dq for q, dq in zip(current_positions, delta)]
        self.send_target(target, duration=duration)
