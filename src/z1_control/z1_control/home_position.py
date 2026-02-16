#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import time


JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]

HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MOVE_TIME = 3.0
TOLERANCE = 0.01


class TestHomePositionAction(Node):

    def __init__(self):
        super().__init__('test_home_position')

        # Action client verso il controller
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()
        self.get_logger().info("Action server ready.")

        # Joint states
        self.current_q = None
        self.received_state = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.get_logger().info("Waiting for joint_states...")
        while rclpy.ok() and not self.received_state:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Initial position: {self.current_q}")

    def joint_state_cb(self, msg):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q = [0.0] * 6
        for i, jn in enumerate(JOINT_NAMES):
            if jn in name_to_idx:
                q[i] = msg.position[name_to_idx[jn]]
        self.current_q = q
        self.received_state = True

    def send_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        # Punto iniziale
        p0 = JointTrajectoryPoint()
        p0.positions = list(self.current_q)
        p0.time_from_start.sec = 0

        # Punto finale (home)
        p1 = JointTrajectoryPoint()
        p1.positions = HOME_POSITION
        p1.time_from_start.sec = int(MOVE_TIME)
        p1.time_from_start.nanosec = int((MOVE_TIME % 1) * 1e9)

        goal.trajectory.points = [p0, p1]

        self.get_logger().info(f"Sending robot to HOME via action: {HOME_POSITION}")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller!")
            return None

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Action result: {result.result.error_code}")

        return result

    def wait_and_check(self):
        # Aspetta che il movimento finisca e acquisisci qualche joint_state
        self.get_logger().info(f"Waiting {MOVE_TIME + 1.0}s for motion to complete...")
        time.sleep(MOVE_TIME + 1.0)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Final position: {self.current_q}")

        errors = [abs(self.current_q[i] - HOME_POSITION[i]) for i in range(6)]
        max_error = max(errors)

        self.get_logger().info(f"Tracking errors (rad): {errors}")
        self.get_logger().info(f"Max error: {max_error:.4f} rad")

        if max_error < TOLERANCE:
            self.get_logger().info("✅ TEST PASSED: Robot reached HOME within tolerance")
            return True
        else:
            self.get_logger().warn(f"⚠️  TEST FAILED: Max error {max_error:.4f} > tolerance {TOLERANCE}")
            return False

    def run_test(self):
        result = self.send_home()
        if result is None:
            return False
        return self.wait_and_check()


def main():
    rclpy.init()
    node = TestHomePositionAction()
    success = node.run_test()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if success else 1)


if __name__ == '__main__':
    main()

