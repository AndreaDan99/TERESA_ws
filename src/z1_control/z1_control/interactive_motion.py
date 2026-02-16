import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import sys
import termios
import tty
import time


JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]

DELTA = 0.05        # incremento rad
DURATION = 0.5      # tempo movimento


# --------------------------------------------------
# Lettura singolo tasto (no Enter)
# --------------------------------------------------
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


# --------------------------------------------------
class Z1KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('z1_keyboard_teleop')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info("Waiting for controller...")
        self.client.wait_for_server()
        self.get_logger().info("Controller ready.")

        self.current_positions = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

    # --------------------------------------------------

    def joint_state_cb(self, msg):
        if self.current_positions is None:
            self.current_positions = {
                name: pos for name, pos in zip(msg.name, msg.position)
                if name in JOINT_NAMES
            }
            self.get_logger().info("Joint states received. Ready.")

    # --------------------------------------------------

    def send_current_position(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [self.current_positions[j] for j in JOINT_NAMES]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(DURATION * 1e9)

        goal.trajectory.points = [point]
        self.client.send_goal_async(goal)

    # --------------------------------------------------

    def update_joint(self, joint_index, delta):
        joint = JOINT_NAMES[joint_index]
        self.current_positions[joint] += delta
        self.send_current_position()


# --------------------------------------------------
def print_instructions():
    print("""
================ Z1 KEYBOARD TELEOP ================

Joint 1 : q / a
Joint 2 : w / s
Joint 3 : e / d
Joint 4 : r / f
Joint 5 : t / g
Joint 6 : y / h

x : EXIT

====================================================
""")


def main():
    rclpy.init()
    node = Z1KeyboardTeleop()

    print_instructions()

    # aspetta joint_states
    while rclpy.ok() and node.current_positions is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        while rclpy.ok():
            key = get_key()

            if key == 'q':
                node.update_joint(0, +DELTA)
            elif key == 'a':
                node.update_joint(0, -DELTA)

            elif key == 'w':
                node.update_joint(1, +DELTA)
            elif key == 's':
                node.update_joint(1, -DELTA)

            elif key == 'e':
                node.update_joint(2, +DELTA)
            elif key == 'd':
                node.update_joint(2, -DELTA)

            elif key == 'r':
                node.update_joint(3, +DELTA)
            elif key == 'f':
                node.update_joint(3, -DELTA)

            elif key == 't':
                node.update_joint(4, +DELTA)
            elif key == 'g':
                node.update_joint(4, -DELTA)

            elif key == 'y':
                node.update_joint(5, +DELTA)
            elif key == 'h':
                node.update_joint(5, -DELTA)

            elif key == 'x':
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
