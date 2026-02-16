import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import sys
import termios
import tty
import threading
import time


JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]

STEP = 0.05          # rad
MOVE_TIME = 2.0      # sec


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Z1IncrementalController(Node):

    def __init__(self):
        super().__init__('z1_incremental_controller')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.q = [0.0] * 6
        self.received_state = False

        self.get_logger().info("Waiting for joint_states...")
        while rclpy.ok() and not self.received_state:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Initial q = {self.q}")
        self.print_help()

        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def joint_state_cb(self, msg):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, jn in enumerate(JOINT_NAMES):
            if jn in name_to_idx:
                self.q[i] = msg.position[name_to_idx[jn]]
        self.received_state = True

    def send_trajectory(self, q_target):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        p0 = JointTrajectoryPoint()
        p0.positions = list(self.q)
        p0.time_from_start.sec = 0

        p1 = JointTrajectoryPoint()
        p1.positions = list(q_target)
        p1.time_from_start.sec = int(MOVE_TIME)

        traj.points = [p0, p1]

        self.pub.publish(traj)
        self.q = list(q_target)

    def keyboard_loop(self):
        joint_idx = 1  # partiamo da joint2

        while rclpy.ok():
            key = get_key()

            if key == 'q':
                break

            elif key in ['1','2','3','4','5','6']:
                joint_idx = int(key) - 1
                self.get_logger().info(f"Selected joint {joint_idx+1}")

            elif key == 'w':
                q_new = list(self.q)
                q_new[joint_idx] += STEP
                self.send_trajectory(q_new)

            elif key == 's':
                q_new = list(self.q)
                q_new[joint_idx] -= STEP
                self.send_trajectory(q_new)

            elif key == 'h':
                self.print_help()

            time.sleep(0.05)

    def print_help(self):
        print("""
========= Z1 Incremental Control =========
1..6  -> select joint
w     -> + step
s     -> - step
h     -> help
q     -> quit
=========================================
""")


def main():
    rclpy.init()
    node = Z1IncrementalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
