import rclpy
from rclpy.node import Node
from .joint_state_monitor import JointStateMonitor


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Z1StateDebug(Node):
    def __init__(self):
        super().__init__("z1_state_debug")
        self.mon = JointStateMonitor(self, JOINT_NAMES)
        self.create_timer(1.0, self.tick)

    def tick(self):
        if not self.mon.ready:
            self.get_logger().info("Waiting for /joint_states...")
            return
        q = self.mon.get_positions()
        self.get_logger().info(f"q = {[round(v, 3) for v in q]}")


def main():
    rclpy.init()
    node = Z1StateDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
