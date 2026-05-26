#!/usr/bin/env python3
"""
WBC Approach Scanner — DEPRECATED passive node.

All functionality has moved to wbc_qp_controller.py:
  - Pose generation (QP null-space grid)
  - Sequencing (BodySearchScanner tick loop)
  - Data collection + FAST point publishing

This node remains as a stub for launch-file compatibility.
Remove from setup.py entry points and launch files when ready.
"""
import rclpy
from rclpy.node import Node


class WBCApproachScanner(Node):

    def __init__(self):
        super().__init__('wbc_approach_scanner')
        self.get_logger().warn(
            'WBC Approach Scanner is DEPRECATED. '
            'All functionality has moved to wbc_qp_controller.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WBCApproachScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
