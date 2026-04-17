#!/usr/bin/env python3
"""
IK Goal Mux — priority multiplexer for /ik_goal_pose and /ik_enable.

WBC has priority over z1_FSM. When WBC is enabled (/wbc/enable=True),
only WBC goals reach z1_ik_to_jtc. When WBC is disabled, z1_FSM goals
pass through unchanged.

Topics in:
  /wbc/enable          (Bool)        — WBC priority flag
  /wbc/ik_goal_pose    (PoseStamped) — goal from wbc_qp_controller
  /wbc/ik_enable       (Bool)        — ik_enable from wbc_qp_controller
  /z1/ik_goal_pose     (PoseStamped) — goal from z1_FSM
  /z1/ik_enable        (Bool)        — ik_enable from z1_FSM

Topics out:
  /ik_goal_pose        (PoseStamped) — to z1_ik_to_jtc
  /ik_enable           (Bool)        — to z1_ik_to_jtc
"""
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import rclpy
from rclpy.node import Node


class IKGoalMuxNode(Node):

    def __init__(self):
        super().__init__('ik_goal_mux')

        self._wbc_active = False

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Bool, '/wbc/enable',
                                 self._cb_wbc_enable, 10)
        self.create_subscription(PoseStamped, '/wbc/ik_goal_pose',
                                 self._cb_wbc_goal, 10)
        self.create_subscription(Bool, '/wbc/ik_enable',
                                 self._cb_wbc_ik_enable, 10)
        self.create_subscription(PoseStamped, '/z1/ik_goal_pose',
                                 self._cb_z1_goal, 10)
        self.create_subscription(Bool, '/z1/ik_enable',
                                 self._cb_z1_ik_enable, 10)

        # ── Publishers ────────────────────────────────────────────────
        self._pub_goal   = self.create_publisher(PoseStamped, '/ik_goal_pose', 10)
        self._pub_enable = self.create_publisher(Bool,        '/ik_enable',    10)

        self.get_logger().info('IK Goal Mux ready. WBC priority: OFF')

    def _cb_wbc_enable(self, msg: Bool) -> None:
        if msg.data != self._wbc_active:
            self._wbc_active = msg.data
            self.get_logger().info(
                f'Mux priority: {"WBC" if self._wbc_active else "z1_FSM"}')
            if not self._wbc_active:
                # Release arm when WBC stops
                en = Bool(); en.data = False
                self._pub_enable.publish(en)

    def _cb_wbc_goal(self, msg: PoseStamped) -> None:
        if self._wbc_active:
            self._pub_goal.publish(msg)

    def _cb_wbc_ik_enable(self, msg: Bool) -> None:
        if self._wbc_active:
            self._pub_enable.publish(msg)

    def _cb_z1_goal(self, msg: PoseStamped) -> None:
        if not self._wbc_active:
            self._pub_goal.publish(msg)

    def _cb_z1_ik_enable(self, msg: Bool) -> None:
        if not self._wbc_active:
            self._pub_enable.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKGoalMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
