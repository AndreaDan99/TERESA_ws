#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class Z1PDEffortNode(Node):
    def __init__(self):
        super().__init__('z1_pd_effort_node')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.n_joints = len(self.joint_names)

        # Gains
        self.Kp = np.array([20.0, 20.0, 15.0, 10.0, 5.0, 3.0])
        self.Kd = np.array([1.0, 1.0, 0.8, 0.5, 0.3, 0.2])

        self.torque_max = 30.0  # dal const.xacro [file:44]

        self.q = np.zeros(self.n_joints)
        self.dq = np.zeros(self.n_joints)
        self.q_d = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.torque_pub = self.create_publisher(
            Float64MultiArray,
            '/torque_controller/commands',
            10
        )
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            '/z1/joint_targets',
            self.target_callback,
            10
        )

        self.control_dt = 0.001  # 1000 Hz come controller_manager
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info('Z1 PD effort node started')

    def joint_state_callback(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        for i, jn in enumerate(self.joint_names):
            if jn in name_to_idx:
                idx = name_to_idx[jn]
                self.q[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.dq[i] = msg.velocity[idx]

        if self.q_d is None:
            self.q_d = self.q.copy()
            self.get_logger().info(f'Initial q_d set to: {self.q_d}')

    def control_loop(self):
        if self.q_d is None:
            return

        e = self.q_d - self.q
        e_dot = -self.dq

        tau = self.Kp * e + self.Kd * e_dot
        tau = np.clip(tau, -self.torque_max, self.torque_max)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.torque_pub.publish(msg)

    def target_callback(self, msg: Float64MultiArray):
        if len(msg.data) == self.n_joints:
            self.q_d = np.array(msg.data)
            self.get_logger().info(f'New target: {self.q_d}')
        else:
            self.get_logger().warn(f'Invalid target size: {len(msg.data)}')



def main(args=None):
    rclpy.init(args=args)
    node = Z1PDEffortNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
