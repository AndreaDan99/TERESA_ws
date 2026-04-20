#!/usr/bin/env python3
# leggi_posizione_fk.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
from ament_index_python.packages import get_package_share_directory

URDF_PATH = os.path.join(get_package_share_directory('z1_description'), 'urdf', 'z1.urdf')
EE_FRAME  = 'link06'

class FKReader(Node):
    def __init__(self):
        super().__init__('fk_reader')
        self.model = pin.buildModelFromUrdf(URDF_PATH)
        self.data  = self.model.createData()
        self.ee_id = self.model.getFrameId(EE_FRAME)

        self.create_subscription(
            JointState, '/joint_states', self.cb, 10)
        self.get_logger().info('In attesa di joint_states...')

    def cb(self, msg):
        q = np.zeros(self.model.nq)
        q[:6] = np.array(msg.position[:6])

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        pos = self.data.oMf[self.ee_id].translation
        # Orientation (rotation matrix -> quaternion)
        rot = self.data.oMf[self.ee_id].rotation
        quat = pin.Quaternion(rot).coeffs()  # [x, y, z, w]

        self.get_logger().info(
            f'📍 EE pose (world frame):\n'
            f'   Position:\n'
            f'      x: {pos[0]:.4f}\n'
            f'      y: {pos[1]:.4f}\n'
            f'      z: {pos[2]:.4f}\n'
            f'   Orientation (quaternion):\n'
            f'      x: {quat[0]:.4f}\n'
            f'      y: {quat[1]:.4f}\n'
            f'      z: {quat[2]:.4f}\n'
            f'      w: {quat[3]:.4f}\n'
            f'\n'
            f'   👉 home_position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]\n'
            f'   👉 home_orientation: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]'
        )

rclpy.init()
rclpy.spin(FKReader())
