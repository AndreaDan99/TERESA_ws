#!/usr/bin/env python3
"""
Safe Controller Switch
1. Legge posizione attuale da /joint_states
2. Manda quella posizione come goal statico al trajectory controller
3. Aspetta 500ms che si stabilizzi
4. Fa lo switch trajectory → torque controller
5. Si chiude (il launch avvierà l'impedance controller)
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController


class SafeControllerSwitch(Node):

    JOINT_NAMES = [
        'joint1', 'joint2', 'joint3',
        'joint4', 'joint5', 'joint6'
    ]

    def __init__(self):
        super().__init__('safe_controller_switch')

        self.current_q = None

        # Subscriber joint states
        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher trajectory controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Client switch controller
        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

        self.get_logger().info('⏳ Safe Switch: attendo joint_states...')

    def joint_state_callback(self, msg: JointState):
        if self.current_q is None:
            if len(msg.position) >= 6:
                self.current_q = list(msg.position[:6])
                self.get_logger().info(
                    f'✅ Posizione letta: {[f"{np.rad2deg(q):.1f}°" for q in self.current_q]}'
                )

    def run(self):
        """Sequenza principale - chiamata dopo init"""

        # 1. Aspetta di ricevere joint_states (max 5 secondi)
        timeout = 5.0
        start = time.time()
        while self.current_q is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('❌ Timeout: nessun joint_state ricevuto!')
                return False

        # 2. Congela il robot sulla posizione attuale
        self.get_logger().info('🔒 Congelo robot sulla posizione attuale...')
        self._hold_current_position()

        # 3. Aspetta 500ms che il trajectory controller sia stabile
        time.sleep(0.5)

        # 4. Switch controller
        self.get_logger().info('🔄 Eseguo switch trajectory → torque...')
        success = self._do_switch()

        if success:
            self.get_logger().info('✅ Switch completato! Avvio impedance controller...')
        else:
            self.get_logger().error('❌ Switch fallito!')

        return success

    def _hold_current_position(self):
        """Manda la posizione attuale come goal statico al trajectory controller"""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self.current_q
        pt.velocities = [0.0] * 6
        pt.accelerations = [0.0] * 6
        # Raggiungi questa posizione in 200ms (è già lì, quindi è un hold)
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)

        traj.points = [pt]
        self.traj_pub.publish(traj)
        self.get_logger().info('📌 Goal "hold" pubblicato sul trajectory controller')

    def _do_switch(self):
        """Chiama il servizio switch_controller"""

        # Aspetta che il servizio sia disponibile
        if not self.switch_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Servizio switch_controller non disponibile!')
            return False

        req = SwitchController.Request()
        req.activate_controllers   = ['torque_controller']
        req.deactivate_controllers = ['joint_trajectory_controller']
        req.strictness = 2  # STRICT: fallisce se un controller non esiste
        req.activate_asap = True

        future = self.switch_client.call_async(req)

        # Aspetta risposta (max 3 secondi)
        timeout = 3.0
        start = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('❌ Timeout switch controller!')
                return False

        result = future.result()
        return result.ok


def main():
    rclpy.init()
    node = SafeControllerSwitch()

    try:
        success = node.run()
    except Exception as e:
        node.get_logger().error(f'❌ Errore: {e}')
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Exit code: 0 = successo (il launch avvierà l'impedance controller)
    # Exit code: 1 = fallito
    import sys
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
