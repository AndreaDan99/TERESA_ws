#!/usr/bin/env python3
"""
Safe Controller Switch

Switches safely between:
- joint_trajectory_controller → torque_controller
- torque_controller → joint_trajectory_controller

Procedure:
1. Read current joint position
2. Send hold command to the controller that must remain active
3. Wait stabilization
4. Perform controller switch
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers


class SafeControllerSwitch(Node):

    JOINT_NAMES = [
        'joint1', 'joint2', 'joint3',
        'joint4', 'joint5', 'joint6'
    ]

    def __init__(self):
        super().__init__('safe_controller_switch')

        self.current_q = None

        # Switch direction parameter
        self.declare_parameter('switch_direction', 'to_torque')
        self.switch_direction = self.get_parameter('switch_direction').value


        # Subscriber joint states
        self.js_sub = self.create_subscription(
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

        # Client list controllers (per verifica stato)
        self.list_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
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
        self.get_logger().info(f'🔄 Eseguo switch: {self.switch_direction}')
        success = self._do_switch()

        # Se sto tornando a JTC, azzera torque per un breve tempo prima dello switch (riduce scatti)
        if self.switch_direction == 'to_jtc':
            start = time.time()
            while time.time() - start < 0.2:
                msg = Float64MultiArray()
                msg.data = [0.0] * 6
                self.torque_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)

        if success and self.switch_direction == 'to_torque':
            self.get_logger().info('⏳ Hold durante avvio impedance...')
            start = time.time()
            while time.time() - start < 1.0:   # 1 secondo di hold
                msg = Float64MultiArray()
                msg.data = [0.0] * 6
                self.torque_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)

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

    def _get_controllers_state(self):
        """Ritorna dict {name: state} per i controller noti."""
        if not self.list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ Servizio list_controllers non disponibile!')
            return None

        req = ListControllers.Request()
        future = self.list_client.call_async(req)
        start = time.time()
        timeout = 2.0
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('❌ Timeout list_controllers!')
                return None

        res = future.result()
        if res is None:
            return None

        out = {}
        for c in res.controller:
            out[c.name] = c.state
        return out

    def _wait_expected_states(self, expected: dict, timeout: float = 3.0):
        """Attende che i controller raggiungano gli stati attesi (es: {'torque_controller':'active'})."""
        start = time.time()
        while time.time() - start < timeout:
            states = self._get_controllers_state()
            if states is None:
                time.sleep(0.1)
                continue

            ok = True
            for name, st in expected.items():
                if states.get(name) != st:
                    ok = False
                    break

            if ok:
                return True

            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.get_logger().error(f"❌ Stati controller non raggiunti entro {timeout:.1f}s: atteso={expected}")
        self.get_logger().error(f"   stati attuali={states}")
        return False

    def _do_switch(self):
        if not self.switch_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Servizio switch_controller non disponibile!')
            return False

        req = SwitchController.Request()

        if self.switch_direction == 'to_torque':
            req.activate_controllers   = ['torque_controller']
            req.deactivate_controllers = ['joint_trajectory_controller']

        elif self.switch_direction == 'to_jtc':
            req.activate_controllers   = ['joint_trajectory_controller']
            req.deactivate_controllers = ['torque_controller']

        else:
            self.get_logger().error(f'❌ switch_direction non valido: {self.switch_direction}')
            return False

        req.strictness = 1   # BEST_EFFORT
        req.activate_asap = True

        future = self.switch_client.call_async(req)

        timeout = 3.0
        start = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('❌ Timeout switch controller!')
                return False

        result = future.result()
        if result.ok:
            self.get_logger().info('✅ Switch riuscito!')
        else:
            self.get_logger().error('❌ Switch fallito (result.ok = false)')
            return False

        # Verifica stato attivo/disattivo
        if self.switch_direction == 'to_torque':
            expected = {
                'torque_controller': 'active',
                'joint_trajectory_controller': 'inactive'
            }
        else:
            expected = {
                'joint_trajectory_controller': 'active',
                'torque_controller': 'inactive'
            }

        if not self._wait_expected_states(expected, timeout=3.0):
            return False

        return True


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
