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

Espone i servizi:
- /safe_switch/to_torque  (std_srvs/Trigger)
- /safe_switch/to_jtc     (std_srvs/Trigger)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import numpy as np

from std_srvs.srv import Trigger
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

        # Callback group reentrant: i service handler possono bloccare senza
        # impedire al nodo di ricevere altri topic (es. joint_states)
        self.cb_group = ReentrantCallbackGroup()

        # Subscriber joint states (sempre aggiornato)
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

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Client switch / list controller
        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        self.list_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        # Servizi esposti verso la FSM
        self.srv_to_torque = self.create_service(
            Trigger,
            '/safe_switch/to_torque',
            self.handle_to_torque,
            callback_group=self.cb_group
        )
        self.srv_to_jtc = self.create_service(
            Trigger,
            '/safe_switch/to_jtc',
            self.handle_to_jtc,
            callback_group=self.cb_group
        )

        self.get_logger().info('✅ SafeControllerSwitch pronto.')
        self.get_logger().info('   /safe_switch/to_torque')
        self.get_logger().info('   /safe_switch/to_jtc')

    # ──────────────────────────────────────────────────────────────
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_q = list(msg.position[:6])

    # ──────────────────────────────────────────────────────────────
    # Service handlers
    # ──────────────────────────────────────────────────────────────
    def handle_to_torque(self, request, response):
        self.get_logger().info('🔄 Richiesta switch JTC → torque_controller')
        success = self._execute_switch('to_torque')
        response.success = success
        response.message = 'OK' if success else 'FAILED'
        return response

    def handle_to_jtc(self, request, response):
        self.get_logger().info('🔄 Richiesta switch torque_controller → JTC')
        success = self._execute_switch('to_jtc')
        response.success = success
        response.message = 'OK' if success else 'FAILED'
        return response

    # ──────────────────────────────────────────────────────────────
    # Sequenza di switch
    # ──────────────────────────────────────────────────────────────
    def _execute_switch(self, direction: str) -> bool:
        # 1. Aspetta joint_states (max 5s)
        start = time.time()
        while self.current_q is None:
            time.sleep(0.1)
            if time.time() - start > 5.0:
                self.get_logger().error('❌ Timeout: nessun joint_state ricevuto!')
                return False

        # 2. Congela robot sulla posizione attuale
        self.get_logger().info('🔒 Congelo robot sulla posizione attuale...')
        self._hold_current_position()
        time.sleep(0.5)

        # 3. Se torno a JTC, azzera il torque prima dello switch
        if direction == 'to_jtc':
            deadline = time.time() + 0.2
            while time.time() < deadline:
                msg = Float64MultiArray()
                msg.data = [0.0] * 6
                self.torque_pub.publish(msg)
                time.sleep(0.01)

        # 4. Switch
        self.get_logger().info(f'🔄 Eseguo switch: {direction}')
        success = self._do_switch(direction)

        # 5. Dopo switch a torque: pubblica zero per 1s (impedance safe startup)
        if success and direction == 'to_torque':
            self.get_logger().info('⏳ Hold zero-torque durante avvio impedance (1s)...')
            deadline = time.time() + 1.0
            while time.time() < deadline:
                msg = Float64MultiArray()
                msg.data = [0.0] * 6
                self.torque_pub.publish(msg)
                time.sleep(0.01)

        return success

    # ──────────────────────────────────────────────────────────────
    def _hold_current_position(self):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self.current_q
        pt.velocities = [0.0] * 6
        pt.accelerations = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)

        traj.points = [pt]
        self.traj_pub.publish(traj)
        self.get_logger().info('📌 Goal "hold" pubblicato sul trajectory controller')

    def _get_controllers_state(self, timeout=2.0):
        if not self.list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ Servizio list_controllers non disponibile!')
            return None

        future = self.list_client.call_async(ListControllers.Request())
        start = time.time()
        while not future.done():
            time.sleep(0.05)
            if time.time() - start > timeout:
                self.get_logger().error('❌ Timeout list_controllers!')
                return None

        res = future.result()
        if res is None:
            return None

        return {c.name: c.state for c in res.controller}

    def _wait_expected_states(self, expected: dict, timeout=3.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            states = self._get_controllers_state()
            if states is None:
                time.sleep(0.1)
                continue

            if all(states.get(name) == st for name, st in expected.items()):
                return True

            time.sleep(0.05)

        self.get_logger().error(
            f'❌ Stati controller non raggiunti entro {timeout:.1f}s: atteso={expected}'
        )
        return False

    def _do_switch(self, direction: str) -> bool:
        if not self.switch_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Servizio switch_controller non disponibile!')
            return False

        req = SwitchController.Request()

        if direction == 'to_torque':
            req.activate_controllers   = ['torque_controller']
            req.deactivate_controllers = ['joint_trajectory_controller']
            expected = {
                'torque_controller': 'active',
                'joint_trajectory_controller': 'inactive'
            }
        elif direction == 'to_jtc':
            req.activate_controllers   = ['joint_trajectory_controller']
            req.deactivate_controllers = ['torque_controller']
            expected = {
                'joint_trajectory_controller': 'active',
                'torque_controller': 'inactive'
            }
        else:
            self.get_logger().error(f'❌ direction non valida: {direction}')
            return False

        req.strictness = 1
        req.activate_asap = True

        future = self.switch_client.call_async(req)
        start = time.time()
        while not future.done():
            time.sleep(0.05)
            if time.time() - start > 3.0:
                self.get_logger().error('❌ Timeout switch controller!')
                return False

        if not future.result().ok:
            self.get_logger().error('❌ Switch fallito (result.ok = false)')
            return False

        self.get_logger().info('✅ Switch riuscito! Verifico stati...')
        return self._wait_expected_states(expected, timeout=3.0)


# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = SafeControllerSwitch()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
