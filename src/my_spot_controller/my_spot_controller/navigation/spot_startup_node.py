#!/usr/bin/env python3
"""
Spot Main Node - Gestisce solo inizializzazione e shutdown del robot
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from spot_msgs.msg import BatteryState, BatteryStateArray, PowerState
import time
import signal
import sys
import threading


class ExitCheck:
    """Gestisce l'uscita sicura catturando SIGTERM/SIGINT (thread-safe)"""
    def __init__(self):
        self._kill_now = False
        self._lock = threading.Lock()
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)
    
    def _sigterm_handler(self, _signum, _frame):
        with self._lock:
            self._kill_now = True
    
    def request_exit(self):
        with self._lock:
            self._kill_now = True
    
    @property
    def kill_now(self):
        with self._lock:
            return self._kill_now


class SpotStartupNode(Node):
    """Nodo principale: gestisce Spot (claim/power/stand) e monitoring battery"""
    def __init__(self, robot_name='my_spot'):
        super().__init__('spot_main_node')
        
        self.robot_name = robot_name
        self.exit_check = ExitCheck()
        
        # --- SUBSCRIBERS ---
        self.power_state_sub = self.create_subscription(
            PowerState, f'/{robot_name}/status/power_state',
            self.power_state_callback, 10)
        
        self.battery_sub = self.create_subscription(
            BatteryStateArray, f'/{robot_name}/status/battery_states',
            self.battery_callback, 10)
        
        # --- SERVICE CLIENTS ---
        self.cli_claim = self.create_client(Trigger, f'/{robot_name}/claim')
        self.cli_power_on = self.create_client(Trigger, f'/{robot_name}/power_on')
        self.cli_stand = self.create_client(Trigger, f'/{robot_name}/stand')
        self.cli_sit = self.create_client(Trigger, f'/{robot_name}/sit')
        self.cli_stop = self.create_client(Trigger, f'/{robot_name}/stop')
        self.cli_power_off = self.create_client(Trigger, f'/{robot_name}/power_off')
        
        # --- STATE VARIABLES ---
        self.latest_power_state = None
        self.latest_battery = None
        
        self.get_logger().info(f'✅ Spot Main Node initialized for robot: {robot_name}')
    
    # --- CALLBACKS ---
    def power_state_callback(self, msg):
        self.latest_power_state = msg
    
    def battery_callback(self, msg):
        self.latest_battery = msg
    
    # --- ROBOT CONTROL FUNCTIONS ---
    def call_service_sync(self, client, timeout=5.0, allow_interrupt=True):
        """
        Helper per chiamate service con timeout REALE.
        
        Args:
            client: Service client
            timeout: Timeout in secondi
            allow_interrupt: Se False, ignora Ctrl+C (usato per shutdown)
        
        Returns:
            Service response o None
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {client.srv_name} not available (timeout)')
            return None
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Spin con check periodico per timeout/interrupt
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Service completato
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'Service {client.srv_name} exception: {e}')
                    return None
            
            # Timeout check
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'Service {client.srv_name} timeout after {timeout}s')
                return None
            
            # Exit check (solo se allow_interrupt=True)
            if allow_interrupt and self.exit_check.kill_now:
                self.get_logger().warn(f'Service {client.srv_name} interrupted by user')
                return None
        
        return None
    
    def start(self):
        """Inizializza il robot: claim -> power_on -> stand"""
        self.get_logger().info('=== Starting robot initialization ===')
        
        # 1. Claim robot
        self.get_logger().info('Claiming robot...')
        result = self.call_service_sync(self.cli_claim, timeout=5.0)
        if result is None or not result.success:
            self.get_logger().error(f'Failed to claim robot: {result.message if result else "timeout"}')
            return False
        self.get_logger().info('✅ Robot claimed')
        
        # 2. Power on
        self.get_logger().info('Powering on robot...')
        result = self.call_service_sync(self.cli_power_on, timeout=10.0)
        if result is None or not result.success:
            self.get_logger().error(f'Failed to power on: {result.message if result else "timeout"}')
            return False
        self.get_logger().info('✅ Robot powered on')
        
        # 3. Stand up
        self.get_logger().info('Standing robot up...')
        result = self.call_service_sync(self.cli_stand, timeout=10.0)
        if result is None or not result.success:
            self.get_logger().error(f'Failed to stand: {result.message if result else "timeout"}')
            return False
        self.get_logger().info('✅ Robot standing')
        
        self.get_logger().info('=== Initialization complete ===')
        return True
    
    def shutdown(self):
        """
        Shutdown sicuro: stop -> sit -> power_off.
        Esegue SEMPRE la sequenza completa (NON interrompibile).
        """
        self.get_logger().info('=== Shutting down safely ===')
        
        # Stop movement
        self.get_logger().info('Stopping movement...')
        result = self.call_service_sync(self.cli_stop, timeout=2.0, allow_interrupt=False)
        if result and result.success:
            self.get_logger().info('✅ Movement stopped')
        else:
            self.get_logger().warn('⚠️ Stop command timeout/failed')
        
        # Sit down
        self.get_logger().info('Sitting down...')
        result = self.call_service_sync(self.cli_sit, timeout=8.0, allow_interrupt=False)
        if result and result.success:
            self.get_logger().info('✅ Robot sitting')
        else:
            self.get_logger().warn('⚠️ Sit command timeout/failed')
        
        # Power off
        self.get_logger().info('Powering off...')
        result = self.call_service_sync(self.cli_power_off, timeout=3.0, allow_interrupt=False)
        if result and result.success:
            self.get_logger().info('✅ Robot powered off')
        else:
            self.get_logger().warn('⚠️ Power off timeout/failed')
        
        self.get_logger().info('=== Shutdown complete ===')
    
    def get_battery_str(self):
        """Ritorna lo stato della batteria"""
        if self.latest_battery is None or not self.latest_battery.battery_states:
            return "Battery: Unknown"
        battery = self.latest_battery.battery_states[0]
        return f"Battery: {battery.charge_percentage:.1f}%"
    
    def get_power_str(self):
        """Ritorna lo stato del motore"""
        if self.latest_power_state is None:
            return "Power: Unknown"
        states = {0: "Unknown", 1: "Off", 2: "On", 3: "Powering On", 4: "Powering Off", 5: "Error"}
        return f"Power: {states.get(self.latest_power_state.motor_power_state, 'Invalid')}"


def main(args=None):
    rclpy.init(args=args)
    node = SpotStartupNode(robot_name='my_spot')
    
    try:
        # Startup sequence
        if not node.start():
            node.get_logger().error('❌ Failed to start robot')
            node.shutdown()
            return
        
        # Status timer (ogni 5 secondi)
        def status_timer_callback():
            node.get_logger().info(
                f'{node.get_battery_str()} | {node.get_power_str()}'
            )
        timer = node.create_timer(5.0, status_timer_callback)
        
        # Main loop
        node.get_logger().info('Robot ready - Press Ctrl+C to exit')
        
        while rclpy.ok() and not node.exit_check.kill_now:
            rclpy.spin_once(node, timeout_sec=0.5)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - shutting down')
    except Exception as e:
        node.get_logger().error(f'Exception: {str(e)}')
    finally:
        # Shutdown sequence
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
