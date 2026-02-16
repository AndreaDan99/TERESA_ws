#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import pinocchio as pin

class Z1PDGravityNode(Node):
    def __init__(self):
        super().__init__('z1_pd_gravity_node')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.n_joints = len(self.joint_names)

        # Dichiara e leggi parametri da YAML
        self.declare_parameter('Kp', [40.0, 40.0, 35.0, 20.0, 8.0, 5.0])
        self.declare_parameter('Kd', [2.5, 4.5, 3.0, 1.5, 0.5, 0.3])
        self.declare_parameter('Ki', [0.0, 1.0, 0.8, 0.3, 0.15, 0.08])
        self.declare_parameter('torque_max', 10.0)
        self.declare_parameter('integral_deadzone', 0.05)
        self.declare_parameter('integral_limit', 0.1)
        self.declare_parameter('control_frequency', 100.0)
        
        # Carica parametri
        self.Kp = np.array(self.get_parameter('Kp').value)
        self.Kd = np.array(self.get_parameter('Kd').value)
        self.Ki = np.array(self.get_parameter('Ki').value)
        self.torque_max = self.get_parameter('torque_max').value
        self.integral_deadzone = self.get_parameter('integral_deadzone').value
        self.integral_limit = self.get_parameter('integral_limit').value
        
        # Log parametri caricati
        self.get_logger().info(f'PID gains loaded: Kp={self.Kp}, Kd={self.Kd}, Ki={self.Ki}')

        # Stato corrente
        self.q = np.zeros(self.n_joints)
        self.dq = np.zeros(self.n_joints)
        self.q_d = None
        
        # Integrale dell'errore
        self.e_integral = np.zeros(self.n_joints)

        # Carica modello Pinocchio
        self.model = None
        self.data = None
        self.load_pinocchio_model()

        # Subscriber e publisher
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.target_sub = self.create_subscription(
            Float64MultiArray, '/z1/joint_targets', self.target_callback, 10
        )
        self.torque_pub = self.create_publisher(
            Float64MultiArray, '/torque_controller/commands', 10
        )

        # Control loop
        control_freq = self.get_parameter('control_frequency').value
        self.control_dt = 1.0 / control_freq
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(f'Z1 PID + Gravity node started @ {control_freq} Hz')

    def load_pinocchio_model(self):
        """Carica il modello URDF con Pinocchio"""
        try:
            # Aspetta che robot_description sia disponibile
            import time
            max_wait = 5.0
            start = time.time()
            
            robot_description = ''
            while (time.time() - start) < max_wait:
                try:
                    robot_description = self.declare_parameter(
                        'robot_description', ''
                    ).get_parameter_value().string_value
                    
                    if robot_description:
                        break
                except:
                    pass
                time.sleep(0.1)
            
            if not robot_description:
                self.get_logger().warn('robot_description not found, trying alternative method')
                robot_description = self.get_robot_description_from_param_server()

            if robot_description:
                # Carica modello Pinocchio da stringa XML
                self.model = pin.buildModelFromXML(robot_description)
                self.data = self.model.createData()
                self.get_logger().info(f'Pinocchio model loaded: {self.model.nq} DOF')
            else:
                raise Exception("Could not load robot_description")
                
        except Exception as e:
            self.get_logger().error(f'Failed to load Pinocchio model: {e}')
            self.get_logger().warn('Running WITHOUT gravity compensation')

    def get_robot_description_from_param_server(self):
        """Leggi robot_description da robot_state_publisher"""
        try:
            import subprocess
            result = subprocess.run(
                ['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description'],
                capture_output=True, text=True, timeout=3.0
            )
            if result.returncode == 0 and 'String value is:' in result.stdout:
                lines = result.stdout.split('\n')
                urdf_lines = [l for l in lines if not l.startswith('String value')]
                return '\n'.join(urdf_lines).strip()
        except Exception as e:
            self.get_logger().warn(f'Fallback method failed: {e}')
        
        return ''

    def compute_gravity(self, q):
        """Calcola vettore di gravità g(q) usando Pinocchio RNEA"""
        if self.model is None or self.data is None:
            return np.zeros(self.n_joints)
        
        try:
            # Costruisci vettore completo (considera solo i primi 6 giunti attivi)
            q_full = np.zeros(self.model.nq)
            q_full[:self.n_joints] = q
            
            v_zero = np.zeros(self.model.nv)
            a_zero = np.zeros(self.model.nv)
            
            # RNEA: Recursive Newton-Euler Algorithm
            # Con v=0, a=0 restituisce solo gravità + Coriolis (che è 0)
            tau_g = pin.rnea(self.model, self.data, q_full, v_zero, a_zero)
            
            return tau_g[:self.n_joints]
            
        except Exception as e:
            self.get_logger().error(f'Gravity computation error: {e}')
            return np.zeros(self.n_joints)

    def joint_state_callback(self, msg: JointState):
        """Aggiorna stato corrente dai sensori"""
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        for i, jn in enumerate(self.joint_names):
            if jn in name_to_idx:
                idx = name_to_idx[jn]
                self.q[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.dq[i] = msg.velocity[idx]

        # Inizializza target alla posa corrente
        if self.q_d is None:
            self.q_d = self.q.copy()
            self.get_logger().info(f'Initial q_d: {np.round(self.q_d, 3)}')

    def target_callback(self, msg: Float64MultiArray):
        """Riceve nuovo target e resetta integrale"""
        if len(msg.data) == self.n_joints:
            self.q_d = np.array(msg.data)
            self.e_integral = np.zeros(self.n_joints)  # Reset integrale a ogni nuovo target
            self.get_logger().info(f'New target: {np.round(self.q_d, 3)}')
        else:
            self.get_logger().warn(f'Invalid target size: {len(msg.data)} (expected {self.n_joints})')

    def control_loop(self):
        """Loop di controllo PID + compensazione gravità"""
        if self.q_d is None:
            return

        # Calcola errore e derivata
        e = self.q_d - self.q
        e_dot = -self.dq
        
        # Accumula integrale SOLO se errore è piccolo (dead-zone)
        e_for_integral = np.where(np.abs(e) < self.integral_deadzone, e, 0.0)
        self.e_integral += e_for_integral * self.control_dt
        self.e_integral = np.clip(self.e_integral, -self.integral_limit, self.integral_limit)
        
        # Legge di controllo PID
        tau_pid = self.Kp * e + self.Kd * e_dot + self.Ki * self.e_integral
        
        # Compensazione gravità
        tau_g = self.compute_gravity(self.q)
        
        # Somma e limita
        tau = tau_pid + tau_g
        tau = np.clip(tau, -self.torque_max, self.torque_max)

        # Log per debug
        self.get_logger().info(
            f'j2: q={self.q[1]:.3f}, q_d={self.q_d[1]:.3f}, '
            f'e={e[1]:.3f}, I={self.e_integral[1]:.3f}, '
            f'tau_p={self.Kp[1]*e[1]:.2f}, tau_i={self.Ki[1]*self.e_integral[1]:.2f}, '
            f'tau_d={self.Kd[1]*e_dot[1]:.2f}, tau_g={tau_g[1]:.2f}, tau={tau[1]:.2f}',
            throttle_duration_sec=0.5
        )

        # Pubblica comandi di coppia
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.torque_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Z1PDGravityNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
