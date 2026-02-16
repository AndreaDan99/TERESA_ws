#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import csv
from datetime import datetime

class TuningLogger(Node):
    def __init__(self):
        super().__init__('tuning_logger')
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.q = np.zeros(6)
        self.dq = np.zeros(6)
        self.tau = np.zeros(6)
        self.q_target = np.zeros(6)
        
        self.log_data = []
        self.logging = False
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.effort_sub = self.create_subscription(
            Float64MultiArray, '/torque_controller/commands', self.effort_callback, 10
        )
        self.target_sub = self.create_subscription(
            Float64MultiArray, '/z1/joint_targets', self.target_callback, 10
        )
        
        self.timer = self.create_timer(0.01, self.log_callback)  # 100 Hz
        
        self.get_logger().info('Tuning logger started. Waiting for new target...')

    def joint_callback(self, msg):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        for i, jn in enumerate(self.joint_names):
            if jn in name_to_idx:
                idx = name_to_idx[jn]
                self.q[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.dq[i] = msg.velocity[idx]

    def effort_callback(self, msg):
        self.tau = np.array(msg.data[:6])

    def target_callback(self, msg):
        self.q_target = np.array(msg.data[:6])
        self.get_logger().info(f'New target received. Starting logging for 5 seconds...')
        self.logging = True
        self.log_data = []
        self.start_time = self.get_clock().now()
        
        # Stop dopo 5 secondi (corretto)
        def stop_callback():
            self.stop_logging()
        
        self.create_timer(5.0, stop_callback)


    def log_callback(self):
        if not self.logging:
            return
        
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        error = self.q_target - self.q
        
        self.log_data.append([
            t,
            *self.q.tolist(),
            *self.dq.tolist(),
            *self.tau.tolist(),
            *error.tolist()
        ])

    def stop_logging(self):
        self.logging = False
        self.save_log()

    def save_log(self):
        if not self.log_data:
            return
        
        filename = f'/tmp/z1_tuning_{datetime.now().strftime("%H%M%S")}.csv'
        
        header = ['time']
        header += [f'q{i}' for i in range(1, 7)]
        header += [f'dq{i}' for i in range(1, 7)]
        header += [f'tau{i}' for i in range(1, 7)]
        header += [f'error{i}' for i in range(1, 7)]
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.log_data)
        
        self.get_logger().info(f'Log saved to {filename}')
        self.print_metrics()

    def print_metrics(self):
        """Calcola metriche di performance"""
        if len(self.log_data) < 10:
            return
        
        data = np.array(self.log_data)
        errors = data[:, 19:25]  # colonne error
        
        self.get_logger().info('=== TUNING METRICS ===')
        for i, jname in enumerate(self.joint_names):
            err = errors[:, i]
            
            # Max overshoot
            max_err = np.max(np.abs(err))
            
            # Settling time (quando errore < 0.01 rad)
            settled_idx = np.where(np.abs(err) < 0.02)[0]
            settling_time = data[settled_idx[0], 0] if len(settled_idx) > 0 else np.nan
            
            # Oscillazioni (zero crossings)
            zero_cross = np.sum(np.diff(np.sign(err)) != 0)
            
            self.get_logger().info(
                f'{jname}: max_err={max_err:.4f} rad, '
                f'settling={settling_time:.2f}s, oscillations={zero_cross}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = TuningLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
