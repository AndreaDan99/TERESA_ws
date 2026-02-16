#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import time


class ZOffsetCommander(Node):
    def __init__(self, z_offset, duration):
        super().__init__('z_offset_commander')
        self.z_offset = z_offset  # in metri
        self.duration = duration
        
        self.current_pose = None
        self.target_pose = None
        self.errors = []
        self.start_time = None
        
        self.sub = self.create_subscription(
            PoseStamped, '/current_ee_pose', self.current_pose_callback, 10
        )
        self.pub = self.create_publisher(PoseStamped, '/desired_pose', 10)
        self.log_timer = None
    
    def current_pose_callback(self, msg):
        self.current_pose = msg
    
    def send_offset_target(self):
        self.get_logger().info('⏳ Attendo posa corrente EE...')
        
        timeout = 3.0
        start = time.time()
        while self.current_pose is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            self.get_logger().error('❌ Timeout: nessuna posa ricevuta')
            return False
        
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.current_pose.header.frame_id
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = self.current_pose.pose.position.z + self.z_offset
        self.target_pose.pose.orientation = self.current_pose.pose.orientation
        
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'✅ TARGET INVIATO con offset Z = {self.z_offset*1000:+.1f} mm\n'
            f'{"="*70}\n'
            f'  Posizione corrente: [{self.current_pose.pose.position.x:.4f}, '
            f'{self.current_pose.pose.position.y:.4f}, {self.current_pose.pose.position.z:.4f}]\n'
            f'  Posizione target:   [{self.target_pose.pose.position.x:.4f}, '
            f'{self.target_pose.pose.position.y:.4f}, {self.target_pose.pose.position.z:.4f}]\n'
            f'{"="*70}\n'
        )
        
        self.pub.publish(self.target_pose)
        self.start_time = time.time()
        self.log_timer = self.create_timer(0.5, self.log_status)
        return True
    
    def log_status(self):
        if self.current_pose is None or self.target_pose is None:
            return
        
        x_curr = self.current_pose.pose.position.x
        y_curr = self.current_pose.pose.position.y
        z_curr = self.current_pose.pose.position.z
        
        x_targ = self.target_pose.pose.position.x
        y_targ = self.target_pose.pose.position.y
        z_targ = self.target_pose.pose.position.z
        
        err_x = (x_targ - x_curr) * 1000
        err_y = (y_targ - y_curr) * 1000
        err_z = (z_targ - z_curr) * 1000
        err_tot = (err_x**2 + err_y**2 + err_z**2)**0.5
        
        # ✅ K_p corretti (dal controller)
        K_x, K_y, K_z = 250.0, 250.0, 300.0
        f_x_est = K_x * (x_targ - x_curr)
        f_y_est = K_y * (y_targ - y_curr)
        f_z_est = K_z * (z_targ - z_curr)
        
        # Salva per statistiche
        self.errors.append(err_tot)
        
        # Indicatore convergenza
        if err_tot < 2.0:
            status = '✅ CONVERGED'
        elif err_tot < 5.0:
            status = '🟡 CONVERGING'
        else:
            status = '🔴 MOVING'
        
        self.get_logger().info(
            f'\n┌─────────────────────────────────────────────────────────────┐\n'
            f'│ STATUS: {status:50s} │\n'
            f'│ CORRENTE:  X={x_curr:+7.4f} m  Y={y_curr:+7.4f} m  Z={z_curr:+7.4f} m │\n'
            f'│ TARGET:    X={x_targ:+7.4f} m  Y={y_targ:+7.4f} m  Z={z_targ:+7.4f} m │\n'
            f'├─────────────────────────────────────────────────────────────┤\n'
            f'│ ERRORE:    X={err_x:+7.2f} mm  Y={err_y:+7.2f} mm  Z={err_z:+7.2f} mm │\n'
            f'│ ERRORE TOTALE: {err_tot:6.2f} mm                                  │\n'
            f'├─────────────────────────────────────────────────────────────┤\n'
            f'│ FORZA (stima): X={f_x_est:+6.1f} N  Y={f_y_est:+6.1f} N  Z={f_z_est:+6.1f} N │\n'
            f'└─────────────────────────────────────────────────────────────┘'
        )
    
    def print_statistics(self):
        if not self.errors or self.start_time is None:
            return
        
        elapsed = time.time() - self.start_time
        avg_error = sum(self.errors) / len(self.errors)
        max_error = max(self.errors)
        final_error = self.errors[-1]
        
        # Tempo di settling (errore < 2mm stabilmente)
        settling_time = None
        for i, err in enumerate(self.errors):
            if err < 2.0 and all(e < 2.0 for e in self.errors[i:]):
                settling_time = i * 0.5  # 0.5s per sample
                break
        
        settling_str = f'{settling_time:.2f} s' if settling_time else f'> {elapsed:.2f} s (non convergente)'
        
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'STATISTICHE MOVIMENTO (Z_offset = {self.z_offset*1000:+.1f} mm)\n'
            f'{"="*70}\n'
            f'  Durata:          {elapsed:.2f} s\n'
            f'  Errore medio:    {avg_error:.2f} mm\n'
            f'  Errore massimo:  {max_error:.2f} mm\n'
            f'  Errore finale:   {final_error:.2f} mm\n'
            f'  Settling time:   {settling_str}\n'
            f'{"="*70}\n'
        )


def main():
    rclpy.init()
    
    if len(sys.argv) < 2:
        print("\n" + "="*70)
        print("USAGE:")
        print("  python3 set_z_offset_target.py <offset_mm> [duration_sec]")
        print("\nEXAMPLES:")
        print("  python3 set_z_offset_target.py -5.0        # offset -5mm, monitor 10s")
        print("  python3 set_z_offset_target.py +5.0 20     # offset +5mm, monitor 20s")
        print("="*70 + "\n")
        sys.exit(1)
    
    z_offset_mm = float(sys.argv[1])
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    
    node = ZOffsetCommander(z_offset=z_offset_mm / 1000.0, duration=duration)
    
    try:
        if not node.send_offset_target():
            return
        
        print(f"\n📊 Monitoraggio per {duration:.1f} secondi...")
        print("   (premi Ctrl+C per fermare)\n")
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        print("\n✅ Monitoraggio completato!\n")
        node.print_statistics()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrotto dall'utente\n")
        node.print_statistics()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
