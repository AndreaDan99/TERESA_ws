#!/usr/bin/env python3
"""
Test avanzato per Impedance Controller
Esegue movimenti complessi VISIBILI in spazio limitato
CON ATTESA CONVERGENZA per ogni waypoint
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import numpy as np


class ImpedanceComplexTest(Node):
    def __init__(self):
        super().__init__('impedance_complex_test')
        
        # Publisher per target pose
        self.pub = self.create_publisher(PoseStamped, '/desired_pose', 10)
        
        # Subscriber per posizione corrente
        self.sub = self.create_subscription(
            PoseStamped, '/current_ee_pose',
            self.pose_callback, 10
        )
        
        # Stato
        self.current_pose = None
        self.pose_updated = False
        self.initial_position = None
        
        # Configurazione - OTTIMIZZATO per convergenza
        self.convergence_tolerance_mm = 8.0  # Tolleranza convergenza (mm)
        self.max_wait_per_waypoint = 5.0     # Max tempo attesa per waypoint (s)
        
        self.get_logger().info('='*70)
        self.get_logger().info('IMPEDANCE CONTROL - ADVANCED TEST (WITH CONVERGENCE)')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Tolleranza convergenza: {self.convergence_tolerance_mm}mm')
        self.get_logger().info(f'Max attesa per waypoint: {self.max_wait_per_waypoint}s')
        self.get_logger().info('Attendo posizione corrente...')
    
    def pose_callback(self, msg):
        """Callback per aggiornare posizione corrente"""
        self.current_pose = msg
        self.pose_updated = True
    
    def wait_for_pose(self, timeout=5.0):
        """Aspetta che la posizione sia disponibile"""
        start = time.time()
        while not self.pose_updated and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.pose_updated:
            self.get_logger().error('❌ Timeout: posizione non ricevuta!')
            return False
        
        self.pose_updated = False
        return True
    
    def send_absolute_target(self, position_xyz):
        """Invia target assoluto mantenendo orientamento corrente"""
        if self.current_pose is None:
            return False
        
        target = PoseStamped()
        target.header.frame_id = 'world'
        target.header.stamp = self.get_clock().now().to_msg()
        
        target.pose.position.x = position_xyz[0]
        target.pose.position.y = position_xyz[1]
        target.pose.position.z = position_xyz[2]
        
        # Mantieni orientamento
        target.pose.orientation = self.current_pose.pose.orientation
        
        self.pub.publish(target)
        return True
    
    def wait_for_convergence(self, target_pos, tolerance_mm=None, timeout=None):
        """
        Aspetta che il robot raggiunga il target entro tolleranza
        Returns: (converged, final_error_mm, time_elapsed)
        """
        if tolerance_mm is None:
            tolerance_mm = self.convergence_tolerance_mm
        if timeout is None:
            timeout = self.max_wait_per_waypoint
        
        start_time = time.time()
        min_error = float('inf')
        samples_below_tolerance = 0
        required_samples = 3  # Deve rimanere sotto tolleranza per 3 campioni (0.15s)
        
        while (time.time() - start_time) < timeout:
            if self.current_pose is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            
            curr = np.array([
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ])
            
            error_mm = np.linalg.norm(curr - target_pos) * 1000
            min_error = min(min_error, error_mm)
            
            # Conta campioni consecutivi sotto tolleranza
            if error_mm < tolerance_mm:
                samples_below_tolerance += 1
                if samples_below_tolerance >= required_samples:
                    elapsed = time.time() - start_time
                    return True, error_mm, elapsed
            else:
                samples_below_tolerance = 0  # Reset se esce dalla tolleranza
            
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)  # 20Hz check rate
        
        # Timeout raggiunto
        elapsed = time.time() - start_time
        return False, min_error, elapsed
    
    def execute_trajectory(self, name, waypoints, show_progress=True):
        """
        Esegue traiettoria definita da waypoints
        waypoints: lista di np.array([x, y, z]) relativi a initial_position
        show_progress: mostra posizione corrente durante movimento
        """
        self.get_logger().info('')
        self.get_logger().info(f'🔷 {name}')
        self.get_logger().info('-'*70)
        self.get_logger().info(f'   Waypoints: {len(waypoints)}')
        
        start_time = time.time()
        total_error = 0.0
        max_error = 0.0
        converged_count = 0
        timeout_count = 0
        
        for i, waypoint in enumerate(waypoints):
            # Calcola posizione assoluta
            target_pos = self.initial_position + waypoint
            
            # Log waypoint
            if show_progress:
                self.get_logger().info(
                    f'   → Waypoint {i+1}/{len(waypoints)}: '
                    f'[{waypoint[0]*1000:+6.1f}, {waypoint[1]*1000:+6.1f}, {waypoint[2]*1000:+6.1f}] mm'
                )
            
            # Invia target
            self.send_absolute_target(target_pos)
            
            # Attendi convergenza
            converged, final_error, elapsed = self.wait_for_convergence(target_pos)
            
            total_error += final_error
            max_error = max(max_error, final_error)
            
            if converged:
                converged_count += 1
                if show_progress:
                    self.get_logger().info(
                        f'      ✅ Raggiunto in {elapsed:.2f}s (errore: {final_error:.1f}mm)'
                    )
            else:
                timeout_count += 1
                if show_progress:
                    self.get_logger().warn(
                        f'      ⚠️ Timeout {elapsed:.2f}s (errore minimo: {final_error:.1f}mm)'
                    )
        
        total_elapsed = time.time() - start_time
        avg_error = total_error / len(waypoints) if waypoints else 0
        
        # Summary
        self.get_logger().info('')
        self.get_logger().info(f'   📊 RISULTATI:')
        self.get_logger().info(f'      Tempo totale: {total_elapsed:.1f}s')
        self.get_logger().info(f'      Converged: {converged_count}/{len(waypoints)}')
        self.get_logger().info(f'      Timeout: {timeout_count}/{len(waypoints)}')
        self.get_logger().info(f'      Errore medio: {avg_error:.1f}mm')
        self.get_logger().info(f'      Errore max: {max_error:.1f}mm')
        
        return {
            'converged': converged_count,
            'timeout': timeout_count,
            'avg_error': avg_error,
            'max_error': max_error,
            'total_time': total_elapsed
        }
    
    def generate_circle_xy(self, radius=0.03, num_points=16):
        """Cerchio sul piano XY"""
        angles = np.linspace(0, 2*np.pi, num_points)
        waypoints = []
        for angle in angles:
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            waypoints.append(np.array([x, y, 0.0]))
        return waypoints
    
    def generate_circle_xz(self, radius=0.03, num_points=16):
        """Cerchio sul piano XZ"""
        angles = np.linspace(0, 2*np.pi, num_points)
        waypoints = []
        for angle in angles:
            x = radius * np.cos(angle)
            z = radius * np.sin(angle)
            waypoints.append(np.array([x, 0.0, z]))
        return waypoints
    
    def generate_spiral_3d(self, radius=0.04, height=0.05, num_turns=1.5, num_points=24):
        """Spirale 3D"""
        t = np.linspace(0, num_turns * 2 * np.pi, num_points)
        waypoints = []
        for i, angle in enumerate(t):
            # Raggio decresce verso il centro
            r = radius * (1 - i / len(t))
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = height * i / len(t)
            waypoints.append(np.array([x, y, z]))
        
        # Ritorno al centro lentamente
        waypoints.append(np.array([0.0, 0.0, height]))
        waypoints.append(np.array([0.0, 0.0, 0.0]))
        return waypoints
    
    def generate_star_pattern(self, radius=0.04, num_rays=6):
        """Pattern a stella"""
        angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        waypoints = []
        
        for angle in angles:
            # Vai verso esterno
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            waypoints.append(np.array([x, y, 0.0]))
            
            # Torna al centro
            waypoints.append(np.array([0.0, 0.0, 0.0]))
        
        return waypoints
    
    def generate_figure_eight(self, size=0.035, num_points=32):
        """Figura a otto (lemniscata)"""
        t = np.linspace(0, 2*np.pi, num_points)
        waypoints = []
        for angle in t:
            # Parametric equation for lemniscate
            scale = size / (1 + np.sin(angle)**2)
            x = scale * np.cos(angle)
            y = scale * np.sin(angle) * np.cos(angle)
            waypoints.append(np.array([x, y, 0.0]))
        return waypoints
    
    def generate_square(self, side=0.05, num_points_per_side=8):
        """Quadrato sul piano XY"""
        waypoints = []
        
        # 4 angoli del quadrato
        corners = [
            np.array([side/2, side/2, 0.0]),    # top-right
            np.array([-side/2, side/2, 0.0]),   # top-left
            np.array([-side/2, -side/2, 0.0]),  # bottom-left
            np.array([side/2, -side/2, 0.0]),   # bottom-right
            np.array([side/2, side/2, 0.0]),    # back to start
        ]
        
        # Interpola tra angoli
        for i in range(len(corners)-1):
            for j in range(num_points_per_side):
                alpha = j / num_points_per_side
                point = corners[i] + alpha * (corners[i+1] - corners[i])
                waypoints.append(point)
        
        return waypoints
    
    def run_test(self):
        """Esegue test completo"""
        
        # Attendi posizione iniziale
        if not self.wait_for_pose():
            return
        
        self.initial_position = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        self.get_logger().info('✅ Posizione iniziale acquisita')
        self.get_logger().info(
            f'   Posizione: [{self.initial_position[0]:.4f}, '
            f'{self.initial_position[1]:.4f}, {self.initial_position[2]:.4f}]'
        )
        self.get_logger().info('='*70)
        
        # Raccolta risultati
        all_results = []
        
        # Test 1: Quadrato XY
        waypoints = self.generate_square(side=0.05, num_points_per_side=8)
        result = self.execute_trajectory('Test 1: Quadrato piano XY (lato 5cm)', waypoints)
        all_results.append(('Quadrato XY', result))
        time.sleep(1.5)
        
        # Test 2: Cerchio XY
        waypoints = self.generate_circle_xy(radius=0.03, num_points=16)
        result = self.execute_trajectory('Test 2: Cerchio piano XY (r=3cm)', waypoints)
        all_results.append(('Cerchio XY', result))
        time.sleep(1.5)
        
        # Test 3: Cerchio XZ (verticale)
        waypoints = self.generate_circle_xz(radius=0.03, num_points=16)
        result = self.execute_trajectory('Test 3: Cerchio piano XZ verticale (r=3cm)', waypoints)
        all_results.append(('Cerchio XZ', result))
        time.sleep(1.5)
        
        # Test 4: Figura a 8
        waypoints = self.generate_figure_eight(size=0.035, num_points=32)
        result = self.execute_trajectory('Test 4: Figura a otto (3.5cm)', waypoints)
        all_results.append(('Figura 8', result))
        time.sleep(1.5)
        
        # Test 5: Spirale 3D
        waypoints = self.generate_spiral_3d(radius=0.04, height=0.05, num_turns=1.5, num_points=24)
        result = self.execute_trajectory('Test 5: Spirale 3D (r=4cm, h=5cm)', waypoints)
        all_results.append(('Spirale 3D', result))
        time.sleep(1.5)
        
        # Test 6: Stella
        waypoints = self.generate_star_pattern(radius=0.04, num_rays=6)
        result = self.execute_trajectory('Test 6: Pattern stella (r=4cm, 6 raggi)', waypoints)
        all_results.append(('Stella', result))
        
        # Test completato - SUMMARY FINALE
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('✅ TUTTI I TEST COMPLETATI - SUMMARY')
        self.get_logger().info('='*70)
        
        total_converged = 0
        total_timeout = 0
        total_waypoints = 0
        
        for name, res in all_results:
            total_converged += res['converged']
            total_timeout += res['timeout']
            total_waypoints += res['converged'] + res['timeout']
            
            self.get_logger().info(
                f'{name:15s} | Conv: {res["converged"]:2d}/{res["converged"]+res["timeout"]:2d} | '
                f'Avg: {res["avg_error"]:5.1f}mm | Max: {res["max_error"]:5.1f}mm | '
                f'Time: {res["total_time"]:5.1f}s'
            )
        
        self.get_logger().info('-'*70)
        self.get_logger().info(
            f'TOTALE: Converged {total_converged}/{total_waypoints} '
            f'({100*total_converged/total_waypoints:.1f}%)'
        )
        
        # Posizione finale vs iniziale
        if self.wait_for_pose():
            final_pos = np.array([
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ])
            total_drift = final_pos - self.initial_position
            drift_magnitude = np.linalg.norm(total_drift)
            
            self.get_logger().info('')
            self.get_logger().info(f'   Drift totale: {drift_magnitude*1000:.1f}mm')
            self.get_logger().info(
                f'   Δ finale: [{total_drift[0]*1000:+.1f}, {total_drift[1]*1000:+.1f}, '
                f'{total_drift[2]*1000:+.1f}] mm'
            )
        
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    
    test_node = ImpedanceComplexTest()
    
    # Attendi connessione
    time.sleep(1.0)
    
    try:
        test_node.run_test()
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        test_node.get_logger().info('❌ Test interrotto!')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
