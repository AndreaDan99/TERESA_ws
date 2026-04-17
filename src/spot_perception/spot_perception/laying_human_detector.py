#!/usr/bin/env python3
"""
Laying Human Detector - CORRETTO.
Rileva persone sdraiate e calcola approach point (NO navigazione per ora).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float32
import numpy as np
import math


class LayingHumanDetector(Node):
    def __init__(self):
        super().__init__('laying_human_detector')
        
        # ============================================================
        # PARAMETRI
        # ============================================================
        self.declare_parameter('approach_distance', 1.0)  # Metri da bbox (AUMENTATO da 0.8)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_valid_keypoints', 4)
        self.declare_parameter('test_mode', True)  # DEFAULT: True (no navigation)
        self.declare_parameter('detection_timeout', 2.0)  # Secondi senza detection per reset

        self.approach_dist = float(self.get_parameter('approach_distance').value)
        self.min_conf = float(self.get_parameter('min_detection_confidence').value)
        self.min_kp = int(self.get_parameter('min_valid_keypoints').value)
        self.test_mode = bool(self.get_parameter('test_mode').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)

        # Subscribers
        self.skeleton_sub = self.create_subscription(
            PoseArray, '/human_pose/points_3d',
            self.skeleton_callback, 10
        )
        
        self.posture_sub = self.create_subscription(
            String, '/human_pose/posture',
            self.posture_callback, 10
        )
        
        self.posture_conf_sub = self.create_subscription(
            Float32, '/human_pose/posture_confidence',
            self.confidence_callback, 10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/laying_human/approach_point', 10  # RINOMINATO topic
        )
        
        self.approach_marker_pub = self.create_publisher(
            Marker, '/laying_human/approach_marker', 10  # RINOMINATO per chiarezza
        )
        
        # State
        self.current_posture = "UNKNOWN"
        self.current_confidence = 0.0
        self.latest_skeleton = None
        self.goal_sent = False
        self.last_detection_time = None
        
        # Timer per reset detection
        self.reset_timer = self.create_timer(1.0, self.check_detection_timeout)
        
        mode_str = "TEST MODE (no navigation)" if self.test_mode else "ACTIVE MODE (navigation enabled)"
        self.get_logger().info(
            f'✅ Laying Human Detector initialized\n'
            f'   Mode: {mode_str}\n'
            f'   Approach distance: {self.approach_dist}m\n'
            f'   Min keypoints: {self.min_kp}\n'
            f'   Min confidence: {self.min_conf}'
        )
    
    def confidence_callback(self, msg):
        """Aggiorna confidence corrente."""
        self.current_confidence = msg.data
    
    def posture_callback(self, msg):
        """Aggiorna stato postura corrente."""
        prev_posture = self.current_posture
        self.current_posture = msg.data
        
        # Reset flag se passa da LYING a altro stato
        if prev_posture == "LYING" and self.current_posture != "LYING":
            self.goal_sent = False
            self.get_logger().info('Detection LYING persa → flag reset')
    
    def check_detection_timeout(self):
        """Resetta detection se passa troppo tempo senza LYING."""
        if self.last_detection_time is None:
            return
        
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if elapsed > self.detection_timeout:
            if self.goal_sent:
                self.goal_sent = False
                self.get_logger().info(
                    f'Detection timeout ({elapsed:.1f}s) → flag reset'
                )
            self.last_detection_time = None
    
    def skeleton_callback(self, msg):
        """Processa skeleton e rileva persona sdraiata."""
        self.latest_skeleton = msg
        
        # FILTRO 1: Check postura
        if self.current_posture != "LYING":
            return
        
        # FILTRO 2: Check confidence
        if self.current_confidence < self.min_conf:
            self.get_logger().warn(
                f'LYING rilevato ma confidence bassa: {self.current_confidence:.2f} < {self.min_conf}',
                throttle_duration_sec=2.0
            )
            return
        
        # Estrai keypoints validi
        valid_points = []
        for pose in msg.poses:
            p = pose.position
            if not (math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z)):
                valid_points.append([p.x, p.y, p.z])
        
        # FILTRO 3: Check qualità detection
        if len(valid_points) < self.min_kp:
            self.get_logger().warn(
                f'LYING rilevato ma pochi keypoints: {len(valid_points)}/{self.min_kp}',
                throttle_duration_sec=2.0
            )
            return
        
        # Update detection time
        self.last_detection_time = self.get_clock().now()
        
        points = np.array(valid_points)

        # Controlla distanza minima: persona troppo vicina → approach point dietro camera
        mean_depth = np.mean(points[:, 2])
        if mean_depth < self.approach_dist + 0.2:
            self.get_logger().warn(
                f'Persona troppo vicina ({mean_depth:.2f}m < {self.approach_dist + 0.2:.2f}m) '
                f'— approach point non valido, ignorato.',
                throttle_duration_sec=2.0
            )
            return

        # Calcola bounding box 3D
        bbox_min = points.min(axis=0)
        bbox_max = points.max(axis=0)
        bbox_center = (bbox_min + bbox_max) / 2.0
        bbox_size = bbox_max - bbox_min
        
        # Log detection
        self.get_logger().info(
            f'✅ PERSONA SDRAIATA rilevata! '
            f'Keypoints: {len(valid_points)}/17, '
            f'Confidence: {self.current_confidence:.2f}, '
            f'BBox center: ({bbox_center[0]:.2f}, {bbox_center[1]:.2f}, {bbox_center[2]:.2f})',
            throttle_duration_sec=2.0
        )
        
        # Calcola e pubblica approach point
        if not self.test_mode and not self.goal_sent:
            self.publish_approach_point(bbox_center, bbox_size, msg.header)
            self.goal_sent = True
            self.get_logger().info('🎯 Approach point pubblicato (Spot pronto per avvicinarsi)')
        elif self.test_mode:
            # In test mode: pubblica approach point MA non invia goal navigazione
            self.publish_approach_point(bbox_center, bbox_size, msg.header, visualize_only=True)
            
    def publish_approach_point(self, bbox_center, bbox_size, header, visualize_only=False):
        """
        Calcola e pubblica approach point per Spot.

        Geometria: frame camera_color_optical_frame (Z=depth forward, X=right, Y=down).
        Spot deve avvicinarsi lungo Z: goal_z = persona_z - approach_dist.
        Il navigator trasforma poi odom→body tramite TF.
        """
        # Goal: camera_color_optical_frame — X=right, Y=down, Z=depth
        # Approach along Z (depth): stay at same X/Y, move closer by approach_dist
        goal_x = bbox_center[0]
        goal_y = bbox_center[1]
        goal_z = bbox_center[2] - self.approach_dist
        
        # Crea goal message
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = header.frame_id
        goal.pose.position.x = float(goal_x)
        goal.pose.position.y = float(goal_y)
        goal.pose.position.z = float(goal_z)
        goal.pose.orientation.w = 1.0  # Orientamento neutro
        
        # Pubblica goal (anche in test mode per visualizzazione)
        self.goal_pub.publish(goal)
        
        # Pubblica marker punto di approccio
        self.publish_approach_marker(goal_x, goal_y, goal_z, header)
        
        if visualize_only:
            self.get_logger().info(
                f'📍 Approach point calcolato (VISUALIZZAZIONE): '
                f'({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f}) '
                f'[{self.approach_dist}m davanti a persona]',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().info(
                f'🎯 Approach point pubblicato: '
                f'({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f}) '
                f'[{self.approach_dist}m davanti a persona]'
            )
    
    def publish_approach_marker(self, x, y, z, header):
        """Visualizza punto di approccio target in RViz (sfera verde)."""
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = 'approach_point'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Verde brillante
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0  # Permanente
        
        self.approach_marker_pub.publish(marker)


def main():
    rclpy.init()
    node = LayingHumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
