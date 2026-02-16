#!/usr/bin/env python3
"""
Spot Camera Assistant - Usa telecamere grayscale Spot per:
1. Rilevare ostacoli laterali (edge detection)
2. Rilevare movimento quando Orbbec perde tracking (optical flow)
3. Dare warning al target_tracker
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class SpotCameraAssistant(Node):
    def __init__(self):
        super().__init__('spot_camera_assistant')
        
        self.bridge = CvBridge()
        
        # --- SUBSCRIBERS alle 5 telecamere Spot ---
        self.create_subscription(Image, '/spot/camera/frontleft/image', 
                                self.frontleft_cb, 10)
        self.create_subscription(Image, '/spot/camera/frontright/image', 
                                self.frontright_cb, 10)
        self.create_subscription(Image, '/spot/camera/left/image', 
                                self.left_cb, 10)
        self.create_subscription(Image, '/spot/camera/right/image', 
                                self.right_cb, 10)
        self.create_subscription(Image, '/spot/camera/back/image', 
                                self.back_cb, 10)
        
        # Subscribe a stato detection Orbbec (per sapere quando è persa)
        self.create_subscription(Bool, '/human_pose/detection_active',
                                self.detection_status_cb, 10)
        
        # --- PUBLISHERS ---
        self.pub_obstacle_warning = self.create_publisher(
            String, '/spot/obstacle_warning', 10)
        
        self.pub_motion_direction = self.create_publisher(
            String, '/spot/motion_direction', 10)
        
        # --- STATE ---
        self.prev_frames = {}  # Per optical flow
        self.orbbec_detecting = True  # Orbbec sta rilevando persona?
        
        # Parametri
        self.declare_parameter('edge_threshold', 100)  # Soglia Canny
        self.declare_parameter('motion_threshold', 1000)  # Pixel in movimento
        
        self.edge_thr = self.get_parameter('edge_threshold').value
        self.motion_thr = self.get_parameter('motion_threshold').value
        
        self.get_logger().info('✅ Spot Camera Assistant READY (grayscale only)')
    
    def detection_status_cb(self, msg: Bool):
        """Aggiorna stato detection Orbbec"""
        self.orbbec_detecting = msg.data
    
    # =====================================================
    # CALLBACKS TELECAMERE
    # =====================================================
    
    def frontleft_cb(self, msg: Image):
        self.process_frame(msg, 'frontleft')
    
    def frontright_cb(self, msg: Image):
        self.process_frame(msg, 'frontright')
    
    def left_cb(self, msg: Image):
        self.process_frame(msg, 'left')
    
    def right_cb(self, msg: Image):
        self.process_frame(msg, 'right')
    
    def back_cb(self, msg: Image):
        self.process_frame(msg, 'back')
    
    # =====================================================
    # PROCESSING
    # =====================================================
    
    def process_frame(self, msg: Image, camera_name: str):
        """
        Processa frame grayscale:
        1. Edge detection per ostacoli vicini
        2. Optical flow per movimento (solo se Orbbec perde tracking)
        """
        # Converti a grayscale numpy
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        # --- 1. OBSTACLE DETECTION (sempre attivo) ---
        self.detect_obstacles(gray, camera_name)
        
        # --- 2. MOTION DETECTION (solo se Orbbec perde tracking) ---
        if not self.orbbec_detecting:
            self.detect_motion(gray, camera_name)
        
        # Salva frame per prossima iterazione (optical flow)
        self.prev_frames[camera_name] = gray
    
    def detect_obstacles(self, gray, camera_name):
        """
        Rileva ostacoli vicini con Canny edge detection.
        Se molti edge nella parte bassa dell'immagine → ostacolo vicino
        """
        # Canny edge detection
        edges = cv2.Canny(gray, self.edge_thr, self.edge_thr * 2)
        
        # Analizza solo metà inferiore (ground level)
        h, w = edges.shape
        bottom_half = edges[h//2:, :]
        
        # Conta pixel edge
        edge_count = np.sum(bottom_half > 0)
        
        # Soglia: se troppi edge → ostacolo
        if edge_count > 5000:  # Tunable
            warning = f"OBSTACLE_{camera_name.upper()}"
            self.pub_obstacle_warning.publish(String(data=warning))
            self.get_logger().warn(
                f'⚠️ Obstacle detected on {camera_name} ({edge_count} edge pixels)',
                throttle_duration_sec=2.0
            )
    
    def detect_motion(self, gray, camera_name):
        """
        Rileva movimento con optical flow (solo se Orbbec ha perso tracking).
        Se rileva movimento → informa target_tracker della direzione
        """
        if camera_name not in self.prev_frames:
            return
        
        prev_gray = self.prev_frames[camera_name]
        
        # Calcola optical flow (metodo semplice: frame differencing)
        diff = cv2.absdiff(gray, prev_gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        
        # Conta pixel in movimento
        motion_pixels = np.sum(thresh > 0)
        
        # Se movimento significativo → pubblica direzione
        if motion_pixels > self.motion_thr:
            direction = f"MOTION_{camera_name.upper()}"
            self.pub_motion_direction.publish(String(data=direction))
            self.get_logger().info(
                f'👁️ Motion detected on {camera_name} ({motion_pixels} pixels)',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = SpotCameraAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
