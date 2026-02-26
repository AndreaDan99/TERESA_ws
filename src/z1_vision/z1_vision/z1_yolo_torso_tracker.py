#!/usr/bin/env python3
"""
Z1 YOLO Torso Tracker + Impedance Control
- YOLO → Torso center (camera_depth_optical_frame)
- Salva target quando torso sparisce
- Pubblica target per trajectory_manager (link06 frame)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from message_filters import ApproximateTimeSynchronizer, Subscriber

# COCO Keypoints YOLO (0-based indices)
TORSO_KEYPOINTS = [5, 6, 11, 12]  # Left/Right Shoulder + Left/Right Hip [web:71]

class Z1YoloTorsoTracker(Node):
    def __init__(self):
        super().__init__('z1_yolo_torso_tracker')
        
        # Parametri
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('conf_thr', 0.5)
        self.declare_parameter('max_depth', 2.5)
        self.declare_parameter('lost_timeout', 1.0)  # sec prima di salvare target
        
        self.model_path = self.get_parameter('model_path').value
        self.conf_thr = self.get_parameter('conf_thr').value
        self.max_depth = self.get_parameter('max_depth').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        
        # YOLO
        self.model = YOLO(self.model_path)
        self.model.to('cuda:0')  # GPU
        
        # Bridge + TF
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers sincronizzati (Realsense)
        self.sub_rgb = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sub_info = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')
        
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], 10, 0.05)
        self.sync.registerCallback(self.callback)
        
        # Publishers
        self.pub_torso_center = self.create_publisher(PointStamped, '/torso_target_camera', 10)
        self.pub_torso_ee = self.create_publisher(PoseStamped, '/torso_target_ee', 10)  # Per trajectory_manager
        self.pub_detection = self.create_publisher(Detection2DArray, '/yolo_detections', 10)
        self.pub_torso_visible = self.create_publisher(Bool, '/torso_visible', 10)
        
        # State
        self.cam_k = None
        self.last_torso = None
        self.target_saved = None
        self.lost_timer = None
        self.torso_visible = False
        
        self.get_logger().info('🚀 Z1 YOLO Torso Tracker pronto!')

    def callback(self, rgb_msg, depth_msg):
        if self.cam_k is None:
            return
            
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except:
            return
        
        # YOLO detection
        results = self.model(rgb, conf=self.conf_thr, verbose=False, classes=[0])
        
        detections = Detection2DArray()
        detections.header = rgb_msg.header
        
        torso_center = None
        
        for r in results:
            boxes = r.boxes
            kpts = r.keypoints.xy.cpu().numpy()
            
            if len(kpts) == 0 or kpts.shape[1] != 17:
                continue
                
            # ✅ TORSO CENTER: media spalle + fianchi (COCO keypoints 5,6,11,12)
            torso_pts = []
            for idx in TORSO_KEYPOINTS:
                x, y = int(kpts[0, idx, 0]), int(kpts[0, idx, 1])
                if 0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]:
                    d = depth[y, x]
                    if d > 0 and d < self.max_depth:
                        # Backproject
                        fx, fy, cx, cy = self.cam_k[0,0], self.cam_k[1,1], self.cam_k[0,2], self.cam_k[1,2]
                        X = (x - cx) * d / fx
                        Y = (y - cy) * d / fy
                        Z = d
                        torso_pts.append([X, Y, Z])
            
            if len(torso_pts) >= 2:  # Almeno 2 punti torso visibili
                torso_center = np.mean(torso_pts, axis=0)
                self.torso_visible = True
                self.lost_timer = self.get_clock().now()
                
                # Pubblica in camera frame
                pt = PointStamped()
                pt.header = rgb_msg.header
                pt.point.x, pt.point.y, pt.point.z = torso_center
                self.pub_torso_center.publish(pt)
                
                self.last_torso = torso_center.copy()
            else:
                self.torso_visible = False
            
            # ✅ NUOVO: Pubblica flag torso visibile per surface node
            vis_msg = Bool()
            vis_msg.data = self.torso_visible
            self.pub_torso_visible.publish(vis_msg)
        
        # ✅ LOGICA LOST TARGET
        if not self.torso_visible:
            if self.lost_timer is None:
                self.lost_timer = self.get_clock().now()
            
            elapsed = (self.get_clock().now() - self.lost_timer).nanoseconds / 1e9
            if elapsed > self.lost_timeout and self.last_torso is not None and self.target_saved is None:
                # SALVA TARGET dall'ultima posizione valida!
                self.target_saved = self.last_torso.copy()
                self.get_logger().info(f'🎯 TARGET SALVATO: {self.target_saved}')
        else:
            self.lost_timer = self.get_clock().now()
        
        # Transform a EE frame (link06) per trajectory_manager
        if self.target_saved is not None or torso_center is not None:
            target_pt = self.target_saved if self.target_saved is not None else torso_center
            
            pt_ee = PointStamped()
            pt_ee.header.frame_id = 'camera_depth_optical_frame'
            pt_ee.header.stamp = self.get_clock().now().to_msg()
            pt_ee.point.x, pt_ee.point.y, pt_ee.point.z = target_pt
            
            try:
                t = self.tf_buffer.lookup_transform('link06', 'camera_depth_optical_frame', 
                                                  rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                pt_ee_transformed = do_transform_point(pt_ee, t)
                pose = PoseStamped(header=pt_ee_transformed.header)
                pose.pose.position = pt_ee_transformed.point
                pose.pose.orientation.w = 1.0
                self.pub_torso_ee.publish(pose)
            except:
                pass  # TF non pronto
        
        detections.header = rgb_msg.header
        self.pub_detection.publish(detections)

    def camera_info_callback(self, msg):
        self.cam_k = np.array(msg.k).reshape(3,3)

def main():
    rclpy.init()
    node = Z1YoloTorsoTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
