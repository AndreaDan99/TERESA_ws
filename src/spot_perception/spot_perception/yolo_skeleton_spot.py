#!/usr/bin/env python3
"""
YOLO Skeleton Detection Node per Spot Boston Dynamics - OTTIMIZZATO + TF.
Processa RGB + Depth → skeleton 3D in BODY FRAME.
"""
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PointStamped
from visualization_msgs.msg import MarkerArray  
from cv_bridge import CvBridge

import numpy as np
import cv2
from ultralytics import YOLO

# Import TF2
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

# Import moduli locali
from .kalman_filter import Kalman3D
from .skeleton_utils import (
    torso_length_constraint, 
    compute_torso_length, 
    smooth_torso_length,
    SKELETON_EDGES
)
from .depth_processing import get_depth_at_pixel, filter_depth_outliers
from .visualization import build_skeleton_markers


class YoloSkeletonSpot(Node):
    """
    YOLO11-Pose detection con Kalman filtering 3D + trasformazione a body frame.
    """
    
    def __init__(self):
        super().__init__("yolo_skeleton_spot")
        
        # ============================================================
        # PARAMETRI FISSI (puoi sovrascrivere con launch file)
        # ============================================================
        self.declare_parameter("model_path", "yolo11n-pose.pt")
        self.declare_parameter("conf_thr", 0.3)
        self.declare_parameter("vel_damping", 0.6)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("camera_name", "frontleft")
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("device", "0")  # "0" per GPU, "cpu" per CPU
        self.declare_parameter("use_half", True)  # FP16 su GPU
        self.declare_parameter("target_frame", "body")  # Frame output: body di Spot
        
        # Leggi parametri
        self.conf_thr = float(self.get_parameter("conf_thr").value)
        self.vel_damping = float(self.get_parameter("vel_damping").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        camera_name = self.get_parameter("camera_name").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        device = self.get_parameter("device").value
        self.use_half = bool(self.get_parameter("use_half").value)
        self.target_frame = self.get_parameter("target_frame").value
        
        # YOLO Model
        model_path = self.get_parameter("model_path").value
        self.model = YOLO(model_path)
        
        # GPU setup con fallback
        try:
            self.model.to(device)
            self.get_logger().info(f"✅ Model loaded on device: {device}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to load on {device}, using CPU: {e}")
            device = "cpu"
            self.use_half = False
            self.model.to("cpu")
        
        self.device = device
        self.bridge = CvBridge()
        
        # TF2 Buffer e Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Synchronized subscribers
        self.sub_color = Subscriber(
            self, Image, f"/camera/{camera_name}/camera/image"
        )
        self.sub_depth = Subscriber(
            self, Image, f"/depth/{camera_name}/camera/image"
        )
        
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth],
            queue_size=2,
            slop=0.05
        )
        self.sync.registerCallback(self.cb_synchronized)
        
        # Camera info
        self.sub_info = self.create_subscription(
            CameraInfo, f"/camera/{camera_name}/camera_info", self.cb_info, 1
        )
        
        # Publishers
        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 1
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 1
        )
        
        # State
        self.cam_info = None
        self.num_joints = 17
        self.torso_len_ref = None
        self.torso_len_smoothed = None
        
        self.kf = [Kalman3D() for _ in range(self.num_joints)]
        self.visible = [False] * self.num_joints
        self.edges = SKELETON_EDGES
        
        self._tune_kalman_filters()
        
        self.get_logger().info(
            f"✅ YOLO Skeleton Node ready\n"
            f"   Camera: {camera_name}\n"
            f"   Target frame: {self.target_frame}\n"
            f"   Imgsz: {self.imgsz}\n"
            f"   Device: {self.device}\n"
            f"   Half precision: {self.use_half}"
        )
        
    def _tune_kalman_filters(self):
        """Tuning Kalman per torso/arms/legs."""
        TORSO = {5, 6, 11, 12}
        ARMS = {7, 8, 9, 10}
        LEGS = {13, 14, 15, 16}
        NOSE = {0}
        
        for i, kf in enumerate(self.kf):
            if i in TORSO:
                kf.Q *= 0.7
                kf.R *= 0.7
            elif i in ARMS:
                kf.Q *= 1.2
                kf.R *= 1.2
            elif i in LEGS:
                kf.Q *= 1.4
                kf.R *= 1.3
            elif i in NOSE:
                kf.Q *= 0.8
                kf.R *= 0.6

    def cb_info(self, msg):
        """Callback camera info."""
        self.cam_info = msg

    def cb_synchronized(self, msg_color, msg_depth):
        """Callback sincronizzato RGB + Depth."""
        
        if self.cam_info is None:
            self.get_logger().warn(
                "Camera info not received yet", 
                throttle_duration_sec=2.0
            )
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg_color, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(msg_depth, desired_encoding='passthrough')
            
            # Spot depth è in millimetri → converti a metri
            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0
            
            # ✅ RESIZE DEPTH to match RGB usando INTER_NEAREST
            rgb_height, rgb_width = frame.shape[:2]
            depth_height, depth_width = depth_img.shape[:2]
            
            if (depth_height != rgb_height) or (depth_width != rgb_width):
                depth_img = cv2.resize(
                    depth_img, 
                    (rgb_width, rgb_height),
                    interpolation=cv2.INTER_NEAREST  # ✅ Preserva discontinuità depth
                )
                
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return
        
        # YOLO inference
        try:
            results = self.model.predict(
                frame,
                conf=self.conf_thr,
                classes=[0],
                verbose=False,
                imgsz=self.imgsz,
                half=self.use_half,
                device=self.device
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return
        
        # Validation
        if len(results) == 0 or results[0].keypoints is None:
            return
        
        kp_data = results[0].keypoints
        if kp_data.xy is None or kp_data.xy.shape[0] == 0:
            return
        
        # Extract keypoints
        kp_xy = kp_data.xy.cpu().numpy()[0]
        kp_conf = kp_data.conf.cpu().numpy()[0]
        
        self._process_skeleton(kp_xy, kp_conf, depth_img, msg_color.header)



    def _process_skeleton(self, kp_xy, kp_conf, depth_img, header):
        """Processa skeleton: depth lookup + Kalman + constraint + TF transform."""
        
        # ✅ DEBUG: Log detection
        self.get_logger().info(
            f'Processing skeleton: {len(kp_xy)} keypoints detected',
            throttle_duration_sec=1.0
        )
        
        K = np.array(self.cam_info.k).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        
        pts = [None] * self.num_joints
        
        # Depth lookup + backprojection nel CAMERA FRAME
        valid_count = 0
        for i in range(self.num_joints):
            if kp_conf[i] < self.conf_thr:
                self.visible[i] = False
                continue
            
            u, v = int(kp_xy[i, 0]), int(kp_xy[i, 1])
            depth = get_depth_at_pixel(depth_img, u, v, window_size=3)
            depth = filter_depth_outliers(depth, self.max_depth_m)
            
            if depth is None:
                self.visible[i] = False
                continue
            
            # Backproject 2D → 3D (camera frame)
            x = (u - cx) * depth / fx
            y = (v - cy) * depth / fy
            z = depth
            
            pts[i] = np.array([x, y, z], dtype=np.float64)
            self.visible[i] = True
            valid_count += 1
        
        # ✅ DEBUG: Log valid points
        self.get_logger().info(
            f'Valid 3D points: {valid_count}/17',
            throttle_duration_sec=1.0
        )

        # Torso constraint (nel camera frame)
        if self.torso_len_ref is None:
            new_len = compute_torso_length(pts)
            if new_len is not None:
                self.torso_len_ref = new_len
                self.torso_len_smoothed = new_len
        else:
            new_len = compute_torso_length(pts)
            self.torso_len_smoothed = smooth_torso_length(
                new_len, self.torso_len_smoothed, alpha=0.3
            )
        
        pts = torso_length_constraint(pts, self.visible, self.torso_len_smoothed)
        
        # Kalman filtering (nel camera frame)
        for i in range(self.num_joints):
            self.kf[i].predict(self.vel_damping)
            
            if pts[i] is not None:
                self.kf[i].update(pts[i])
            
            pts[i] = self.kf[i].get_position()
        
        # Trasforma da camera frame a body frame
        pts_transformed = self._transform_points_to_target(
            pts, 
            self.cam_info.header.frame_id,
            self.target_frame,
            header.stamp
        )
        
        # Publish nel body frame
        self._publish_skeleton(pts_transformed, header)

    def _transform_points_to_target(self, pts, source_frame, target_frame, stamp):
        """Trasforma lista di punti da source a target frame usando TF2."""
        
        # ✅ AGGIUNTO: Se target_frame vuoto, salta trasformazione
        if not target_frame or target_frame == '':
            return pts
        
        try:
            # ✅ CORRETTO: Usa Time(0) per latest available transform
            # Evita extrapolation error
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # ✅ Usa latest available invece di stamp specifico
                timeout=rclpy.duration.Duration(seconds=0.5)  # Aumentato timeout
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Transform {source_frame}→{target_frame} failed: {e}",
                throttle_duration_sec=2.0
            )
            return pts  # Return original se transform fallisce
        
        # Trasforma ogni punto
        pts_transformed = []
        for p in pts:
            if p is None:
                pts_transformed.append(None)
                continue
            
            # Crea PointStamped con Time(0) per latest
            point_stamped = PointStamped()
            point_stamped.header.stamp = rclpy.time.Time().to_msg()  # ✅ Latest
            point_stamped.header.frame_id = source_frame
            point_stamped.point.x = float(p[0])
            point_stamped.point.y = float(p[1])
            point_stamped.point.z = float(p[2])
            
            # Applica transform
            transformed = do_transform_point(point_stamped, transform)
            
            # Converti back a numpy
            pts_transformed.append(np.array([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z
            ], dtype=np.float64))
        
        return pts_transformed


    def _publish_skeleton(self, pts, header):
        """
        Pubblica PoseArray + Markers nel target frame.
        
        IMPORTANTE: Pubblica SEMPRE 17 pose (anche se None → NaN)
        """
        pa = PoseArray()
        pa.header = header
        pa.header.frame_id = self.target_frame if self.target_frame else header.frame_id
        
        # ✅ CORRETTO: Pubblica SEMPRE 17 pose
        for i in range(self.num_joints):
            pose = Pose()
            
            if pts[i] is not None:
                # Punto valido
                pose.position.x = float(pts[i][0])
                pose.position.y = float(pts[i][1])
                pose.position.z = float(pts[i][2])
            else:
                # Punto non valido → NaN
                pose.position.x = float('nan')
                pose.position.y = float('nan')
                pose.position.z = float('nan')
            
            pose.orientation.w = 1.0  # Orientamento default
            pa.poses.append(pose)
        
        # Pubblica PoseArray (sempre 17 elementi)
        self.pub_poses.publish(pa)
        
        # Markers per visualizzazione
        ma = build_skeleton_markers(pa, pts, self.visible)
        self.pub_markers.publish(ma)



def main():
    rclpy.init()
    node = YoloSkeletonSpot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
