#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_matrix
import tf_transformations as tf

class RealSenseSurfaceNode(Node):
    def __init__(self):
        super().__init__("realsense_surface_node")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parametri
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("ee_frame", "link06")  # o ee_link
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("patch_radius_px", 20)
        self.declare_parameter("min_depth", 0.18)  # [m]
        self.declare_parameter("max_depth", 0.50)  # [m]

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.patch_r = self.get_parameter("patch_radius_px").get_parameter_value().integer_value
        self.min_depth = self.get_parameter("min_depth").get_parameter_value().double_value
        self.max_depth = self.get_parameter("max_depth").get_parameter_value().double_value

        self.fx = self.fy = self.ppx = self.ppy = None

        # Subscriber
        self.depth_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw",
            self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info",
            self.info_callback, 10
        )

        # Publisher
        self.surface_pub = self.create_publisher(PoseStamped, "surface_frame", 10)
        self.dist_pub = self.create_publisher(Float32, "surface_signed_distance", 10)

        self.get_logger().info("="*70)
        self.get_logger().info("RealSense Surface Node Inizializzato")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")
        self.get_logger().info(f"EE frame: {self.ee_frame}")
        self.get_logger().info(f"Base frame: {self.base_frame}")
        self.get_logger().info(f"ROI patch radius: {self.patch_r}px")
        self.get_logger().info(f"Depth range: [{self.min_depth:.2f}, {self.max_depth:.2f}] m")
        self.get_logger().info("="*70)

    def info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.ppx = msg.k[2]
            self.ppy = msg.k[5]
            self.get_logger().info(f"✅ Intrinseci: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.ppx:.1f}, cy={self.ppy:.1f}")

    def depth_to_points(self, depth_img, mask=None):
        """Converte depth image + maschera in point cloud Nx3 nel frame camera."""
        if self.fx is None:
            return None

        h, w = depth_img.shape
        if mask is None:
            mask = depth_img > 0

        ys, xs = np.where(mask)
        zs = depth_img[ys, xs]

        # Filtro range depth
        valid = (zs > self.min_depth) & (zs < self.max_depth)
        xs = xs[valid]
        ys = ys[valid]
        zs = zs[valid]
        
        if zs.size < 10:
            return None

        # Backprojection
        xs_f = (xs - self.ppx) * zs / self.fx
        ys_f = (ys - self.ppy) * zs / self.fy

        pts = np.vstack([xs_f, ys_f, zs]).T  # Nx3
        return pts

    def fit_plane_pca(self, pts):
        """Fit piano p = p0 + n·d via PCA."""
        p0 = pts.mean(axis=0)
        pts_c = pts - p0
        _, _, Vt = np.linalg.svd(pts_c, full_matrices=False)
        n = Vt[-1, :]  # autovettore con varianza minima
        n = n / np.linalg.norm(n)
        return p0, n

    def quat_to_rot(self, q):
        return tf.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    def depth_callback(self, msg: Image):
        if self.fx is None:
            return

        # 1) Depth → CV2
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        # Gestione encoding (mm vs m)
        if msg.encoding == "16UC1":
            depth_img = depth_img.astype(np.float32) / 1000.0  # mm → m
        else:
            depth_img = depth_img.astype(np.float32)

        h, w = depth_img.shape

        # 2) TF camera → EE per proiezione TCP
        try:
            tf_cam_ee = self.tf_buffer.lookup_transform(
                self.camera_frame, self.ee_frame, rclpy.time.Time()
            )
        except Exception:
            return

        tx = tf_cam_ee.transform.translation
        if tx.z <= 0:  # TCP dietro la camera
            return

        # Proiezione TCP nel depth image
        u_ee = int(self.fx * tx.x / tx.z + self.ppx)
        v_ee = int(self.fy * tx.y / tx.z + self.ppy)

        if not (0 <= u_ee < w and 0 <= v_ee < h):
            return

        # 3) ROI attorno al TCP
        r = self.patch_r
        u_min = max(u_ee - r, 0)
        u_max = min(u_ee + r, w - 1)
        v_min = max(v_ee - r, 0)
        v_max = min(v_ee + r, h - 1)

        # Crea maschera ROI
        mask = np.zeros((h, w), dtype=bool)
        mask[v_min:v_max+1, u_min:u_max+1] = True

        pts_cam = self.depth_to_points(depth_img, mask=mask)
        if pts_cam is None:
            return

        # 4) Fit piano nel frame camera
        p0_cam, n_cam = self.fit_plane_pca(pts_cam)

        # Orienta normale verso la camera (z > 0)
        if n_cam[2] < 0:
            n_cam = -n_cam

        # 5) TF base → camera per trasformare piano
        try:
            tf_base_cam = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time()
            )
        except Exception:
            return

        R_bc = self.quat_to_rot(tf_base_cam.transform.rotation)
        t_bc = np.array([
            tf_base_cam.transform.translation.x,
            tf_base_cam.transform.translation.y,
            tf_base_cam.transform.translation.z
        ])

        p0_base = R_bc @ p0_cam + t_bc
        n_base = R_bc @ n_cam

        # 6) Distanza firmata TCP–superficie
        try:
            tf_base_ee = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
        except Exception:
            return

        tcp_base = np.array([
            tf_base_ee.transform.translation.x,
            tf_base_ee.transform.translation.y,
            tf_base_ee.transform.translation.z
        ])

        d = np.dot(tcp_base - p0_base, n_base)  # >0 sopra, <0 dentro

        # 7) Costruisci PoseStamped surface_frame
        z_axis = n_base / np.linalg.norm(n_base)
        
        # Scegli x,y ortonormali
        aux = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(aux, z_axis)) > 0.9:
            aux = np.array([0.0, 1.0, 0.0])
        
        x_axis = np.cross(aux, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        T = np.eye(4)
        T[:3, 0] = x_axis
        T[:3, 1] = y_axis
        T[:3, 2] = z_axis
        T[:3, 3] = p0_base

        q = quaternion_from_matrix(T)

        surf_msg = PoseStamped()
        surf_msg.header.stamp = msg.header.stamp
        surf_msg.header.frame_id = self.base_frame
        surf_msg.pose.position.x = float(p0_base[0])
        surf_msg.pose.position.y = float(p0_base[1])
        surf_msg.pose.position.z = float(p0_base[2])
        surf_msg.pose.orientation.x = q[0]
        surf_msg.pose.orientation.y = q[1]
        surf_msg.pose.orientation.z = q[2]
        surf_msg.pose.orientation.w = q[3]
        self.surface_pub.publish(surf_msg)

        dist_msg = Float32()
        dist_msg.data = float(d)
        self.dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseSurfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
