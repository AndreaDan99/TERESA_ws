#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_matrix
import tf_transformations as tf

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

class RealSenseSurfaceNode(Node):
    def __init__(self):
        super().__init__("realsense_surface_node")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parametri
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("ee_frame", "link06")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("patch_radius_px", 30)
        self.declare_parameter("min_depth", 0.10)
        self.declare_parameter("max_depth", 2.0)
        self.declare_parameter("desired_normal_offset", -0.005)

        # Debug / test params
        self.declare_parameter("require_lock_valid", True)
        self.declare_parameter("use_latest_tf", True)  # if True, ignore msg stamp and use latest TF
        self.declare_parameter("publish_debug_markers", True)
        self.declare_parameter("debug_marker_ns", "torso_surface")

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.patch_r = self.get_parameter("patch_radius_px").get_parameter_value().integer_value
        self.min_depth = self.get_parameter("min_depth").get_parameter_value().double_value
        self.max_depth = self.get_parameter("max_depth").get_parameter_value().double_value
        self.desired_normal_offset = self.get_parameter("desired_normal_offset").get_parameter_value().double_value

        self.require_lock_valid = self.get_parameter("require_lock_valid").get_parameter_value().bool_value
        self.use_latest_tf = self.get_parameter("use_latest_tf").get_parameter_value().bool_value
        self.publish_debug_markers = self.get_parameter("publish_debug_markers").get_parameter_value().bool_value
        self.debug_marker_ns = self.get_parameter("debug_marker_ns").get_parameter_value().string_value

        self.fx = self.fy = self.ppx = self.ppy = None

        # Optional: torso target already in base/world frame (stable), useful if /torso_target_camera is not published
        self.torso_world_sub = self.create_subscription(
            PoseStamped, "/torso_target_ee_locked", self.torso_world_callback, 10
        )
        self.lock_valid_sub = self.create_subscription(
            Bool, "/target_lock_valid", self.lock_valid_callback, 10
        )

        # Subscriber Realsense
        self.depth_sub = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self.info_callback, 10
        )

        # Publisher
        self.surface_pub = self.create_publisher(PoseStamped, "/torso_surface_frame", 10)
        self.dist_pub = self.create_publisher(Float32, "/surface_signed_distance", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/torso_surface_markers", 10)
        self.surface_point_pub = self.create_publisher(PointStamped, "/torso_surface_point", 10)

        # State torso
        self.torso_center_base = None         # np.array([x,y,z]) in base/world frame
        self.target_lock_valid = False

        self.get_logger().info("="*70)
        self.get_logger().info("🚀 TORSO-SPECIFIC Surface Node")
        self.get_logger().info("ROI dinamico su /torso_target_camera (fallback: /torso_target_ee_locked) | gated by /target_lock_valid")
        self.get_logger().info("="*70)

    def lock_valid_callback(self, msg: Bool):
        self.target_lock_valid = msg.data
        if not self.target_lock_valid:
            self.torso_center_base = None
            self.torso_center_cam = None

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.ppx = msg.k[2]
            self.ppy = msg.k[5]
            self.get_logger().info(
                f"✅ Intrinseci: fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.ppx:.1f}, cy={self.ppy:.1f}"
            )

    def torso_world_callback(self, msg: PoseStamped):
        """Centro torso 3D (base/world frame), es. da /torso_target_ee_locked"""
        self.torso_center_base = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ])


    def project_to_image(self, pt_cam):
        """3D camera → pixel (u,v)"""
        if self.fx is None or pt_cam[2] <= 0:
            return None
        u = self.ppx + self.fx * pt_cam[0] / pt_cam[2]
        v = self.ppy + self.fy * pt_cam[1] / pt_cam[2]
        return int(u), int(v)

    def depth_to_points(self, depth_img, mask=None):
        if self.fx is None:
            return None
        h, w = depth_img.shape
        if mask is None:
            mask = depth_img > 0
        ys, xs = np.where(mask)
        zs = depth_img[ys, xs]
        valid = (zs > self.min_depth) & (zs < self.max_depth)
        xs, ys, zs = xs[valid], ys[valid], zs[valid]
        if zs.size < 10:
            return None
        xs_f = (xs - self.ppx) * zs / self.fx
        ys_f = (ys - self.ppy) * zs / self.fy
        return np.vstack([xs_f, ys_f, zs]).T

    def fit_plane_pca(self, pts):
        p0 = pts.mean(axis=0)
        pts_c = pts - p0
        _, _, Vt = np.linalg.svd(pts_c, full_matrices=False)
        n = Vt[-1, :] / np.linalg.norm(Vt[-1, :])
        return p0, n

    def quat_to_rot(self, q):
        return tf.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    def _base_point_to_camera(self, p_base, stamp_msg):
        """Trasforma un punto (base/world) in camera frame usando TF."""
        try:
            tf_time = rclpy.time.Time() if self.use_latest_tf else rclpy.time.Time.from_msg(stamp_msg)
            tf_cam_base = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame,
                tf_time,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(
                f"⚠️ TF {self.camera_frame}<-{self.base_frame} non disponibile: {e}",
                throttle_duration_sec=2.0,
            )
            return None

        R_cb = self.quat_to_rot(tf_cam_base.transform.rotation)
        t_cb = np.array([
            tf_cam_base.transform.translation.x,
            tf_cam_base.transform.translation.y,
            tf_cam_base.transform.translation.z,
        ])
        p_cam = R_cb @ p_base + t_cb
        return p_cam

    def depth_callback(self, msg: Image):
        
        if self.require_lock_valid and (not self.target_lock_valid):
            return

        # SOLO LOCKED: usiamo solo il punto in world (/torso_target_ee_locked) e lo trasformiamo in camera
        if self.torso_center_base is None:
            return

        p_cam = self._base_point_to_camera(self.torso_center_base, msg.header.stamp)
        if p_cam is None:
            return

        self.torso_center_cam = p_cam

        if self.fx is None:
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except:
            return
        
        if msg.encoding == "16UC1":
            depth_img = depth_img.astype(np.float32) / 1000.0
        else:
            depth_img = depth_img.astype(np.float32)

        h, w = depth_img.shape

        # If the depth message provides a frame_id, prefer it over the parameter (reduces TF mismatch issues)
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

        # ✅ ROI su PROIEZIONE TORSO
        torso_uv = self.project_to_image(self.torso_center_cam)
        if torso_uv is None:
            return
        u_ee, v_ee = torso_uv
        
        u_min = max(u_ee - self.patch_r, 0)
        u_max = min(u_ee + self.patch_r, w - 1)
        v_min = max(v_ee - self.patch_r, 0)
        v_max = min(v_ee + self.patch_r, h - 1)

        mask = np.zeros((h, w), dtype=bool)
        mask[v_min:v_max+1, u_min:u_max+1] = True

        pts_cam = self.depth_to_points(depth_img, mask=mask)
        if pts_cam is None:
            return

        # Fit piano
        try:
            p0_cam, n_cam = self.fit_plane_pca(pts_cam)
        except:
            return

        if n_cam[2] < 0:
            n_cam = -n_cam

        # TF base → camera
        try:
            tf_time = rclpy.time.Time() if self.use_latest_tf else rclpy.time.Time.from_msg(msg.header.stamp)
            tf_base_cam = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                tf_time,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF {self.base_frame}<-{self.camera_frame} non disponibile: {e}", throttle_duration_sec=2.0)
            return

        R_bc = self.quat_to_rot(tf_base_cam.transform.rotation)
        t_bc = np.array([tf_base_cam.transform.translation.x,
                         tf_base_cam.transform.translation.y,
                         tf_base_cam.transform.translation.z])

        p0_base = R_bc @ p0_cam + t_bc
        n_base = R_bc @ n_cam

        # Distanza TCP-superficie
        try:
            tf_time = rclpy.time.Time() if self.use_latest_tf else rclpy.time.Time.from_msg(msg.header.stamp)
            tf_base_ee = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                tf_time,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF {self.base_frame}<-{self.ee_frame} non disponibile: {e}", throttle_duration_sec=2.0)
            return

        tcp_base = np.array([
            tf_base_ee.transform.translation.x,
            tf_base_ee.transform.translation.y,
            tf_base_ee.transform.translation.z,
        ])

        # Orient normal to point from the surface toward the TCP (stable sign convention)
        if np.dot(n_base, (tcp_base - p0_base)) < 0.0:
            n_base = -n_base

        d = float(np.dot(tcp_base - p0_base, n_base))

        # Costruisci Pose superficie
        z_axis = n_base / np.linalg.norm(n_base)
        p0_base = p0_base + self.desired_normal_offset * z_axis
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
        surf_msg.pose.orientation.x = float(q[0])
        surf_msg.pose.orientation.y = float(q[1])
        surf_msg.pose.orientation.z = float(q[2])
        surf_msg.pose.orientation.w = float(q[3])
        
        self.surface_pub.publish(surf_msg)

        dist_msg = Float32()
        dist_msg.data = float(d)
        self.dist_pub.publish(dist_msg)
        
        # Debug point
        ptm = PointStamped()
        ptm.header = surf_msg.header
        ptm.point.x = float(p0_base[0])
        ptm.point.y = float(p0_base[1])
        ptm.point.z = float(p0_base[2])
        self.surface_point_pub.publish(ptm)

        # Debug markers for RViz
        if self.publish_debug_markers:
            ma = MarkerArray()

            # Sphere at surface point
            m0 = Marker()
            m0.header = surf_msg.header
            m0.ns = self.debug_marker_ns
            m0.id = 0
            m0.type = Marker.SPHERE
            m0.action = Marker.ADD
            m0.pose = surf_msg.pose
            m0.scale.x = 0.05
            m0.scale.y = 0.05
            m0.scale.z = 0.05
            m0.color.r = 0.0
            m0.color.g = 1.0
            m0.color.b = 0.0
            m0.color.a = 0.9
            m0.lifetime = Duration(sec=0, nanosec=250_000_000)
            ma.markers.append(m0)

            # Arrow for normal
            m1 = Marker()
            m1.header = surf_msg.header
            m1.ns = self.debug_marker_ns
            m1.id = 1
            m1.type = Marker.ARROW
            m1.action = Marker.ADD
            p_start = Point(x=float(p0_base[0]), y=float(p0_base[1]), z=float(p0_base[2]))
            p_end = Point(
                x=float(p0_base[0] + 0.15 * z_axis[0]),
                y=float(p0_base[1] + 0.15 * z_axis[1]),
                z=float(p0_base[2] + 0.15 * z_axis[2]),
            )
            m1.points = [p_start, p_end]
            m1.scale.x = 0.01  # shaft diameter
            m1.scale.y = 0.02  # head diameter
            m1.scale.z = 0.02  # head length
            m1.color.r = 1.0
            m1.color.g = 0.6
            m1.color.b = 0.0
            m1.color.a = 0.9
            m1.lifetime = Duration(sec=0, nanosec=250_000_000)
            ma.markers.append(m1)

            self.marker_pub.publish(ma)
        
        self.get_logger().info(
            f"✅ TORSO SURFACE [{p0_base[0]:.3f}, {p0_base[1]:.3f}, {p0_base[2]:.3f}] "
            f"ROI@({u_ee},{v_ee}) dist: {d*1000:.1f}mm"
        )
        self.get_logger().info(
            f"🔍 normale {self.base_frame}: [{n_base[0]:.3f}, {n_base[1]:.3f}, {n_base[2]:.3f}] | "
            f"offset: {self.desired_normal_offset:+.4f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseSurfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
