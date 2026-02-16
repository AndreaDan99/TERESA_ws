import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO

import tf2_ros
import tf2_geometry_msgs


class YoloHandNode(Node):
    def __init__(self):
        super().__init__("yolo_hand_node")

        self.get_logger().info("Loading YOLO-Pose model...")
        self.model = YOLO("yolov8n-pose.pt")

        self.bridge = CvBridge()

        # --- SUBSCRIBERS ---
        self.sub_color = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10,
        )

        self.sub_depth = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )

        self.sub_info = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.info_callback,
            10,
        )

        self.depth_image = None
        self.color_info = None

        # --- PUBLISHER ---
        self.pub_hands = self.create_publisher(
            PoseArray, "/hand_pose/wrists_3d", 10
        )

        # --- TF BUFFER & LISTENER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def info_callback(self, msg):
        self.color_info = msg


    def depth_callback(self, msg):
        """Read depth (uint16, single channel)."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Ensure depth is mono-channel
        if len(img.shape) == 3:
            img = img[:, :, 0]

        self.depth_image = img


    def color_callback(self, msg):

        if self.depth_image is None or self.color_info is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLO inference
        results = self.model(color_img, verbose=False)
        result = results[0]

        if result.keypoints is None:
            return

        keypoints = result.keypoints.xy.cpu().numpy()

        if keypoints.shape[0] == 0:
            return

        # Take only first detected person
        kp = keypoints[0]

        LEFT_WRIST = 9
        RIGHT_WRIST = 10
        indices = [LEFT_WRIST, RIGHT_WRIST]

        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_depth_optical_frame"

        fx = self.color_info.k[0]
        fy = self.color_info.k[4]
        cx = self.color_info.k[2]
        cy = self.color_info.k[5]

        # Try to get TF transform once per callback
        try:
            transform = self.tf_buffer.lookup_transform(
                "camera_depth_optical_frame",
                "camera_color_optical_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        for idx in indices:
            u, v = kp[idx]
            u, v = int(u), int(v)

            if u < 0 or v < 0:
                continue
            if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
                continue

            depth_raw = self.depth_image[v, u]

            # Depth must be a scalar
            if isinstance(depth_raw, (np.ndarray, list)):
                depth_raw = depth_raw[0]

            depth = float(depth_raw)

            if depth <= 0 or np.isnan(depth):
                continue

            depth_m = depth * 0.001  # mm → meters

            # 3D reprojection in COLOR optical frame
            X = (u - cx) * depth_m / fx
            Y = (v - cy) * depth_m / fy
            Z = depth_m

            pose = Pose()
            pose.position.x = float(X)
            pose.position.y = float(Y)
            pose.position.z = float(Z)

            # Transform to DEPTH optical frame
            try:
                pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                continue

            pose_array.poses.append(pose)

        if len(pose_array.poses) > 0:
            self.pub_hands.publish(pose_array)
            self.get_logger().info(f"Published {len(pose_array.poses)} 3D wrist keypoints")


def main(args=None):
    rclpy.init(args=args)
    node = YoloHandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
