import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np

from ultralytics import YOLO

class YoloPoseNode(Node):
    def __init__(self):
        super().__init__('yolo_pose_node')

        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("yolov8n-pose.pt")

        self.bridge = CvBridge()

        # Subscribe RealSense topics
        self.sub_color = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10,
        )

        # ⭐ USE DEPTH ALIGNED TO COLOR ⭐
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

        self.pub_pose3d = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10
        )

    def info_callback(self, msg):
        self.color_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        if self.depth_image is None or self.color_info is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLO Pose inference
        results = self.model(color_img, verbose=False)
        result = results[0]

        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.header.frame_id = "camera_depth_optical_frame"
        # Loop through each detected person
        if result.keypoints is None:
            self.pub_pose3d.publish(pose_array)
            return

        keypoints_all = result.keypoints.xy.cpu().numpy()  # shape [N_people, 17, 2]

        for person_kp in keypoints_all:
            for (u, v) in person_kp:
                u, v = int(u), int(v)

                if u < 0 or v < 0:
                    continue
                if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
                    continue

                # Extract depth safely
                depth_pixel = self.depth_image[v, u]

                # If depth is multi-channel (array), pick the first entry
                if isinstance(depth_pixel, (np.ndarray, list, tuple)):
                    depth_pixel = depth_pixel[0]

                # Try converting to float
                try:
                    depth_val = float(depth_pixel)
                except:
                    continue

                # Skip invalid depth
                if depth_val <= 0 or np.isnan(depth_val):
                    continue

                # Convert mm → m
                depth = depth_val * 0.001


                fx = self.color_info.k[0]
                fy = self.color_info.k[4]
                cx = self.color_info.k[2]
                cy = self.color_info.k[5]

                # Reprojection
                X = (u - cx) * depth / fx
                Y = (v - cy) * depth / fy
                Z = depth

                p = Pose()
                p.position.x = float(X)
                p.position.y = float(Y)
                p.position.z = float(Z)
                pose_array.poses.append(p)

        self.pub_pose3d.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} 3D keypoints")


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
