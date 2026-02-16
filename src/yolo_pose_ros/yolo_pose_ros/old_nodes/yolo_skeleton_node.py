import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO


class YoloSkeletonNode(Node):
    def __init__(self):
        super().__init__("yolo_skeleton_node")

        self.get_logger().info("Loading YOLO Pose model...")
        self.model = YOLO("yolov8n-pose.pt")

        self.bridge = CvBridge()

        # Subscribe to color, depth, camera_info
        self.sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.color_callback, 10
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

        # Publishers
        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10
        )
        self.pub_skeleton = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10
        )

        self.edges = [
            (0, 1), (0, 2),
            (1, 3), (2, 4),       
            (5, 6),
            (5, 7), (7, 9), 
            (6, 8),(8,10),        
            (11, 12), 
            (11, 13), (13, 15),           
            (12, 14), (14, 16), 
            (0, 5), (0, 6),
            (5, 11), (6, 12)         
        ]

    # ----------------------- CALLBACKS -------------------------

    def info_callback(self, msg: CameraInfo):
        self.color_info = msg

    def depth_callback(self, msg: Image):
        # Depth uint16 in mm
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def color_callback(self, msg: Image):
        if self.depth_image is None or self.color_info is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(color_img, verbose=False)

        if len(results) == 0:
            return

        result = results[0]
        if result.keypoints is None:
            return

        keypoints = result.keypoints.xy.cpu().numpy()  # [N_person, K, 2]
        if keypoints.shape[0] == 0:
            return

        # Prendiamo solo la prima persona
        kp = keypoints[0]  # shape [K, 2]

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = "camera_color_optical_frame"

        points_3d = []

        fx = self.color_info.k[0]
        fy = self.color_info.k[4]
        cx = self.color_info.k[2]
        cy = self.color_info.k[5]

        for (u, v) in kp:
            u, v = int(u), int(v)

            # Controllo bounds
            if not (
                0 <= u < self.depth_image.shape[1]
                and 0 <= v < self.depth_image.shape[0]
            ):
                points_3d.append(None)
                continue

            d = float(self.depth_image[v, u])  # mm
            if d <= 0:
                points_3d.append(None)
                continue

            depth = d * 0.001  # mm -> m

            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth

            p = Pose()
            p.position.x = float(X)
            p.position.y = float(Y)
            p.position.z = float(Z)
            p.orientation.w = 1.0

            pose_array.poses.append(p)
            points_3d.append((X, Y, Z))

        # Pubblica PoseArray
        self.pub_poses.publish(pose_array)

        # Pubblica MarkerArray per lo skeleton
        self.publish_skeleton_markers(points_3d, msg.header.stamp)

        self.get_logger().info(
            f"Published {len(pose_array.poses)} 3D skeleton points"
        )

    # ----------------------- MARKER BUILDER -------------------------

    def publish_skeleton_markers(self, pts, stamp):
        markers = MarkerArray()

        # Marker per le giunzioni (sfere)
        joint_marker = Marker()
        joint_marker.header.frame_id = "camera_color_optical_frame"
        joint_marker.header.stamp = stamp
        joint_marker.ns = "skeleton_joints"
        joint_marker.id = 0
        joint_marker.type = Marker.SPHERE_LIST
        joint_marker.action = Marker.ADD
        joint_marker.scale.x = 0.03
        joint_marker.scale.y = 0.03
        joint_marker.scale.z = 0.03
        joint_marker.color.r = 1.0
        joint_marker.color.g = 0.4
        joint_marker.color.b = 0.1
        joint_marker.color.a = 1.0

        for p in pts:
            if p is None:
                continue
            x, y, z = p
            joint_marker.points.append(Point(x=x, y=y, z=z))

        markers.markers.append(joint_marker)

        # Marker per le "ossa" (linee tra joint)
        bone_marker = Marker()
        bone_marker.header.frame_id = "camera_color_optical_frame"
        bone_marker.header.stamp = stamp
        bone_marker.ns = "skeleton_bones"
        bone_marker.id = 1
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.015
        bone_marker.color.r = 0.0
        bone_marker.color.g = 0.9
        bone_marker.color.b = 0.9
        bone_marker.color.a = 1.0

        for a, b in self.edges:
            if a < len(pts) and b < len(pts) and pts[a] and pts[b]:
                pa = Point(x=pts[a][0], y=pts[a][1], z=pts[a][2])
                pb = Point(x=pts[b][0], y=pts[b][1], z=pts[b][2])

            bone_marker.points.append(Point(x=pts[a][0], y=pts[a][1], z=pts[a][2]))
            bone_marker.points.append(Point(x=pts[b][0], y=pts[b][1], z=pts[b][2]))


        markers.markers.append(bone_marker)

        self.pub_skeleton.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = YoloSkeletonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
