import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO


class YoloSkeletonSmooth(Node):
    def __init__(self):
        super().__init__("yolo_skeleton_smooth_node")

        self.get_logger().info("Loading YOLO Pose model...")
        self.model = YOLO("yolov8n-pose.pt")

        self.bridge = CvBridge()

        # --- Subscribers ---
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

        # State
        self.depth_image = None
        self.color_info = None

        # --- Publishers ---
        self.pub_poses = self.create_publisher(PoseArray, "/human_pose/points_3d", 10)
        self.pub_skeleton = self.create_publisher(MarkerArray, "/human_pose/skeleton_markers", 10)

        # YOLOv8 keypoints = 17
        self.num_joints = 17

        # EMA smoothing state
        self.prev_pts = [None] * self.num_joints
        self.alpha = 0.45  # smoothing strength

        # Skeleton edges (COCO)
        self.edges = [
            (0, 1), (1, 2), (2, 3), (3, 4),     # head
            (0, 5), (0, 6),
            (5, 6),
            (5, 11), (6, 12),
            (11, 12),

            (5, 7), (7, 9),       # left arm
            (6, 8), (8, 10),      # right arm

            (11, 13), (13, 15),   # left leg
            (12, 14), (14, 16)    # right leg
        ]


    # ----------------------- CALLBACKS -------------------------

    def info_callback(self, msg: CameraInfo):
        self.color_info = msg

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )  # uint16 depth in mm

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

        kp_xy = result.keypoints.xy.cpu().numpy()        # [N,17,2]
        kp_conf_raw = result.keypoints.conf.cpu().numpy()  # maybe [N,17] or [N,17,1]

        if kp_xy.shape[0] == 0:
            return

        kp = kp_xy[0]     # [17,2]

        # ---- CONFIDENCE HANDLING ----
        conf = self._extract_conf(kp_conf_raw)

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = "camera_color_optical_frame"

        points_3d = []
        fx = self.color_info.k[0]
        fy = self.color_info.k[4]
        cx = self.color_info.k[2]
        cy = self.color_info.k[5]

        for j in range(self.num_joints):
            u, v = kp[j]
            u, v = int(u), int(v)

            # Skip low confidence joints
            if conf[j] < 0.45:
                points_3d.append(None)
                continue

            if not (0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]):
                points_3d.append(None)
                continue

            d = float(self.depth_image[v, u])
            if d <= 0:
                points_3d.append(None)
                continue

            depth = d * 0.001
            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth

            # EMA smoothing
            smoothed = self._smooth_point(j, (X, Y, Z))
            points_3d.append(smoothed)

            # Publish pose
            p = Pose()
            p.position.x, p.position.y, p.position.z = smoothed
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self.pub_poses.publish(pose_array)
        self.publish_skeleton_markers(points_3d, msg.header.stamp)


    # ----------------------- CONFIDENCE EXTRACTOR -------------------------

    def _extract_conf(self, kps_conf):
        """Handles YOLO-v8 different confidence shapes safely."""
        try:
            if len(kps_conf.shape) == 3:  # [N,17,1]
                return kps_conf[0, :, 0]

            if len(kps_conf.shape) == 2:  # [N,17]
                return kps_conf[0]

        except:
            pass

        return np.ones(self.num_joints)


    # ----------------------- SMOOTHING -------------------------

    def _smooth_point(self, idx, new_pt):
        """Exponential smoothing for each keypoint."""
        prev = self.prev_pts[idx]

        if prev is None:
            self.prev_pts[idx] = new_pt
            return new_pt

        # EMA
        smoothed = (
            self.alpha * np.array(new_pt)
            + (1 - self.alpha) * np.array(prev)
        )

        smoothed = tuple(smoothed.tolist())
        self.prev_pts[idx] = smoothed
        return smoothed


    # ----------------------- MARKER BUILDER -------------------------

    def publish_skeleton_markers(self, pts, stamp):
        markers = MarkerArray()

        # Joints
        joint_marker = Marker()
        joint_marker.header.frame_id = "camera_color_optical_frame"
        joint_marker.header.stamp = stamp
        joint_marker.ns = "skeleton_joints"
        joint_marker.id = 0
        joint_marker.type = Marker.SPHERE_LIST
        joint_marker.scale.x = joint_marker.scale.y = joint_marker.scale.z = 0.03
        joint_marker.color.r = 1.0
        joint_marker.color.g = 0.3
        joint_marker.color.b = 0.1
        joint_marker.color.a = 1.0

        for p in pts:
            if p is not None:
                joint_marker.points.append(Point(x=p[0], y=p[1], z=p[2]))

        markers.markers.append(joint_marker)

        # Bones
        bone_marker = Marker()
        bone_marker.header.frame_id = "camera_color_optical_frame"
        bone_marker.header.stamp = stamp
        bone_marker.ns = "skeleton_bones"
        bone_marker.id = 1
        bone_marker.type = Marker.LINE_LIST
        bone_marker.scale.x = 0.015
        bone_marker.color.g = 0.8
        bone_marker.color.b = 1.0
        bone_marker.color.a = 1.0

        for a, b in self.edges:
            if a < len(pts) and b < len(pts) and pts[a] and pts[b]:
                bone_marker.points.append(Point(x=pts[a][0], y=pts[a][1], z=pts[a][2]))
                bone_marker.points.append(Point(x=pts[b][0], y=pts[b][1], z=pts[b][2]))

        markers.markers.append(bone_marker)
        self.pub_skeleton.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSkeletonSmooth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
