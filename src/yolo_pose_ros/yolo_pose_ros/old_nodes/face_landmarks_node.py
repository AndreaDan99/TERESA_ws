import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

import numpy as np
import cv2
import mediapipe as mp


class FaceLandmarksNode(Node):
    def __init__(self):
        super().__init__("face_landmarks_node")

        self.get_logger().info("Loading MediaPipe Face Mesh...")

        # MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,          # punti in più su occhi/labbra
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.bridge = CvBridge()

        # --- Sottoscrizioni RealSense ---
        # Colore
        self.sub_color = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10,
        )

        # Depth ALLINEATA al colore (come prima)
        self.sub_depth = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )

        # Info camera (matrice K)
        self.sub_info = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.info_callback,
            10,
        )

        self.depth_image = None          # uint16 in mm
        self.color_info = None           # CameraInfo

        # Publisher 3D landmark
        self.pub_face_3d = self.create_publisher(
            PoseArray, "/face_landmarks/points_3d", 10
        )

    # --------- CALLBACKS ---------

    def info_callback(self, msg: CameraInfo):
        self.color_info = msg

    def depth_callback(self, msg: Image):
        # depth: uint16, mm
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def color_callback(self, msg: Image):
        # Aspetto depth + camera_info
        if self.depth_image is None or self.color_info is None:
            return

        # BGR -> RGB per MediaPipe
        color_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = color_rgb.shape

        # Inferenza Face Mesh
        results = self.face_mesh.process(color_rgb)

        pose_array = PoseArray()
        # Mettiamo il frame della CAMERA COLORE, così coincide con la pointcloud
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = "camera_color_optical_frame"

        if not results.multi_face_landmarks:
            # Nessuna faccia trovata -> pubblichiamo array vuoto
            self.pub_face_3d.publish(pose_array)
            return

        # Prendiamo solo la prima faccia
        face_landmarks = results.multi_face_landmarks[0].landmark

        # Intrinseci camera
        fx = self.color_info.k[0]
        fy = self.color_info.k[4]
        cx = self.color_info.k[2]
        cy = self.color_info.k[5]

        # Per ogni landmark 2D -> punto 3D via depth
        valid_points = 0

        for lm in face_landmarks:
            # lm.x, lm.y sono normalizzati [0,1]
            u = int(lm.x * w)
            v = int(lm.y * h)

            # Bound check
            if u < 0 or v < 0:
                continue
            if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
                continue

            depth_mm = int(self.depth_image[v, u])

            # Depth 0 = nessuna misura
            if depth_mm <= 0:
                continue

            depth = float(depth_mm) * 0.001  # mm -> m

            # Back-projection in camera_color_optical_frame
            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth

            p = Pose()
            p.position.x = float(X)
            p.position.y = float(Y)
            p.position.z = float(Z)
            # orientazione neutra
            p.orientation.w = 1.0

            pose_array.poses.append(p)
            valid_points += 1

        self.pub_face_3d.publish(pose_array)
        self.get_logger().info(f"Published {valid_points} 3D face landmarks")


def main(args=None):
    rclpy.init(args=args)
    node = FaceLandmarksNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
