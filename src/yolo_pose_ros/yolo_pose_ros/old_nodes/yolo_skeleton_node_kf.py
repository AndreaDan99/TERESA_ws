import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO

# ============================================================
#           Simple 3D Kalman Filter (per keypoint)
# ============================================================

class Kalman3D:
    def __init__(self, dt=1/30, q=0.02, r=0.01):
        self.dt = dt

        # Stato: [x,y,z,vx,vy,vz]
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 1.0

        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self.Q = np.eye(6) * q
        self.R = np.eye(3) * r

        self.initialized = False

    def predict(self):
        # x_k|k-1 = F x_{k-1|k-1}
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z = z.reshape(3, 1)

        if not self.initialized:
            # initialize state directly from measurement
            self.x[0:3] = z
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def get_position(self):
        # restituisce sempre un 3D np.array (anche se non inizializzato)
        return self.x[0:3].flatten()


# ============================================================
#                       Skeleton Node
# ============================================================

class YoloSkeletonNodeKF(Node):
    def __init__(self):
        super().__init__("yolo_skeleton_kf_node")

        self.get_logger().info("Loading YOLO Pose model...")
        self.model = YOLO("yolov8n-pose.pt")

        self.bridge = CvBridge()
        self.initialized = False  # almeno UNA persona vista una volta

        # Subscribe
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

        self.num_joints = 17

        # 17 Kalman filters (uno per joint)
        self.kf = [Kalman3D() for _ in range(self.num_joints)]

        # Conteggio di quanti frame consecutivi il joint è "mancante"
        self.miss_count = [0 for _ in range(self.num_joints)]
        self.max_miss_frames = 10  # dopo 10 frame senza misura → freeza

        # Publishers
        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10
        )

        # Edges di skeleton completi (testa–spalle–busto–bacino)
        # Indici COCO (YOLOv8 pose, 17 keypoints)
        self.edges = [
            (0, 1), (0, 2),        # nose → eyes
            (1, 3), (2, 4),        # eyes → ears
            (5, 6),                # shoulders
            (5, 7), (7, 9),        # left arm
            (6, 8), (8, 10),       # right arm
            (11, 12),              # hips (bacino)
            (11, 13), (13, 15),    # left leg
            (12, 14), (14, 16),    # right leg
            (0, 5), (0, 6),        # nose → shoulders
            (5, 11), (6, 12)       # shoulders → hips (collega busto/leg)
        ]

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------

    def info_callback(self, msg):
        self.color_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def color_callback(self, msg):
        if self.depth_image is None or self.color_info is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(color_img, verbose=False)

        # ------------------------------------------
        # SAFE KEYPOINT PARSING - FIRST PERSON WAIT
        # ------------------------------------------
        if len(results) == 0:
            # molto raro ma per sicurezza
            if not self.initialized:
                self.get_logger().warn("Waiting for first person...")
            return

        # Se YOLO non ritorna keypoints → nessuna persona rilevata
        if results[0].keypoints is None:
            if not self.initialized:
                self.get_logger().warn("Waiting for first person...")
                return
            else:
                # persona scomparsa, ma sistema già inizializzato:
                # opzionalmente potresti fare solo predict+freeze qui
                return

        kp_np = results[0].keypoints.xy.cpu().numpy()

        # Se YOLO ha trovato 0 persone
        if kp_np.shape[0] == 0:
            if not self.initialized:
                self.get_logger().warn("Waiting for first person...")
                return
            else:
                # nessuna persona in questo frame → nessun update
                return

        # --------------------------------------------------
        # Se arrivi qui → c’è almeno una persona
        # --------------------------------------------------

        kp = kp_np[0]  # prima persona trovata

        # Se è la prima volta → inizializza flag globale
        if not self.initialized:
            self.initialized = True
            self.get_logger().info(
                "First person detected. Kalman Filters are now active."
            )

        fx = self.color_info.k[0]
        fy = self.color_info.k[4]
        cx = self.color_info.k[2]
        cy = self.color_info.k[5]

        points_3d = []

        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_color_optical_frame"
        pose_array.header.stamp = msg.header.stamp

        # ----------------- Convert each keypoint + KF -------------------
        for j, (u, v) in enumerate(kp):
            u, v = int(u), int(v)

            pos = None

            # Controllo bounds immagine
            in_bounds = (
                0 <= u < self.depth_image.shape[1]
                and 0 <= v < self.depth_image.shape[0]
            )

            if not in_bounds:
                # Nessuna osservazione valida per questo joint
                pos = self._handle_missing_measurement(j)
            else:
                d = float(self.depth_image[v, u])  # profondità in mm
                if d <= 0:
                    # Depth non valida
                    pos = self._handle_missing_measurement(j)
                else:
                    # Misura valida → reset contatore mancate
                    self.miss_count[j] = 0
                    depth = d * 0.001  # mm → metri

                    X = (u - cx) * depth / fx
                    Y = (v - cy) * depth / fy
                    Z = depth

                    # Kalman: predict + update
                    self.kf[j].predict()
                    self.kf[j].update(np.array([X, Y, Z]))
                    pos = self.kf[j].get_position()

            points_3d.append(pos)

            if pos is not None:
                p = Pose()
                p.position.x = float(pos[0])
                p.position.y = float(pos[1])
                p.position.z = float(pos[2])
                p.orientation.w = 1.0
                pose_array.poses.append(p)

        # Pubblica pose array (solo joint validi)
        self.pub_poses.publish(pose_array)

        self.get_logger().info(
            f"Published {len(pose_array.poses)} 3D skeleton points"
        )

        # Publish markers
        self.publish_skeleton(points_3d, msg.header.stamp)

    # ---------------------------------------------------------
    # Gestione dei joint con misura mancante
    # ---------------------------------------------------------

    def _handle_missing_measurement(self, j: int):
        """
        Gestisce il caso in cui un joint non ha una misura valida in questo frame.
        - per i primi 'max_miss_frames' frame mancanti: fa ancora una predict
        - oltre: freeza (usa solo l'ultima posizione stimata, senza predict)
        - se il KF non è mai stato inizializzato: restituisce None
        """
        kf = self.kf[j]

        if not kf.initialized:
            # Non abbiamo mai avuto una misura per questo joint
            return None

        # Abbiamo già avuto almeno una misura in passato
        if self.miss_count[j] < self.max_miss_frames:
            # per qualche frame (es. 10) continuiamo a predire
            kf.predict()
            self.miss_count[j] += 1
        else:
            # oltre la soglia → non facciamo più predict,
            # usiamo l’ultima posizione senza farla “esplodere”
            pass

        return kf.get_position()

    # ---------------------------------------------------------
    # Marker publisher
    # ---------------------------------------------------------

    def publish_skeleton(self, pts, stamp):
        markers = MarkerArray()

        # Joint marker (sfere)
        joint = Marker()
        joint.header.frame_id = "camera_color_optical_frame"
        joint.header.stamp = stamp
        joint.ns = "joints"
        joint.id = 0
        joint.type = Marker.SPHERE_LIST
        joint.action = Marker.ADD
        joint.scale.x = joint.scale.y = joint.scale.z = 0.03
        joint.color.r = 1.0
        joint.color.g = 0.4
        joint.color.b = 0.1
        joint.color.a = 1.0

        for p in pts:
            if p is None:
                continue
            joint.points.append(
                Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            )

        markers.markers.append(joint)

        # Bones (linee tra joint)
        bones = Marker()
        bones.header.frame_id = "camera_color_optical_frame"
        bones.header.stamp = stamp
        bones.ns = "bones"
        bones.id = 1
        bones.type = Marker.LINE_LIST
        bones.action = Marker.ADD
        bones.scale.x = 0.015
        bones.color.r = 0.0
        bones.color.g = 0.9
        bones.color.b = 0.9
        bones.color.a = 1.0

        for a, b in self.edges:
            # Safety: controlla che esistano e non siano None
            if a >= len(pts) or b >= len(pts):
                continue
            if pts[a] is None or pts[b] is None:
                continue

            pa = pts[a]
            pb = pts[b]
            bones.points.append(
                Point(x=float(pa[0]), y=float(pa[1]), z=float(pa[2]))
            )
            bones.points.append(
                Point(x=float(pb[0]), y=float(pb[1]), z=float(pb[2]))
            )

        markers.markers.append(bones)

        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSkeletonNodeKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
