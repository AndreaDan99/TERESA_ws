#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO


# ============================================================
#                Kalman Filter 3D
# ============================================================

class Kalman3D:
    def __init__(self, dt=1/30, q=0.2, r=0.02, p0=1.0):
        self.dt = float(dt)

        self.x = np.zeros((6,1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * p0

        self.F = np.eye(6, dtype=np.float64)
        self.F[0,3] = self.F[1,4] = self.F[2,5] = self.dt

        self.H = np.zeros((3,6), dtype=np.float64)
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0

        self.Q_base = np.eye(6, dtype=np.float64) * q
        self.Q = self.Q_base.copy()
        self.R = np.eye(3, dtype=np.float64) * r

        self.initialized = False

    def predict(self, vel_damping=1.0):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[3:,0] *= float(vel_damping)

    def update(self, z):
        z = np.asarray(z, dtype=np.float64).reshape(3,1)

        if not self.initialized:
            self.x[0:3] = z
            self.x[3:] = 0.0
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_position(self):
        if not self.initialized:
            return None
        return self.x[0:3,0].copy()

    def set_position(self, p):
        self.x[0:3,0] = np.asarray(p, dtype=np.float64).reshape(3)


# ============================================================
#                       Skeleton Node
# ============================================================
def torso_length_constraint(pts, visible, L_ref, stiffness=0.35):
    if L_ref is None:
        return pts

    idx = [5, 6, 11, 12]
    if any(pts[i] is None for i in idx):
        return pts

    # se tutti visibili → NON fare nulla
    if all(visible[i] for i in idx):
        return pts

    sh_mid = 0.5 * (pts[5] + pts[6])
    hip_mid = 0.5 * (pts[11] + pts[12])

    v = sh_mid - hip_mid
    dist = np.linalg.norm(v)
    if dist < 1e-6:
        return pts

    v_corr = (v / dist) * L_ref
    target_sh_mid = hip_mid + v_corr

    delta = target_sh_mid - sh_mid

    # muovi SOLO le spalle (le anche restano ancorate)
    pts[5] += stiffness * delta
    pts[6] += stiffness * delta

    return pts

class YoloSkeletonNodeStable(Node):

    def __init__(self):
        super().__init__("yolo_skeleton_node_stable")

        self.declare_parameter("model_path", "yolov8n-pose.pt")
        self.declare_parameter("conf_thr", 0.3)
        self.declare_parameter("vel_damping", 0.6)
        self.declare_parameter("max_depth_m", 3.0)

        self.conf_thr = float(self.get_parameter("conf_thr").value)
        self.vel_damping = float(self.get_parameter("vel_damping").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.model = YOLO(self.get_parameter("model_path").value)
        self.bridge = CvBridge()

        self.sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.cb_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.cb_depth, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
        )

        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10
        )
 
        self.depth_img = None
        self.cam_info = None

        self.num_joints = 17
        self.torso_len_ref = None
        self.kf = [Kalman3D() for _ in range(self.num_joints)]

        TORSO = {5, 6, 11, 12}
        ARMS  = {7, 8, 9, 10}
        LEGS  = {13, 14, 15, 16}
        NOSE  = {0}

        for i, kf in enumerate(self.kf):

            # Torso → molto stabile
            if i in TORSO:
                kf.Q *= 0.7
                kf.R *= 0.7

            # Braccia → più libertà
            elif i in ARMS:
                kf.Q *= 1.2
                kf.R *= 1.2

            # Gambe → predizione più morbida
            elif i in LEGS:
                kf.Q *= 1.4
                kf.R *= 1.3

            # Naso / viso → molto rumoroso
            elif i in NOSE:
                kf.Q *= 1.8
                kf.R *= 1.8

        self.KNEE_MIN_DEG = 30.0
        self.KNEE_MAX_DEG = 175.0

        self.visible = [False]*self.num_joints
        self.missing_count = [0] * self.num_joints

        self.get_logger().info("✅ YOLO skeleton node (visible vs predicted) ready")

    def adaptive_Q(self, kf, joint_idx):
        Q = kf.Q_base.copy()

        miss = self.missing_count[joint_idx]
        time_factor = min(1.0 + 0.15 * miss, 3.0)

        if joint_idx in {5, 6, 11, 12}:          # torso
            part_factor = 0.7
        elif joint_idx in {7, 8, 9, 10}:         # arms
            part_factor = 1.2
        elif joint_idx in {13, 14, 15, 16}:      # legs
            part_factor = 1.4
        elif joint_idx == 0:                     # nose
            part_factor = 1.8
        else:
            part_factor = 1.0

        kf.Q = Q * time_factor * part_factor

        # Skeleton edges (COCO)
        self.edges = [
            (0,1),(0,2),(1,3),(2,4),
            (5,6),
            (5,7),(7,9),
            (6,8),(8,10),
            (11,12),
            (11,13),(13,15),
            (12,14),(14,16),
            (5,11),(6,12),
        ]

       

    # ============================================================

    def cb_info(self, msg):
        self.cam_info = msg

    def cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def robust_depth(self, u, v, win=5):
        h, w = self.depth_img.shape
        r = win // 2
        patch = self.depth_img[
            max(0,v-r):min(h,v+r+1),
            max(0,u-r):min(w,u+r+1)
        ]
        patch = patch[patch > 0]
        if patch.size < 6:
            return None
        return float(np.median(patch)) * 0.001

    def knee_angle_ok(self, hip, knee, ankle):
        v1 = hip - knee
        v2 = ankle - knee
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return True
        c = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        ang = np.degrees(np.arccos(c))
        return self.KNEE_MIN_DEG <= ang <= self.KNEE_MAX_DEG

    # ============================================================

    def cb_color(self, msg):
        if self.depth_img is None or self.cam_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.model(img, verbose=False)

        self.visible = [False]*self.num_joints


        # ---------- NO PERSON / NO KEYPOINTS ----------
        if len(res) == 0 or res[0].keypoints is None or res[0].keypoints.xy is None:
            self.predict_only(msg.header.stamp)
            return

        kp_xy = res[0].keypoints.xy
        if kp_xy.shape[0] == 0:   # nessuna persona rilevata
            self.predict_only(msg.header.stamp)
            return

        kp = kp_xy[0].cpu().numpy()
        
        conf = res[0].keypoints.conf
        if conf is not None:
            conf = conf[0].cpu().numpy()

        fx, fy, cx, cy = (
            self.cam_info.k[0],
            self.cam_info.k[4],
            self.cam_info.k[2],
            self.cam_info.k[5]
        )

        pts = [None]*self.num_joints

        for i in range(self.num_joints):
            # Non predire occhi / orecchie
            if i in {1, 2, 3, 4}:
                continue

            if conf is not None and conf[i] < self.conf_thr:
                continue

            u, v = int(kp[i][0]), int(kp[i][1])
            d = self.robust_depth(u, v)
            if d is None or d > self.max_depth_m:
                continue

            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            Z = d

            meas = np.array([X, Y, Z], dtype=np.float64)

            self.kf[i].predict(1.0)

            if i == 13 and not self.visible[13]:
                if pts[11] is not None and pts[15] is not None:
                    if not self.knee_angle_ok(pts[11], meas, pts[15]):
                        continue

            if i == 14:  # right knee
                if pts[12] is not None and pts[16] is not None:
                    if not self.knee_angle_ok(pts[12], meas, pts[16]):
                        self.kf[i].Q *= 0.3
                        continue

            # ---------- GATING ----------
            if self.kf[i].initialized:
                pred = self.kf[i].get_position()
                sigma = np.sqrt(np.trace(self.kf[i].P[0:3,0:3]))
                if np.linalg.norm(meas - pred) < 2.5 * sigma:
                    self.kf[i].update(meas)
            else:
                self.kf[i].update(meas)

            self.visible[i] = True

        for i in range(self.num_joints):
            if self.visible[i]:
                self.missing_count[i] = 0
            else:
                self.missing_count[i] += 1

        # Predict missing joints
        for i in range(self.num_joints):
            if not self.visible[i]:
                self.adaptive_Q(self.kf[i], i)
                self.kf[i].predict(self.vel_damping)
            else:
                self.kf[i].Q = self.kf[i].Q_base.copy()

            pts[i] = self.kf[i].get_position()

        if (
            pts[5] is not None and pts[6] is not None and
            pts[11] is not None and pts[12] is not None
        ):
            sh_mid = 0.5 * (pts[5] + pts[6])
            hip_mid = 0.5 * (pts[11] + pts[12])
            L = np.linalg.norm(sh_mid - hip_mid)

            if self.torso_len_ref is None:
                self.torso_len_ref = L
            else:
                # aggiornamento lento
                self.torso_len_ref = 0.98 * self.torso_len_ref + 0.02 * L
        # Vincolo morbido torso

        pts = torso_length_constraint(
            pts,
            self.visible,
            self.torso_len_ref,
            stiffness=0.35
        )

        # -------------------------------
        # Vincolo morbido NASO → spalle
        # -------------------------------
        # Applica SOLO se il naso è predetto (non visibile)
        if (
            pts[0] is not None and
            pts[5] is not None and
            pts[6] is not None and
            not self.visible[0]
        ):
            sh_mid = 0.5 * (pts[5] + pts[6])
            pts[0] = pts[0] + 0.55 * (sh_mid - pts[0])

        # Publish
        self.publish_all(pts, msg.header.stamp)

    # ============================================================

    def predict_only(self, stamp):
        pts = []
        for i, k in enumerate(self.kf):
            # Occhi/orecchie: solo se confidence bassissima
            if i in {1, 2, 3, 4}:
                continue
            if k.initialized:
                self.adaptive_Q(k, i)
                k.predict(self.vel_damping)
                k.Q = k.Q_base.copy()
                pts.append(k.get_position())
            else:
                pts.append(None)
        self.visible = [False]*self.num_joints
        self.publish_all(pts, stamp)

    # ============================================================

    def publish_all(self, pts, stamp):
        # ---------- PoseArray ----------
        pa = PoseArray()
        pa.header.frame_id = "camera_color_optical_frame"
        pa.header.stamp = stamp

        for p in pts:
            pose = Pose()
            if p is None:
                pose.position.x = pose.position.y = pose.position.z = float("nan")
            else:
                pose.position.x, pose.position.y, pose.position.z = p
            pose.orientation.w = 1.0
            pa.poses.append(pose)

        self.pub_poses.publish(pa)

        # ---------- Markers ----------
        ma = MarkerArray()

        # Visible joints (RED)
        j_vis = Marker()
        j_vis.header = pa.header
        j_vis.ns = "joints_visible"
        j_vis.id = 0
        j_vis.type = Marker.SPHERE_LIST
        j_vis.scale.x = j_vis.scale.y = j_vis.scale.z = 0.03
        j_vis.color.r = 1.0
        j_vis.color.a = 1.0

        # Predicted joints (BLUE)
        j_pred = Marker()
        j_pred.header = pa.header
        j_pred.ns = "joints_predicted"
        j_pred.id = 1
        j_pred.type = Marker.SPHERE_LIST
        j_pred.scale.x = j_pred.scale.y = j_pred.scale.z = 0.03
        j_pred.color.b = 1.0
        j_pred.color.a = 1.0

        for i, p in enumerate(pts):
            if p is None:
                continue
            pt = Point(x=p[0], y=p[1], z=p[2])
            if self.visible[i]:
                j_vis.points.append(pt)
            else:
                j_pred.points.append(pt)

        ma.markers.append(j_vis)
        ma.markers.append(j_pred)

        # Bones (GREEN)
        b = Marker()
        b.header = pa.header
        b.ns = "bones"
        b.id = 2
        b.type = Marker.LINE_LIST
        b.scale.x = 0.015
        b.color.g = 1.0
        b.color.a = 1.0

        for a, c in self.edges:
            if pts[a] is not None and pts[c] is not None:
                b.points.append(Point(x=pts[a][0], y=pts[a][1], z=pts[a][2]))
                b.points.append(Point(x=pts[c][0], y=pts[c][1], z=pts[c][2]))

        ma.markers.append(b)

        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = YoloSkeletonNodeStable()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()