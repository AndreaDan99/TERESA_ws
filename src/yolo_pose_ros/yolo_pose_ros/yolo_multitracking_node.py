#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

from ultralytics import YOLO


# ============================================================
#                Kalman Filter 3D (per joint / per centro)
#   State: [x,y,z,vx,vy,vz]
# ============================================================

class Kalman3D:
    def __init__(self, dt=1/30, q=0.2, r=0.02, p0=1.0):
        self.dt = float(dt)

        self.x = np.zeros((6, 1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * float(p0)

        self.F = np.eye(6, dtype=np.float64)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt

        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.Q = np.eye(6, dtype=np.float64) * float(q)
        self.R = np.eye(3, dtype=np.float64) * float(r)

        self.initialized = False

    def predict(self, vel_damping=1.0):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        d = float(vel_damping)
        self.x[3, 0] *= d
        self.x[4, 0] *= d
        self.x[5, 0] *= d

    def update(self, z_xyz):
        z = np.asarray(z_xyz, dtype=np.float64).reshape(3, 1)
        if not self.initialized:
            self.x[0:3] = z
            self.x[3:6] = 0.0
            self.initialized = True
            return

        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ self.H) @ self.P

    def get_position(self):
        if not self.initialized:
            return None
        return self.x[0:3, 0].copy()

    def set_position(self, p_xyz):
        p = np.asarray(p_xyz, dtype=np.float64).reshape(3)
        self.x[0, 0] = p[0]
        self.x[1, 0] = p[1]
        self.x[2, 0] = p[2]


# ============================================================
#                  Bone constraints (soft)
# ============================================================

def apply_bone_constraints(pts, edges, lengths, iters=2, stiffness=0.9):
    """In-place soft projection: keep segment lengths near target."""
    iters = int(max(0, iters))
    s = float(stiffness)
    if iters <= 0 or s <= 0.0:
        return pts

    for _ in range(iters):
        for (a, b) in edges:
            L = lengths.get((a, b), None)
            if L is None:
                continue
            if pts[a] is None or pts[b] is None:
                continue

            pa = pts[a]
            pb = pts[b]
            d = pb - pa
            dist = float(np.linalg.norm(d))
            if dist < 1e-9:
                continue

            u = d / dist
            err = dist - float(L)
            corr = 0.5 * s * err * u

            pts[a] = pa + corr
            pts[b] = pb - corr

    return pts


# ============================================================
#                  COCO skeleton (17 keypoints)
# ============================================================

# indices (COCO-17 in YOLOv8 pose)
NOSE = 0
L_EYE = 1
R_EYE = 2
L_EAR = 3
R_EAR = 4
L_SHOULDER = 5
R_SHOULDER = 6
L_ELBOW = 7
R_ELBOW = 8
L_WRIST = 9
R_WRIST = 10
L_HIP = 11
R_HIP = 12
L_KNEE = 13
R_KNEE = 14
L_ANKLE = 15
R_ANKLE = 16

EDGES = [
    (NOSE, L_EYE), (NOSE, R_EYE),
    (L_EYE, L_EAR), (R_EYE, R_EAR),

    (L_SHOULDER, R_SHOULDER),

    (L_SHOULDER, L_ELBOW), (L_ELBOW, L_WRIST),
    (R_SHOULDER, R_ELBOW), (R_ELBOW, R_WRIST),

    (L_HIP, R_HIP),

    (L_HIP, L_KNEE), (L_KNEE, L_ANKLE),
    (R_HIP, R_KNEE), (R_KNEE, R_ANKLE),

    (L_SHOULDER, L_HIP),
    (R_SHOULDER, R_HIP),
]

DEFAULT_BONE_LENGTHS = {
    (L_SHOULDER, L_HIP): 0.45,
    (R_SHOULDER, R_HIP): 0.45,

    (L_HIP, L_KNEE): 0.42,
    (L_KNEE, L_ANKLE): 0.43,
    (R_HIP, R_KNEE): 0.42,
    (R_KNEE, R_ANKLE): 0.43,

    (L_SHOULDER, L_ELBOW): 0.30,
    (L_ELBOW, L_WRIST): 0.27,
    (R_SHOULDER, R_ELBOW): 0.30,
    (R_ELBOW, R_WRIST): 0.27,
}


# ============================================================
#                  Tracker (one person)
# ============================================================

class PersonTrack:
    def __init__(self, track_id: int, dt: float, q: float, r: float):
        self.id = int(track_id)

        self.num_joints = 17
        self.kf_joints = [Kalman3D(dt=dt, q=q, r=r) for _ in range(self.num_joints)]
        self.kf_center = Kalman3D(dt=dt, q=q, r=r)

        self.visible_joints = [False] * self.num_joints  # per-frame
        self.age = 0
        self.missed = 0

        self.confirm_hits = 0
        self.confirmed = False

        self.last_stamp = None

    def predict_all(self, vel_damping: float):
        self.age += 1
        self.missed += 1

        self.kf_center.predict(vel_damping=vel_damping)
        for kf in self.kf_joints:
            if kf.initialized:
                kf.predict(vel_damping=vel_damping)

    def update_from_detection(self, joints_meas, joints_visible, center_meas,
                              vel_damping_when_missing: float,
                              gating_sigma_mult: float):
        """
        joints_meas: list len17 of np.array(3,) or None
        joints_visible: list len17 bool
        center_meas: np.array(3,) or None
        """
        self.visible_joints = list(joints_visible)
        self.missed = 0
        self.last_stamp = None

        # update center
        self.kf_center.predict(vel_damping=1.0)
        if center_meas is not None:
            if self.kf_center.initialized:
                pred = self.kf_center.get_position()
                sigma = float(np.sqrt(np.trace(self.kf_center.P[0:3, 0:3])))
                if sigma < 1e-6:
                    sigma = 1e-6
                if np.linalg.norm(center_meas - pred) < gating_sigma_mult * sigma:
                    self.kf_center.update(center_meas)
            else:
                self.kf_center.update(center_meas)

        # update joints
        for i in range(self.num_joints):
            kf = self.kf_joints[i]
            if joints_visible[i] and joints_meas[i] is not None:
                meas = joints_meas[i]
                kf.predict(vel_damping=1.0)

                if kf.initialized:
                    pred = kf.get_position()
                    sigma = float(np.sqrt(np.trace(kf.P[0:3, 0:3])))
                    if sigma < 1e-6:
                        sigma = 1e-6
                    if np.linalg.norm(meas - pred) < gating_sigma_mult * sigma:
                        kf.update(meas)
                else:
                    kf.update(meas)
            else:
                if kf.initialized:
                    kf.predict(vel_damping=vel_damping_when_missing)

        # confirmation logic
        if center_meas is not None:
            self.confirm_hits += 1
        if (not self.confirmed) and (self.confirm_hits >= 2):
            self.confirmed = True

    def get_joints(self):
        pts = []
        for kf in self.kf_joints:
            pts.append(kf.get_position())
        return pts

    def get_center(self):
        return self.kf_center.get_position()


# ============================================================
#                  Multi-tracker manager
# ============================================================

def _euclid(a, b):
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))

def compute_torso_center(joints_3d):
    """Use shoulders+hips average if available; else average of available joints."""
    idx = [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]
    good = [joints_3d[i] for i in idx if joints_3d[i] is not None]
    if len(good) >= 2:
        return np.mean(np.stack(good, axis=0), axis=0)
    good2 = [p for p in joints_3d if p is not None]
    if len(good2) == 0:
        return None
    return np.mean(np.stack(good2, axis=0), axis=0)


class TrackerManager:
    def __init__(self, dt, q, r,
                 match_dist_m=0.60,
                 max_missed=15):
        self.dt = float(dt)
        self.q = float(q)
        self.r = float(r)

        self.match_dist_m = float(match_dist_m)
        self.max_missed = int(max_missed)

        self.next_id = 0
        self.tracks = []  # list[PersonTrack]

    def _new_track(self):
        t = PersonTrack(self.next_id, dt=self.dt, q=self.q, r=self.r)
        self.next_id += 1
        self.tracks.append(t)
        return t

    def predict_all(self, vel_damping):
        for t in self.tracks:
            t.predict_all(vel_damping=vel_damping)

    def prune_dead(self):
        self.tracks = [t for t in self.tracks if t.missed <= self.max_missed]

    def associate_and_update(self, detections, vel_damping_when_missing, gating_sigma_mult):
        """
        detections: list of dict:
          {
            "joints": [np.array(3,) or None]*17,
            "visible": [bool]*17,
            "center": np.array(3,) or None
          }
        """

        # If no active tracks -> create one per detection
        if len(self.tracks) == 0:
            for det in detections:
                t = self._new_track()
                t.update_from_detection(det["joints"], det["visible"], det["center"],
                                        vel_damping_when_missing, gating_sigma_mult)
            return

        # Build cost matrix (center distance)
        track_centers = [t.get_center() for t in self.tracks]
        det_centers = [d["center"] for d in detections]

        # Tracks without center yet are harder: set big cost.
        cost = np.full((len(self.tracks), len(detections)), 1e9, dtype=np.float64)
        for i, tc in enumerate(track_centers):
            for j, dc in enumerate(det_centers):
                if tc is None or dc is None:
                    continue
                cost[i, j] = _euclid(tc, dc)

        # Greedy assignment with threshold (simple and stable)
        assigned_tracks = set()
        assigned_dets = set()

        while True:
            if cost.size == 0:
                break
            i, j = np.unravel_index(np.argmin(cost), cost.shape)
            best = float(cost[i, j])
            if best > self.match_dist_m:
                break

            # assign
            assigned_tracks.add(i)
            assigned_dets.add(j)
            # invalidate row/col
            cost[i, :] = 1e9
            cost[:, j] = 1e9

            # update track i with detection j
            t = self.tracks[i]
            det = detections[j]
            t.update_from_detection(det["joints"], det["visible"], det["center"],
                                    vel_damping_when_missing, gating_sigma_mult)

        # Unassigned detections -> new tracks
        for j, det in enumerate(detections):
            if j in assigned_dets:
                continue
            # Create new track only if it has a reasonable center
            if det["center"] is None:
                continue
            t = self._new_track()
            t.update_from_detection(det["joints"], det["visible"], det["center"],
                                    vel_damping_when_missing, gating_sigma_mult)

        # Tracks not assigned will remain predicted (already predicted in predict_all)

    def get_confirmed_tracks(self):
        # In emergenza ti conviene vedere anche non-confirmed,
        # ma per evitare “fantasmi” puoi filtrare qui.
        return self.tracks


# ============================================================
#                       Main ROS2 Node
# ============================================================

class YoloSkeletonMultiTracker(Node):
    def __init__(self):
        super().__init__("yolo_skeleton_multi_tracker")

        # ---------------- Params ----------------
        self.declare_parameter("model_path", "yolov8n-pose.pt")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.declare_parameter("dt", 1.0 / 30.0)
        self.declare_parameter("q", 0.2)
        self.declare_parameter("r", 0.02)

        self.declare_parameter("conf_thr", 0.30)
        self.declare_parameter("conf_face_thr", 0.60)
        self.declare_parameter("vel_damping", 0.60)
        self.declare_parameter("max_depth_m", 3.0)

        self.declare_parameter("match_dist_m", 0.60)
        self.declare_parameter("max_missed_frames", 15)

        self.declare_parameter("constraint_iters", 2)
        self.declare_parameter("constraint_stiffness", 0.90)
        self.declare_parameter("gating_sigma_mult", 2.5)

        self.model_path = self.get_parameter("model_path").value
        self.frame_id = self.get_parameter("frame_id").value

        self.dt = float(self.get_parameter("dt").value)
        self.q = float(self.get_parameter("q").value)
        self.r = float(self.get_parameter("r").value)

        self.conf_thr = float(self.get_parameter("conf_thr").value)
        self.conf_face_thr = float(self.get_parameter("conf_face_thr").value)
        self.vel_damping = float(self.get_parameter("vel_damping").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.constraint_iters = int(self.get_parameter("constraint_iters").value)
        self.constraint_stiffness = float(self.get_parameter("constraint_stiffness").value)
        self.gating_sigma_mult = float(self.get_parameter("gating_sigma_mult").value)

        # ---------------- YOLO ----------------
        self.get_logger().info(f"Loading YOLO pose: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # ---------------- Subscriptions ----------------
        self.sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.cb_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.cb_depth, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
        )

        # ---------------- Publishers ----------------
        # Nota: per multi-person pubblichiamo SOLO markers (più chiaro).
        # PoseArray rimane, ma pubblica solo il "trk0" (se esiste) per compatibilità con altri nodi.
        self.pub_poses = self.create_publisher(PoseArray, "/human_pose/points_3d", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/human_pose/skeleton_markers", 10)

        self.depth_img = None
        self.cam_info = None

        # ---------------- Tracking ----------------
        self.manager = TrackerManager(
            dt=self.dt, q=self.q, r=self.r,
            match_dist_m=float(self.get_parameter("match_dist_m").value),
            max_missed=int(self.get_parameter("max_missed_frames").value),
        )

        self.edges = list(EDGES)
        self.bone_lengths = dict(DEFAULT_BONE_LENGTHS)

        self.get_logger().info("✅ Multi-tracking node ready (stable trk ids).")

    # ============================================================

    def cb_info(self, msg: CameraInfo):
        self.cam_info = msg

    def cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def robust_depth(self, u, v, win=5):
        if self.depth_img is None:
            return None
        h, w = self.depth_img.shape
        r = win // 2
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = self.depth_img[v0:v1, u0:u1]
        patch = patch[patch > 0]
        if patch.size < 6:
            return None
        return float(np.median(patch)) * 0.001

    def _intrinsics(self):
        fx = float(self.cam_info.k[0])
        fy = float(self.cam_info.k[4])
        cx = float(self.cam_info.k[2])
        cy = float(self.cam_info.k[5])
        return fx, fy, cx, cy

    def _meas_3d(self, u, v, fx, fy, cx, cy):
        u = int(u); v = int(v)
        if u < 0 or v < 0:
            return None
        if v >= self.depth_img.shape[0] or u >= self.depth_img.shape[1]:
            return None
        d = self.robust_depth(u, v)
        if d is None or d <= 0.0 or d > self.max_depth_m:
            return None
        X = (u - cx) * d / fx
        Y = (v - cy) * d / fy
        Z = d
        return np.array([X, Y, Z], dtype=np.float64)

    # ============================================================

    def cb_color(self, msg: Image):
        if self.depth_img is None or self.cam_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img, verbose=False)

        # 1) Predict all tracks first (so they remain stable)
        self.manager.predict_all(vel_damping=self.vel_damping)

        detections = []
        fx, fy, cx, cy = self._intrinsics()

        # 2) Parse detections (multi-person)
        if results is not None and len(results) > 0 and results[0].keypoints is not None:
            kp_xy_all = results[0].keypoints.xy  # (N,17,2)
            kp_conf_all = results[0].keypoints.conf  # (N,17) maybe None

            if kp_xy_all is not None and kp_xy_all.shape[0] > 0:
                N = int(kp_xy_all.shape[0])

                conf_np = None
                if kp_conf_all is not None:
                    conf_np = kp_conf_all.cpu().numpy()  # (N,17)

                kp_xy_np = kp_xy_all.cpu().numpy()  # (N,17,2)

                for n in range(N):
                    joints = [None] * 17
                    visible = [False] * 17

                    for i in range(17):
                        thr = self.conf_face_thr if i < 5 else self.conf_thr
                        if conf_np is not None and float(conf_np[n, i]) < thr:
                            continue

                        u, v = kp_xy_np[n, i]
                        p3 = self._meas_3d(u, v, fx, fy, cx, cy)
                        if p3 is None:
                            continue

                        joints[i] = p3
                        visible[i] = True

                    center = compute_torso_center(joints)
                    if center is None:
                        # niente 3D utile -> skippa (evita trk che esplodono)
                        continue

                    detections.append({
                        "joints": joints,
                        "visible": visible,
                        "center": center
                    })

        # 3) Associate & update
        if len(detections) > 0:
            self.manager.associate_and_update(
                detections,
                vel_damping_when_missing=self.vel_damping,
                gating_sigma_mult=self.gating_sigma_mult
            )

        # 4) Prune dead
        self.manager.prune_dead()

        # 5) Constraints per track (stabilizza mentre mancano joint)
        for trk in self.manager.get_confirmed_tracks():
            pts = trk.get_joints()
            apply_bone_constraints(
                pts, self.edges, self.bone_lengths,
                iters=self.constraint_iters,
                stiffness=self.constraint_stiffness
            )
            # write back into filters (stability)
            for i, p in enumerate(pts):
                if p is not None and trk.kf_joints[i].initialized:
                    trk.kf_joints[i].set_position(p)

        # 6) Publish markers for ALL tracks
        self.publish_markers_all_tracks(stamp=msg.header.stamp)

        # 7) Publish PoseArray for compatibility: trk0 if exists else NaN
        self.publish_posearray_trk0(stamp=msg.header.stamp)

    # ============================================================

    def publish_posearray_trk0(self, stamp):
        # find track with smallest id (usually trk0)
        tracks = self.manager.get_confirmed_tracks()
        if len(tracks) == 0:
            pts = [None] * 17
            vis = [False] * 17
        else:
            t0 = sorted(tracks, key=lambda t: t.id)[0]
            pts = t0.get_joints()
            vis = t0.visible_joints

        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = stamp

        nan = float("nan")
        for p in pts:
            pose = Pose()
            if p is None:
                pose.position.x = nan
                pose.position.y = nan
                pose.position.z = nan
            else:
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pa.poses.append(pose)

        self.pub_poses.publish(pa)

    # ============================================================

    def publish_markers_all_tracks(self, stamp):
        ma = MarkerArray()

        # We also add DELETEALL to avoid stale markers when tracks disappear
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        for trk in self.manager.get_confirmed_tracks():
            pts = trk.get_joints()
            vis = trk.visible_joints

            # Visible joints (RED)
            j_vis = Marker()
            j_vis.header.frame_id = self.frame_id
            j_vis.header.stamp = stamp
            j_vis.ns = f"trk_{trk.id}_joints_visible"
            j_vis.id = 0
            j_vis.type = Marker.SPHERE_LIST
            j_vis.action = Marker.ADD
            j_vis.scale.x = j_vis.scale.y = j_vis.scale.z = 0.03
            j_vis.color.r = 1.0
            j_vis.color.a = 1.0

            # Predicted joints (BLUE)
            j_pred = Marker()
            j_pred.header.frame_id = self.frame_id
            j_pred.header.stamp = stamp
            j_pred.ns = f"trk_{trk.id}_joints_predicted"
            j_pred.id = 1
            j_pred.type = Marker.SPHERE_LIST
            j_pred.action = Marker.ADD
            j_pred.scale.x = j_pred.scale.y = j_pred.scale.z = 0.03
            j_pred.color.b = 1.0
            j_pred.color.a = 1.0

            for i, p in enumerate(pts):
                if p is None:
                    continue
                pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                if i < len(vis) and vis[i]:
                    j_vis.points.append(pt)
                else:
                    j_pred.points.append(pt)

            ma.markers.append(j_vis)
            ma.markers.append(j_pred)

            # Bones (GREEN) – use filtered visibility: if both endpoints exist (either visible or predicted)
            b = Marker()
            b.header.frame_id = self.frame_id
            b.header.stamp = stamp
            b.ns = f"trk_{trk.id}_bones"
            b.id = 2
            b.type = Marker.LINE_LIST
            b.action = Marker.ADD
            b.scale.x = 0.015
            b.color.g = 1.0
            b.color.a = 1.0

            for a, c in self.edges:
                if a >= len(pts) or c >= len(pts):
                    continue
                if pts[a] is None or pts[c] is None:
                    continue
                b.points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=float(pts[a][2])))
                b.points.append(Point(x=float(pts[c][0]), y=float(pts[c][1]), z=float(pts[c][2])))

            ma.markers.append(b)

            # Optional: Track label (TEXT)
            center = trk.get_center()
            if center is not None:
                txt = Marker()
                txt.header.frame_id = self.frame_id
                txt.header.stamp = stamp
                txt.ns = f"trk_{trk.id}_label"
                txt.id = 3
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = float(center[0])
                txt.pose.position.y = float(center[1]) - 0.15
                txt.pose.position.z = float(center[2])
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.08
                txt.color.r = 1.0
                txt.color.g = 1.0
                txt.color.b = 1.0
                txt.color.a = 1.0
                txt.text = f"trk{trk.id}  missed={trk.missed}"
                ma.markers.append(txt)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSkeletonMultiTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()