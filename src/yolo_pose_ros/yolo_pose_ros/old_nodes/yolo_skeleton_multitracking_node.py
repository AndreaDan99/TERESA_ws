#!/usr/bin/env python3
import time
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
#                Kalman Filter 3D
#   state: [x,y,z,vx,vy,vz]
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
#                  Bone constraints
# ============================================================

def apply_bone_constraints(pts, edges, lengths, iters=2, stiffness=0.9):
    """Simple length projection. pts: list of np.array(3,) or None"""
    iters = int(iters)
    stiffness = float(stiffness)
    if iters <= 0 or stiffness <= 0.0:
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
            if dist < 1e-6:
                continue

            u = d / dist
            err = dist - float(L)
            corr = 0.5 * stiffness * err * u

            pts[a] = pa + corr
            pts[b] = pb - corr

    return pts


# ============================================================
#                 COCO indices helpers
# ============================================================

NOSE = 0
L_SHOULDER, R_SHOULDER = 5, 6
L_HIP, R_HIP = 11, 12


def compute_torso_center(pts_3d, visible_mask):
    """Return a robust 3D center for matching. Uses torso anchors if possible, else mean of visible joints."""
    anchors = [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]
    vals = []
    for i in anchors:
        if i < len(pts_3d) and visible_mask[i] and pts_3d[i] is not None:
            vals.append(pts_3d[i])
    if len(vals) >= 2:
        return np.mean(np.stack(vals, axis=0), axis=0)

    # fallback: all visible points
    vals = []
    for i in range(len(pts_3d)):
        if visible_mask[i] and pts_3d[i] is not None:
            vals.append(pts_3d[i])
    if len(vals) >= 3:
        return np.mean(np.stack(vals, axis=0), axis=0)

    return None


# ============================================================
#                    Person tracker
# ============================================================

def _id_color_rgb(track_id: int):
    # deterministic-ish color from id
    rng = np.random.default_rng(seed=12345 + int(track_id) * 97)
    c = rng.random(3)
    c = 0.2 + 0.8 * c  # avoid too dark
    return float(c[0]), float(c[1]), float(c[2])


class PersonTracker:
    def __init__(self, track_id: int, num_joints: int, dt: float, q: float, r: float):
        self.id = int(track_id)
        self.num_joints = int(num_joints)
        self.kf = [Kalman3D(dt=dt, q=q, r=r) for _ in range(self.num_joints)]
        self.visible = [False] * self.num_joints
        self.last_seen = time.time()
        self.color = _id_color_rgb(self.id)

    def predicted_torso_center(self):
        vals = []
        for i in [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]:
            p = self.kf[i].get_position() if i < self.num_joints else None
            if p is not None:
                vals.append(p)
        if len(vals) >= 2:
            return np.mean(np.stack(vals, axis=0), axis=0)
        # fallback: mean of all initialized joints
        vals = []
        for k in self.kf:
            p = k.get_position()
            if p is not None:
                vals.append(p)
        if len(vals) >= 3:
            return np.mean(np.stack(vals, axis=0), axis=0)
        return None

    def step_unassigned(self, vel_damping: float):
        self.visible = [False] * self.num_joints
        for k in self.kf:
            if k.initialized:
                k.predict(vel_damping)
        # do not update last_seen

    def step_assigned(self, meas_pts, meas_visible, vel_damping, gate_sigma=2.5, max_jump=0.50):
        """
        meas_pts: list len=17 of np.array(3,) or None (ONLY for visible ones)
        meas_visible: list bool len=17
        """
        self.visible = [bool(v) for v in meas_visible]
        # update/predict per joint
        for i in range(self.num_joints):
            if self.visible[i] and meas_pts[i] is not None:
                meas = meas_pts[i]
                self.kf[i].predict(1.0)

                if self.kf[i].initialized:
                    pred = self.kf[i].get_position()
                    # gating using covariance + absolute sanity
                    sigma = float(np.sqrt(max(1e-9, np.trace(self.kf[i].P[0:3, 0:3]))))
                    if np.linalg.norm(meas - pred) > max_jump:
                        # too crazy -> treat as missing (predict only)
                        self.visible[i] = False
                        self.kf[i].predict(vel_damping)
                        continue
                    if np.linalg.norm(meas - pred) < gate_sigma * sigma:
                        self.kf[i].update(meas)
                    # else: reject update, keep prediction
                else:
                    self.kf[i].update(meas)
            else:
                if self.kf[i].initialized:
                    self.kf[i].predict(vel_damping)

        self.last_seen = time.time()

    def get_points(self):
        pts = []
        for k in self.kf:
            pts.append(k.get_position())
        return pts


# ============================================================
#                   Multi-tracking Node
# ============================================================

class YoloSkeletonMultiTrackNode(Node):
    def __init__(self):
        super().__init__("yolo_skeleton_multitrack_node")

        # ---------------- Params ----------------
        self.declare_parameter("model_path", "yolov8n-pose.pt")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.declare_parameter("dt", 1.0/30.0)
        self.declare_parameter("q", 0.2)
        self.declare_parameter("r", 0.02)

        self.declare_parameter("conf_thr", 0.30)
        self.declare_parameter("conf_thr_face", 0.60)  # stricter for face keypoints 0..4
        self.declare_parameter("vel_damping", 0.6)
        self.declare_parameter("max_depth_m", 3.0)

        self.declare_parameter("match_dist_m", 0.60)     # torso matching threshold
        self.declare_parameter("track_timeout_s", 1.5)   # remove if not seen
        self.declare_parameter("max_tracks", 6)

        self.declare_parameter("constraint_iters", 2)
        self.declare_parameter("constraint_stiffness", 0.9)

        self.model_path = self.get_parameter("model_path").value
        self.frame_id = self.get_parameter("frame_id").value

        self.dt = float(self.get_parameter("dt").value)
        self.q = float(self.get_parameter("q").value)
        self.r = float(self.get_parameter("r").value)

        self.conf_thr = float(self.get_parameter("conf_thr").value)
        self.conf_thr_face = float(self.get_parameter("conf_thr_face").value)
        self.vel_damping = float(self.get_parameter("vel_damping").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.match_dist_m = float(self.get_parameter("match_dist_m").value)
        self.track_timeout_s = float(self.get_parameter("track_timeout_s").value)
        self.max_tracks = int(self.get_parameter("max_tracks").value)

        self.constraint_iters = int(self.get_parameter("constraint_iters").value)
        self.constraint_stiffness = float(self.get_parameter("constraint_stiffness").value)

        # ---------------- YOLO ----------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
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
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10
        )

        # PoseArray per tracker (on-demand)
        self.pose_pubs = {}  # track_id -> publisher

        # ---------------- State ----------------
        self.depth_img = None
        self.cam_info = None

        self.num_joints = 17

        self.trackers = {}   # id -> PersonTracker
        self.next_id = 0

        # Skeleton edges
        self.edges = [
            (0, 1), (0, 2), (1, 3), (2, 4),
            (5, 6),
            (5, 7), (7, 9),
            (6, 8), (8, 10),
            (11, 12),
            (11, 13), (13, 15),
            (12, 14), (14, 16),
            (5, 11), (6, 12),
        ]

        # Same default bone lengths you used (ok for constraints)
        self.bone_lengths = {
            (5, 11): 0.45, (6, 12): 0.45,
            (11, 13): 0.42, (13, 15): 0.43,
            (12, 14): 0.42, (14, 16): 0.43,
            (5, 7): 0.30, (7, 9): 0.27,
            (6, 8): 0.30, (8, 10): 0.27
        }

        self.get_logger().info("✅ YOLO multi-tracking skeleton node ready")

    # ============================================================

    def cb_info(self, msg):
        self.cam_info = msg

    def cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def robust_depth(self, u, v, win=5):
        if self.depth_img is None:
            return None
        h, w = self.depth_img.shape
        if u < 0 or v < 0 or u >= w or v >= h:
            return None
        r = win // 2
        patch = self.depth_img[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1),
        ]
        patch = patch[patch > 0]
        if patch.size < 6:
            return None
        return float(np.median(patch)) * 0.001

    def _get_intrinsics(self):
        fx = float(self.cam_info.k[0])
        fy = float(self.cam_info.k[4])
        cx = float(self.cam_info.k[2])
        cy = float(self.cam_info.k[5])
        return fx, fy, cx, cy

    # ============================================================

    def cb_color(self, msg: Image):
        if self.depth_img is None or self.cam_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res_list = self.model(img, verbose=False)

        # ---------- NO RESULTS SAFE ----------
        if res_list is None or len(res_list) == 0:
            self._step_no_detections()
            self._publish_all_trackers(msg.header.stamp)
            return

        res0 = res_list[0]
        if res0.keypoints is None or res0.keypoints.xy is None:
            self._step_no_detections()
            self._publish_all_trackers(msg.header.stamp)
            return

        kp_xy_all = res0.keypoints.xy  # (N,17,2) torch
        if kp_xy_all.shape[0] == 0:
            self._step_no_detections()
            self._publish_all_trackers(msg.header.stamp)
            return

        conf_all = res0.keypoints.conf  # (N,17) torch or None
        kp_xy_all = kp_xy_all.cpu().numpy()
        conf_np = conf_all.cpu().numpy() if conf_all is not None else None

        fx, fy, cx, cy = self._get_intrinsics()

        detections = []
        n_persons = int(kp_xy_all.shape[0])

        # -------- Build detections (3D measurements per person) --------
        for p_idx in range(n_persons):
            kp = kp_xy_all[p_idx]  # (17,2)
            conf = conf_np[p_idx] if conf_np is not None else None

            pts3 = [None] * self.num_joints
            vis = [False] * self.num_joints

            for j in range(self.num_joints):
                if conf is not None:
                    thr = self.conf_thr_face if j < 5 else self.conf_thr
                    if float(conf[j]) < thr:
                        continue

                u = int(kp[j][0])
                v = int(kp[j][1])

                d = self.robust_depth(u, v)
                if d is None or d <= 0.0 or d > self.max_depth_m:
                    continue

                X = (u - cx) * d / fx
                Y = (v - cy) * d / fy
                Z = d

                pts3[j] = np.array([X, Y, Z], dtype=np.float64)
                vis[j] = True

            center = compute_torso_center(pts3, vis)
            if center is None:
                # if YOLO is too weak this frame, skip this detection
                continue

            detections.append({
                "pts3": pts3,
                "vis": vis,
                "center": center,
            })

        # If no usable detections -> predict only
        if len(detections) == 0:
            self._step_no_detections()
            self._publish_all_trackers(msg.header.stamp)
            return

        # -------- Data association: detections -> trackers (greedy) --------
        assignments = self._associate(detections)

        # -------- Update assigned trackers --------
        assigned_tracker_ids = set()
        for det_idx, trk_id in assignments.items():
            det = detections[det_idx]
            trk = self.trackers[trk_id]
            trk.step_assigned(
                det["pts3"], det["vis"],
                vel_damping=self.vel_damping
            )
            pts = trk.get_points()
            pts = apply_bone_constraints(
                pts, self.edges, self.bone_lengths,
                iters=self.constraint_iters,
                stiffness=self.constraint_stiffness
            )
            # write back constraint-projected points
            for j in range(self.num_joints):
                if pts[j] is not None and trk.kf[j].initialized:
                    trk.kf[j].set_position(pts[j])

            assigned_tracker_ids.add(trk_id)

        # -------- Unassigned trackers -> predict only --------
        for trk_id, trk in list(self.trackers.items()):
            if trk_id not in assigned_tracker_ids:
                trk.step_unassigned(self.vel_damping)

        # -------- Cleanup old trackers --------
        self._cleanup_tracks()

        # -------- Publish all trackers --------
        self._publish_all_trackers(msg.header.stamp)

    # ============================================================

    def _step_no_detections(self):
        for trk in self.trackers.values():
            trk.step_unassigned(self.vel_damping)
        self._cleanup_tracks()

    def _cleanup_tracks(self):
        now = time.time()
        to_del = []
        for trk_id, trk in self.trackers.items():
            if (now - trk.last_seen) > self.track_timeout_s:
                to_del.append(trk_id)
        for trk_id in to_del:
            # also remove pose publisher if exists
            if trk_id in self.pose_pubs:
                del self.pose_pubs[trk_id]
            del self.trackers[trk_id]

    def _associate(self, detections):
        """
        Greedy assignment based on torso center distance.
        Returns dict: det_idx -> tracker_id
        """
        # create trackers if none
        if len(self.trackers) == 0:
            out = {}
            for i in range(min(len(detections), self.max_tracks)):
                trk_id = self._new_tracker()
                out[i] = trk_id
            return out

        # compute all pair distances
        pairs = []
        tracker_ids = list(self.trackers.keys())
        for d_i, det in enumerate(detections):
            c_det = det["center"]
            for trk_id in tracker_ids:
                c_trk = self.trackers[trk_id].predicted_torso_center()
                if c_trk is None:
                    continue
                dist = float(np.linalg.norm(c_det - c_trk))
                pairs.append((dist, d_i, trk_id))

        pairs.sort(key=lambda x: x[0])

        assigned_dets = set()
        assigned_trks = set()
        out = {}

        # greedy match
        for dist, d_i, trk_id in pairs:
            if dist > self.match_dist_m:
                continue
            if d_i in assigned_dets or trk_id in assigned_trks:
                continue
            out[d_i] = trk_id
            assigned_dets.add(d_i)
            assigned_trks.add(trk_id)

        # create new trackers for unmatched detections (if capacity)
        for d_i in range(len(detections)):
            if d_i in assigned_dets:
                continue
            if len(self.trackers) >= self.max_tracks:
                continue
            trk_id = self._new_tracker()
            out[d_i] = trk_id
            assigned_dets.add(d_i)

        return out

    def _new_tracker(self):
        trk_id = int(self.next_id)
        self.next_id += 1
        self.trackers[trk_id] = PersonTracker(
            track_id=trk_id,
            num_joints=self.num_joints,
            dt=self.dt, q=self.q, r=self.r
        )
        return trk_id

    # ============================================================

    def _get_pose_pub(self, track_id: int):
        if track_id in self.pose_pubs:
            return self.pose_pubs[track_id]
        topic = f"/human_pose/points_3d/trk_{int(track_id)}"
        self.pose_pubs[track_id] = self.create_publisher(PoseArray, topic, 10)
        self.get_logger().info(f"PoseArray publisher created: {topic}")
        return self.pose_pubs[track_id]

    def _publish_all_trackers(self, stamp):
        ma = MarkerArray()

        # markers IDs must be unique inside MarkerArray;
        # we allocate blocks per tracker
        base_id = 0

        for trk_id in sorted(self.trackers.keys()):
            trk = self.trackers[trk_id]
            pts = trk.get_points()
            vis = trk.visible

            # ---- publish PoseArray per tracker (optional but useful for next nodes) ----
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

            self._get_pose_pub(trk_id).publish(pa)

            # ---- Visible joints marker ----
            j_vis = Marker()
            j_vis.header = pa.header
            j_vis.ns = f"joints_visible_{trk_id}"
            j_vis.id = base_id + 0
            j_vis.type = Marker.SPHERE_LIST
            j_vis.action = Marker.ADD
            j_vis.scale.x = j_vis.scale.y = j_vis.scale.z = 0.03
            # visible = red-ish, but per-person tint
            r, g, b = trk.color
            j_vis.color.r = 1.0
            j_vis.color.g = 0.2 * g
            j_vis.color.b = 0.2 * b
            j_vis.color.a = 1.0

            # ---- Predicted joints marker ----
            j_pred = Marker()
            j_pred.header = pa.header
            j_pred.ns = f"joints_predicted_{trk_id}"
            j_pred.id = base_id + 1
            j_pred.type = Marker.SPHERE_LIST
            j_pred.action = Marker.ADD
            j_pred.scale.x = j_pred.scale.y = j_pred.scale.z = 0.03
            # predicted = blue-ish, per-person tint
            j_pred.color.r = 0.2 * r
            j_pred.color.g = 0.2 * g
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

            # ---- Bones marker (green, per-person tint) ----
            bones = Marker()
            bones.header = pa.header
            bones.ns = f"bones_{trk_id}"
            bones.id = base_id + 2
            bones.type = Marker.LINE_LIST
            bones.action = Marker.ADD
            bones.scale.x = 0.015
            bones.color.r = 0.0
            bones.color.g = 1.0
            bones.color.b = 0.0
            bones.color.a = 1.0

            for a, c in self.edges:
                if a >= len(pts) or c >= len(pts):
                    continue
                if pts[a] is None or pts[c] is None:
                    continue
                bones.points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=float(pts[a][2])))
                bones.points.append(Point(x=float(pts[c][0]), y=float(pts[c][1]), z=float(pts[c][2])))

            ma.markers.append(bones)

            base_id += 10  # leave space for safety

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSkeletonMultiTrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()