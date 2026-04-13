#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO

from spot_perception.person_tracking import (
    PersonTrack,
    assign_detections_to_tracks,
    select_target,
    TORSO_length_constraint,
)

# ============================================================
#                       Skeleton Node
# ============================================================

class YoloSkeletonNodeOrbbec(Node):

    def __init__(self):
        super().__init__("yolo_skeleton_node_orbbec")

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter("model_path",             "yolo11n-pose.pt")
        self.declare_parameter("conf_thr",                0.25)
        self.declare_parameter("vel_damping",             0.5)
        self.declare_parameter("max_depth_m",             5.0)
        self.declare_parameter("z_offset",                0.0)
        self.declare_parameter("max_track_distance",      0.6)
        self.declare_parameter("track_timeout",           1.5)
        self.declare_parameter("lying_torso_angle_min",  65.0)
        self.declare_parameter("max_tracks",              5)
        self.declare_parameter("target_hysteresis_frames", 10)

        self.conf_thr              = float(self.get_parameter("conf_thr").value)
        self.vel_damping           = float(self.get_parameter("vel_damping").value)
        self.max_depth_m           = float(self.get_parameter("max_depth_m").value)
        self.z_offset              = float(self.get_parameter("z_offset").value)
        self._max_track_distance   = float(self.get_parameter("max_track_distance").value)
        self._track_timeout        = float(self.get_parameter("track_timeout").value)
        self._lying_angle_min      = float(self.get_parameter("lying_torso_angle_min").value)
        self._max_tracks           = int(self.get_parameter("max_tracks").value)
        self._hysteresis_frames    = int(self.get_parameter("target_hysteresis_frames").value)

        self.model  = YOLO(self.get_parameter("model_path").value)
        self.bridge = CvBridge()

        # ── Subscriptions ────────────────────────────────────────
        self.sub_color = self.create_subscription(Image,      "/camera/color/image_raw",   self.cb_color, 10)
        self.sub_depth = self.create_subscription(Image,      "/camera/depth/image_raw",   self.cb_depth, 10)
        self.sub_info  = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.cb_info,  10)

        # ── Publishers ───────────────────────────────────────────
        self.pub_poses   = self.create_publisher(PoseArray,   "/human_pose/points_3d",       10)
        self.pub_markers = self.create_publisher(MarkerArray, "/human_pose/skeleton_markers", 10)

        # ── Sensor state ─────────────────────────────────────────
        self.depth_img = None
        self.cam_info  = None

        # ── Multi-track state ────────────────────────────────────
        self.tracks: list              = []    # list[PersonTrack]
        self._next_track_id: int       = 0
        self._target_track_id          = None  # int | None
        self._target_hysteresis_miss   = 0
        self._published_track_ids: set = set()

        # ── Skeleton structure ───────────────────────────────────
        self.num_joints = 17
        self.TORSO = {5, 6, 11, 12}
        self.ARMS  = {7, 8, 9, 10}
        self.LEGS  = {13, 14, 15, 16}
        self.NOSE  = {0}

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

        self.KNEE_MIN_DEG = 30.0
        self.KNEE_MAX_DEG = 175.0

        self.get_logger().info("✅ YOLO skeleton node (Orbbec) — multi-person tracking ready")

    def _adaptive_Q(self, kf, missing_count, joint_idx):
        Q = kf.Q_base.copy()
        miss = missing_count[joint_idx]
        time_factor = min(1.0 + 0.15 * miss, 3.0)
        if joint_idx in {5, 6, 11, 12}:
            part_factor = 0.7
        elif joint_idx in {7, 8, 9, 10}:
            part_factor = 1.2
        elif joint_idx in {13, 14, 15, 16}:
            part_factor = 1.4
        elif joint_idx == 0:
            part_factor = 1.8
        else:
            part_factor = 1.0
        kf.Q = Q * time_factor * part_factor

    # ============================================================

    def cb_info(self, msg):
        self.cam_info = msg

    def cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def _compute_raw_centroid(self, kp, conf, fx, fy, cx, cy):
        """
        Compute 3D centroid of torso joints (5,6,11,12) from raw YOLO keypoints.
        Used for track assignment — no Kalman involved.
        Returns np.array([x,y,z]) or None if fewer than 2 torso joints have valid depth.
        """
        pts = []
        for i in [5, 6, 11, 12]:
            if conf is not None and conf[i] < self.conf_thr:
                continue
            u, v = int(kp[i][0]), int(kp[i][1])
            d = self.robust_depth(u, v)
            if d is None or d > self.max_depth_m:
                continue
            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            Z = d + self.z_offset
            pts.append(np.array([X, Y, Z], dtype=np.float64))
        if len(pts) < 2:
            return None
        return np.mean(pts, axis=0)

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
        now = time.monotonic()

        fx = self.cam_info.k[0];  fy = self.cam_info.k[4]
        cx = self.cam_info.k[2];  cy = self.cam_info.k[5]

        # ── Collect all YOLO detections ──────────────────────────
        kp_all, conf_all = [], []
        if (len(res) > 0
                and res[0].keypoints is not None
                and res[0].keypoints.xy is not None):
            kp_xy = res[0].keypoints.xy
            for di in range(kp_xy.shape[0]):
                kp_all.append(kp_xy[di].cpu().numpy())
                c = res[0].keypoints.conf
                conf_all.append(c[di].cpu().numpy() if c is not None else None)

        # ── Compute raw centroids for assignment ─────────────────
        centroids = [
            self._compute_raw_centroid(kp, conf, fx, fy, cx, cy)
            for kp, conf in zip(kp_all, conf_all)
        ]

        # ── Assign detections → tracks ───────────────────────────
        matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(
            centroids, self.tracks, self._max_track_distance
        )

        # ── Update matched tracks ─────────────────────────────────
        for di, ti in matches:
            self._update_track(self.tracks[ti], kp_all[di], conf_all[di], fx, fy, cx, cy)
            self.tracks[ti].last_seen = now
            if centroids[di] is not None:
                self.tracks[ti].centroid = centroids[di]

        # ── Predict unmatched tracks ──────────────────────────────
        for ti in unmatched_tracks:
            self._predict_track(self.tracks[ti])

        # ── Remove timed-out tracks ───────────────────────────────
        self.tracks = [
            t for t in self.tracks
            if (now - t.last_seen) < self._track_timeout
        ]

        # ── Create new tracks for unmatched detections ────────────
        for di in unmatched_dets:
            if len(self.tracks) >= self._max_tracks:
                break
            new_track = PersonTrack(self._next_track_id)
            self._next_track_id += 1
            self._update_track(new_track, kp_all[di], conf_all[di], fx, fy, cx, cy)
            new_track.last_seen = now
            if centroids[di] is not None:
                new_track.centroid = centroids[di]
            self.tracks.append(new_track)

        # ── Select target ─────────────────────────────────────────
        self._target_track_id, self._target_hysteresis_miss = select_target(
            self.tracks,
            lying_angle_min=self._lying_angle_min,
            current_target_id=self._target_track_id,
            hysteresis_miss_count=self._target_hysteresis_miss,
            hysteresis_frames=self._hysteresis_frames,
        )

        # ── Publish ───────────────────────────────────────────────
        target = next(
            (t for t in self.tracks if t.track_id == self._target_track_id), None
        )
        if target is not None:
            pts = [kf.get_position() for kf in target.kf]
            self._publish_target_pose(pts, msg.header.stamp)
        else:
            self.publish_empty(msg.header.stamp)

        self._publish_all_markers(msg.header.stamp)

    # ============================================================

    def _update_track(self, track, kp, conf, fx, fy, cx, cy):
        """
        Run one Kalman update step for the given PersonTrack using YOLO keypoints kp.
        Mirrors the per-joint logic from the original single-person cb_color.
        Returns pts: list[np.array|None] of length 17.
        """
        track.visible = [False] * self.num_joints
        pts = [None] * self.num_joints

        for i in range(self.num_joints):
            if i in {1, 2, 3, 4}:
                continue

            if i in self.LEGS:
                damping = 0.5
            elif i in self.ARMS:
                damping = 0.4
            elif i in self.TORSO:
                damping = 0.2
            else:
                damping = self.vel_damping

            if conf is not None and conf[i] < self.conf_thr:
                continue

            u, v = int(kp[i][0]), int(kp[i][1])
            d = self.robust_depth(u, v)
            if d is None or d > self.max_depth_m:
                continue

            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            Z = d + self.z_offset
            meas = np.array([X, Y, Z], dtype=np.float64)

            track.kf[i].predict(1.0)

            if i == 13 and pts[11] is not None and pts[15] is not None:
                if not self.knee_angle_ok(pts[11], meas, pts[15]):
                    track.kf[i].predict(damping)
                    pts[i] = track.kf[i].get_position()
                    continue

            if i == 14 and pts[12] is not None and pts[16] is not None:
                if not self.knee_angle_ok(pts[12], meas, pts[16]):
                    track.kf[i].Q *= 0.3
                    continue

            if track.kf[i].initialized:
                pred  = track.kf[i].get_position()
                sigma = np.sqrt(np.trace(track.kf[i].P[0:3, 0:3]))
                threshold = 3.5 if i in self.LEGS else 2.5
                if np.linalg.norm(meas - pred) < threshold * sigma:
                    track.kf[i].update(meas)
            else:
                track.kf[i].update(meas)

            track.visible[i] = True

        # Update missing counts
        for i in range(self.num_joints):
            if track.visible[i]:
                track.missing_count[i] = 0
            else:
                track.missing_count[i] += 1

        # Predict missing joints + get all positions
        for i in range(self.num_joints):
            if not track.visible[i]:
                self._adaptive_Q(track.kf[i], track.missing_count, i)
                if i in self.LEGS:
                    damp = 0.5
                elif i in self.ARMS:
                    damp = 0.4
                elif i in self.TORSO:
                    damp = 0.2
                else:
                    damp = self.vel_damping
                track.kf[i].predict(damp)
            else:
                track.kf[i].Q = track.kf[i].Q_base.copy()
            pts[i] = track.kf[i].get_position()

        # TORSO length constraint
        if all(pts[i] is not None for i in [5, 6, 11, 12]):
            sh_mid  = 0.5 * (pts[5]  + pts[6])
            hip_mid = 0.5 * (pts[11] + pts[12])
            L = np.linalg.norm(sh_mid - hip_mid)
            if track.TORSO_len_ref is None:
                track.TORSO_len_ref = L
            else:
                track.TORSO_len_ref = 0.98 * track.TORSO_len_ref + 0.02 * L

        pts = TORSO_length_constraint(pts, track.visible, track.TORSO_len_ref, stiffness=0.35)

        # Nose → shoulders soft constraint (only when nose is predicted, not visible)
        if (pts[0] is not None and pts[5] is not None
                and pts[6] is not None and not track.visible[0]):
            sh_mid = 0.5 * (pts[5] + pts[6])
            pts[0] = pts[0] + 0.55 * (sh_mid - pts[0])

        return pts

    def _predict_track(self, track):
        """Predict-only step for a track that had no matching detection this frame."""
        for i in range(self.num_joints):
            if i in {1, 2, 3, 4}:
                continue
            if track.kf[i].initialized:
                self._adaptive_Q(track.kf[i], track.missing_count, i)
                if i in self.LEGS:
                    damp = 0.5
                elif i in self.ARMS:
                    damp = 0.4
                elif i in self.TORSO:
                    damp = 0.2
                else:
                    damp = self.vel_damping
                track.kf[i].predict(damp)
                track.kf[i].Q = track.kf[i].Q_base.copy()
                track.missing_count[i] += 1
        track.visible = [False] * self.num_joints

    # ============================================================

    def publish_empty(self, stamp):
        """Publish empty PoseArray when no target is selected."""
        empty = PoseArray()
        empty.header.stamp = stamp
        empty.header.frame_id = "camera_color_optical_frame"
        self.pub_poses.publish(empty)

    def _publish_target_pose(self, pts, stamp):
        """Publish PoseArray of 17 joints for the target person only."""
        pa = PoseArray()
        pa.header.frame_id = "camera_color_optical_frame"
        pa.header.stamp = stamp
        for p in pts:
            pose = Pose()
            if p is None:
                pose.position.x = pose.position.y = pose.position.z = float("nan")
            else:
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.pub_poses.publish(pa)

    def _publish_all_markers(self, stamp):
        """
        Publish MarkerArray with all tracked skeletons.
        Target: green joints + bones + 'TARGET' text.
        Others: grey joints + bones.
        Removed tracks: DELETE markers.
        """
        ma = MarkerArray()
        current_ids = {t.track_id for t in self.tracks}

        # DELETE markers for tracks that no longer exist
        for old_id in self._published_track_ids - current_ids:
            for offset in range(4):
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = "camera_color_optical_frame"
                m.ns = "multi_track"
                m.id = old_id * 10 + offset
                m.action = Marker.DELETE
                ma.markers.append(m)

        # ADD / UPDATE markers for active tracks
        for track in self.tracks:
            is_target = (track.track_id == self._target_track_id)
            pts = [kf.get_position() for kf in track.kf]
            base_id = track.track_id * 10

            r, g, b = (0.0, 1.0, 0.0) if is_target else (0.6, 0.6, 0.6)

            # Visible joints
            jv = Marker()
            jv.header.stamp = stamp
            jv.header.frame_id = "camera_color_optical_frame"
            jv.ns = "multi_track";  jv.id = base_id + 0
            jv.type = Marker.SPHERE_LIST;  jv.action = Marker.ADD
            jv.scale.x = jv.scale.y = jv.scale.z = 0.03
            jv.color.r = r;  jv.color.g = g;  jv.color.b = b;  jv.color.a = 1.0

            # Predicted joints (dimmer)
            jp = Marker()
            jp.header.stamp = stamp
            jp.header.frame_id = "camera_color_optical_frame"
            jp.ns = "multi_track";  jp.id = base_id + 1
            jp.type = Marker.SPHERE_LIST;  jp.action = Marker.ADD
            jp.scale.x = jp.scale.y = jp.scale.z = 0.03
            jp.color.r = r * 0.4;  jp.color.g = g * 0.4
            jp.color.b = b * 0.4 + 0.3;  jp.color.a = 0.5

            for i, p in enumerate(pts):
                if p is None:
                    continue
                pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                if i < len(track.visible) and track.visible[i]:
                    jv.points.append(pt)
                else:
                    jp.points.append(pt)

            # Bones
            bn = Marker()
            bn.header.stamp = stamp
            bn.header.frame_id = "camera_color_optical_frame"
            bn.ns = "multi_track";  bn.id = base_id + 2
            bn.type = Marker.LINE_LIST;  bn.action = Marker.ADD
            bn.scale.x = 0.015
            bn.color.r = r;  bn.color.g = g;  bn.color.b = b;  bn.color.a = 0.8

            for a, c in self.edges:
                if pts[a] is not None and pts[c] is not None:
                    bn.points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=float(pts[a][2])))
                    bn.points.append(Point(x=float(pts[c][0]), y=float(pts[c][1]), z=float(pts[c][2])))

            ma.markers.extend([jv, jp, bn])

            # TARGET text label — only for target track
            if is_target and pts[5] is not None and pts[6] is not None:
                sh_mid = 0.5 * (pts[5] + pts[6])
                lbl = Marker()
                lbl.header.stamp = stamp
                lbl.header.frame_id = "camera_color_optical_frame"
                lbl.ns = "multi_track";  lbl.id = base_id + 3
                lbl.type = Marker.TEXT_VIEW_FACING;  lbl.action = Marker.ADD
                lbl.pose.position.x = float(sh_mid[0])
                lbl.pose.position.y = float(sh_mid[1]) - 0.15  # slightly above shoulders
                lbl.pose.position.z = float(sh_mid[2])
                lbl.pose.orientation.w = 1.0
                lbl.scale.z = 0.12
                lbl.color.r = 0.0;  lbl.color.g = 1.0;  lbl.color.b = 0.0;  lbl.color.a = 1.0
                lbl.text = "TARGET"
                ma.markers.append(lbl)
            else:
                # Ensure label is deleted when track is no longer the target
                lbl_del = Marker()
                lbl_del.header.stamp = stamp
                lbl_del.header.frame_id = "camera_color_optical_frame"
                lbl_del.ns = "multi_track";  lbl_del.id = base_id + 3
                lbl_del.action = Marker.DELETE
                ma.markers.append(lbl_del)

        self._published_track_ids = current_ids
        self.pub_markers.publish(ma)

    # ============================================================


def main():
    rclpy.init()
    node = YoloSkeletonNodeOrbbec()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
