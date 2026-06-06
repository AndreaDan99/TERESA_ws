#!/usr/bin/env python3
"""
YOLO Skeleton detection with ByteTrack multi-person tracking (Orbbec Femto Bolt).

Uses YOLO's built-in model.track() (ByteTrack) for persistent per-person IDs
across frames.  Depth back-projection gives 3D keypoints from 2D+depth.
Target selection picks the closest LYING person with hysteresis.

Internally tracks 17 COCO keypoints from YOLO.
At publish time, converts to 24 SMPL joints via coco_to_smpl_24().

Publishers:
  /human_pose/points_3d        (PoseArray, 24 SMPL joints of target)
  /human_pose/skeleton_markers  (MarkerArray, all tracked skeletons)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO

from spot_perception.yolo_to_smpl_pad import coco_to_smpl_24


# ── Geometry helper ──────────────────────────────────────────────────

def angle_deg(a, b):
    """Angle between two 3D vectors in degrees."""
    an = a / (np.linalg.norm(a) + 1e-9)
    bn = b / (np.linalg.norm(b) + 1e-9)
    c = float(np.clip(np.dot(an, bn), -1.0, 1.0))
    return float(np.degrees(math.acos(c)))


# ── Node ─────────────────────────────────────────────────────────────

class YoloSkeletonNode(Node):
    """YOLO pose + ByteTrack → 3D skeleton + target selection."""

    def __init__(self):
        super().__init__("yolo_skeleton_node_orbbec")

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter("model_path",                "yolo11n-pose.pt")
        self.declare_parameter("conf_thr",                   0.25)
        self.declare_parameter("max_depth_m",                5.0)
        self.declare_parameter("z_offset",                   0.0)
        self.declare_parameter("lying_torso_angle_min",     65.0)
        self.declare_parameter("target_hysteresis_frames",   10)

        self.conf_thr         = float(self.get_parameter("conf_thr").value)
        self.max_depth_m      = float(self.get_parameter("max_depth_m").value)
        self.z_offset         = float(self.get_parameter("z_offset").value)
        self._lying_angle_min = float(self.get_parameter("lying_torso_angle_min").value)
        self._hysteresis_frames = int(self.get_parameter("target_hysteresis_frames").value)

        self.model  = YOLO(self.get_parameter("model_path").value)
        self.bridge = CvBridge()

        # ── Subscriptions ────────────────────────────────────────
        self.sub_color = self.create_subscription(
            Image, "/orbbec/color/image_raw", self.cb_color, 10)
        self.sub_depth = self.create_subscription(
            Image, "/orbbec/depth/image_raw", self.cb_depth, 10)
        self.sub_info  = self.create_subscription(
            CameraInfo, "/orbbec/color/camera_info", self.cb_info, 10)

        # ── Publishers ───────────────────────────────────────────
        self.pub_poses   = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10)

        # ── Sensor state ─────────────────────────────────────────
        self.depth_img = None
        self.cam_info  = None

        # ── Tracking state ───────────────────────────────────────
        self.target_id           = None          # current target (ByteTrack ID)
        self.hysteresis_miss     = 0             # consecutive non-lying frames
        self._published_track_ids: set = set()   # for marker DELETEs
        self._smoothed_kp: dict = {}   # EMA smoothed keypoints per ByteTrack person ID
        self._ema_alpha = 0.3          # smoothing factor (0=no update, 1=raw)

        # ── Skeleton structure (COCO 17) ─────────────────────────
        self.num_joints = 17
        self.edges = [
            (0, 1),  (0, 2),  (1, 3),  (2, 4),
            (5, 6),
            (5, 7),  (7, 9),   (6, 8),  (8, 10),
            (11, 12),
            (11, 13), (13, 15), (12, 14), (14, 16),
            (5, 11), (6, 12),
        ]

        self.get_logger().info(
            "✅ YOLO skeleton node (Orbbec) — ByteTrack tracking ready, 24 SMPL output")

    # ================================================================
    #  Callbacks
    # ================================================================

    def cb_info(self, msg):
        self.cam_info = msg

    def cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    # ================================================================
    #  Depth utilities
    # ================================================================

    def robust_depth(self, u, v, win=5):
        """Median depth in a *win*×*win* patch around (u, v).  Returns metres."""
        h, w = self.depth_img.shape
        r = win // 2
        patch = self.depth_img[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1),
        ]
        patch = patch[patch > 0]
        if patch.size < 6:
            return None
        return float(np.median(patch)) * 0.001

    def _back_project(self, u, v, d, fx, fy, cx, cy):
        """Pixel + depth → 3D point in camera optical frame."""
        return np.array([
            (u - cx) * d / fx,
            (v - cy) * d / fy,
            d + self.z_offset,
        ], dtype=np.float64)

    # ================================================================
    #  Keypoints → 3D
    # ================================================================

    def _keypoints_3d(self, kp_xy, kp_conf, fx, fy, cx, cy):
        """Convert a single person's 2D keypoints → list[Optional[np.ndarray]]."""
        pts = [None] * self.num_joints
        for i in range(self.num_joints):
            if kp_conf is not None and kp_conf[i] < self.conf_thr:
                continue
            u, v = int(kp_xy[i][0]), int(kp_xy[i][1])
            d = self.robust_depth(u, v)
            if d is None or d > self.max_depth_m:
                continue
            pts[i] = self._back_project(u, v, d, fx, fy, cx, cy)
        return pts

    # ================================================================
    #  Posture / target selection
    # ================================================================

    @staticmethod
    def _torso_angle(pts_3d):
        """Angle between torso vector and camera vertical (-Y).  Deg, or None."""
        shoulders = [pts_3d[i] for i in (5, 6) if pts_3d[i] is not None]
        hips      = [pts_3d[i] for i in (11, 12) if pts_3d[i] is not None]
        if not shoulders or not hips:
            return None
        sh_mid  = np.mean(shoulders, axis=0)
        hip_mid = np.mean(hips, axis=0)
        up = np.array([0.0, -1.0, 0.0], dtype=np.float64)  # Orbbec optical frame
        return angle_deg(sh_mid - hip_mid, up)

    def _select_target(self, detections):
        """Hysteresis-based selection: closest LYING person. Returns track_id or None."""
        lying = []
        for det in detections:
            angle = self._torso_angle(det["pts_3d"])
            if angle is not None and angle > self._lying_angle_min:
                lying.append((det["depth_z"], det["id"]))

        if not lying:
            self.hysteresis_miss += 1
            if self.hysteresis_miss > self._hysteresis_frames:
                self.target_id = None
            return self.target_id

        lying.sort()  # by depth_z ascending → closest first
        self.hysteresis_miss = 0

        # Keep current target if still lying
        if self.target_id is not None:
            for _, tid in lying:
                if tid == self.target_id:
                    return self.target_id

        # Switch to closest lying
        self.target_id = lying[0][1]
        return self.target_id

    # ================================================================
    #  Main detection callback
    # ================================================================

    def cb_color(self, msg):
        if self.depth_img is None or self.cam_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # ── YOLO with ByteTrack ─────────────────────────────────
        results = self.model.track(
            img,
            persist=True,
            tracker="bytetrack.yaml",
            conf=self.conf_thr,
            verbose=False,
        )

        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        # ── Collect per-person detections ────────────────────────
        detections = []
        if (len(results) > 0
                and results[0].keypoints is not None
                and results[0].keypoints.xy is not None):
            kp_xy   = results[0].keypoints.xy
            kp_conf = results[0].keypoints.conf
            boxes   = results[0].boxes

            for di in range(kp_xy.shape[0]):
                kp   = kp_xy[di].cpu().numpy()                    # (17, 2)
                conf = kp_conf[di].cpu().numpy() if kp_conf is not None else None

                # ByteTrack persistent ID
                tid = None
                if boxes is not None and boxes.id is not None:
                    tid = int(boxes.id[di].item())

                pts_3d = self._keypoints_3d(kp, conf, fx, fy, cx, cy)

                # EMA smoothing per person (replaces Kalman3D)
                if tid is not None:
                    prev = self._smoothed_kp.get(tid, pts_3d)
                    smoothed = []
                    for j in range(self.num_joints):
                        if pts_3d[j] is not None and prev[j] is not None:
                            smoothed.append(self._ema_alpha * pts_3d[j] + (1 - self._ema_alpha) * prev[j])
                        else:
                            smoothed.append(pts_3d[j] if pts_3d[j] is not None else (prev[j] if prev[j] is not None else None))
                    self._smoothed_kp[tid] = smoothed
                    pts_3d = smoothed

                # Centre-depth for sorting (median of torso joints)
                torso_z = []
                for i in (5, 6, 11, 12):
                    if pts_3d[i] is not None:
                        torso_z.append(pts_3d[i][2])
                depth_z = float(np.median(torso_z)) if torso_z else self.max_depth_m

                detections.append({
                    "id":      tid,
                    "pts_3d":  pts_3d,
                    "depth_z": depth_z,
                })

        # ── Target selection ─────────────────────────────────────
        target_id = self._select_target(detections)

        # ── Publish target PoseArray (24 SMPL joints) ────────────
        target = next((d for d in detections if d["id"] == target_id), None)
        if target is not None:
            self._publish_pose(target["pts_3d"], msg.header.stamp)
        else:
            self._publish_empty(msg.header.stamp)

        # ── Publish all skeletons as markers ──────────────────────
        self._publish_markers(detections, target_id, msg.header.stamp)

    # ================================================================
    #  Publishers
    # ================================================================

    def _publish_empty(self, stamp):
        pa = PoseArray()
        pa.header.stamp    = stamp
        pa.header.frame_id = "orbbec_color_optical_frame"
        self.pub_poses.publish(pa)

    def _publish_pose(self, pts, stamp):
        """Publish PoseArray of 24 SMPL joints for the target person."""
        # Build 17 COCO poses first
        coco_poses = []
        for p in pts:
            pose = Pose()
            if p is None:
                pose.position.x = pose.position.y = pose.position.z = float("nan")
            else:
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            coco_poses.append(pose)

        # Convert to 24 SMPL joints
        smpl_poses = coco_to_smpl_24(coco_poses)

        pa = PoseArray()
        pa.header.frame_id = "orbbec_color_optical_frame"
        pa.header.stamp    = stamp
        pa.poses = smpl_poses
        self.pub_poses.publish(pa)

    def _publish_markers(self, detections, target_id, stamp):
        ma = MarkerArray()
        current_ids = {d["id"] for d in detections if d["id"] is not None}

        # DELETE markers for disappeared tracks
        for old_id in self._published_track_ids - current_ids:
            for offset in range(4):
                m = Marker()
                m.header.stamp    = stamp
                m.header.frame_id = "orbbec_color_optical_frame"
                m.ns   = "multi_track"
                m.id   = old_id * 10 + offset
                m.action = Marker.DELETE
                ma.markers.append(m)

        # ADD / UPDATE markers for current detections
        for det in detections:
            tid = det["id"]
            if tid is None:
                continue
            pts       = det["pts_3d"]
            is_target = (tid == target_id)
            base_id   = tid * 10

            r, g, b = (0.0, 1.0, 0.0) if is_target else (0.6, 0.6, 0.6)

            # ── Joint spheres (visible) ──────────────────────
            jv = Marker()
            jv.header.stamp    = stamp
            jv.header.frame_id = "orbbec_color_optical_frame"
            jv.ns   = "multi_track"
            jv.id   = base_id + 0
            jv.type = Marker.SPHERE_LIST
            jv.action = Marker.ADD
            jv.scale.x = jv.scale.y = jv.scale.z = 0.03
            jv.color.r = r;  jv.color.g = g;  jv.color.b = b;  jv.color.a = 1.0

            # ── Joint spheres (predicted — empty, no Kalman) ──
            jp = Marker()
            jp.header.stamp    = stamp
            jp.header.frame_id = "orbbec_color_optical_frame"
            jp.ns   = "multi_track"
            jp.id   = base_id + 1
            jp.type = Marker.SPHERE_LIST
            jp.action = Marker.ADD
            jp.scale.x = jp.scale.y = jp.scale.z = 0.03
            jp.color.r = r * 0.4;  jp.color.g = g * 0.4
            jp.color.b = b * 0.4 + 0.3;  jp.color.a = 0.5

            for p in pts:
                if p is None:
                    continue
                pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                jv.points.append(pt)  # all observed — no Kalman prediction

            # ── Bones ────────────────────────────────────────
            bn = Marker()
            bn.header.stamp    = stamp
            bn.header.frame_id = "orbbec_color_optical_frame"
            bn.ns   = "multi_track"
            bn.id   = base_id + 2
            bn.type = Marker.LINE_LIST
            bn.action = Marker.ADD
            bn.scale.x = 0.015
            bn.color.r = r;  bn.color.g = g;  bn.color.b = b;  bn.color.a = 0.8

            for a, c in self.edges:
                if pts[a] is not None and pts[c] is not None:
                    bn.points.append(Point(
                        x=float(pts[a][0]), y=float(pts[a][1]), z=float(pts[a][2])))
                    bn.points.append(Point(
                        x=float(pts[c][0]), y=float(pts[c][1]), z=float(pts[c][2])))

            ma.markers.extend([jv, jp, bn])

            # ── TARGET label ─────────────────────────────────
            if is_target and pts[5] is not None and pts[6] is not None:
                sh_mid = 0.5 * (pts[5] + pts[6])
                lbl = Marker()
                lbl.header.stamp    = stamp
                lbl.header.frame_id = "orbbec_color_optical_frame"
                lbl.ns   = "multi_track"
                lbl.id   = base_id + 3
                lbl.type = Marker.TEXT_VIEW_FACING
                lbl.action = Marker.ADD
                lbl.pose.position.x = float(sh_mid[0])
                lbl.pose.position.y = float(sh_mid[1]) - 0.15
                lbl.pose.position.z = float(sh_mid[2])
                lbl.pose.orientation.w = 1.0
                lbl.scale.z = 0.12
                lbl.color.r = 0.0;  lbl.color.g = 1.0
                lbl.color.b = 0.0;  lbl.color.a = 1.0
                lbl.text = "TARGET"
                ma.markers.append(lbl)
            else:
                lbl_del = Marker()
                lbl_del.header.stamp    = stamp
                lbl_del.header.frame_id = "orbbec_color_optical_frame"
                lbl_del.ns   = "multi_track"
                lbl_del.id   = base_id + 3
                lbl_del.action = Marker.DELETE
                ma.markers.append(lbl_del)

        self._published_track_ids = current_ids
        self.pub_markers.publish(ma)


# ── Entry point ──────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = YoloSkeletonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
