#!/usr/bin/env python3
"""
NLF Skeleton Node — drop-in replacement for yolo_skeleton_spot.py.
RGB-only (no depth), 24 SMPL joints natively, NO COCO→SMPL mapping.
Publishes on same ROS2 topics: /human_pose/points_3d, /human_pose/skeleton_markers.
"""

import time

import numpy as np

import torch
import torchvision  # MANDATORY for NLF TorchScript model (YOLOv8x ops)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from std_msgs.msg import Header

from spot_perception.person_tracking import (
    Kalman3D,
    assign_detections_to_tracks,
)

from spot_perception.sml_pose_indices import (
    PELVIS, HIP_LEFT, HIP_RIGHT,
    SPINE1, SPINE2, SPINE3,
    KNEE_LEFT, KNEE_RIGHT,
    ANKLE_LEFT, ANKLE_RIGHT,
    FOOT_LEFT, FOOT_RIGHT,
    NECK, COLLAR_LEFT, COLLAR_RIGHT, HEAD,
    SHOULDER_LEFT, SHOULDER_RIGHT,
    ELBOW_LEFT, ELBOW_RIGHT,
    WRIST_LEFT, WRIST_RIGHT,
    HAND_LEFT, HAND_RIGHT,
    NUM_JOINTS,
)

# ── SMPL joint group sets ─────────────────────────────────────────────────────
SMPL_TORSO_CENTROID = {SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT}
SMPL_TORSO_BODY = {
    SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT,
    SPINE1, SPINE2, SPINE3, PELVIS, NECK,
}
SMPL_ARMS = {
    SHOULDER_LEFT, ELBOW_LEFT, WRIST_LEFT,
    SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT,
    COLLAR_LEFT, COLLAR_RIGHT,
}
SMPL_LEGS = {
    KNEE_LEFT, KNEE_RIGHT,
    ANKLE_LEFT, ANKLE_RIGHT,
    FOOT_LEFT, FOOT_RIGHT,
}
SMPL_HANDS = {HAND_LEFT, HAND_RIGHT}
SMPL_HEAD_GROUP = {NECK, HEAD}

# ── SMPL kinematic tree edges (for skeleton visualization) ─────────────────────
_SMPL_EDGES = [
    # Spine chain
    (PELVIS, SPINE1), (SPINE1, SPINE2), (SPINE2, SPINE3), (SPINE3, NECK), (NECK, HEAD),
    # Left leg
    (PELVIS, HIP_LEFT), (HIP_LEFT, KNEE_LEFT), (KNEE_LEFT, ANKLE_LEFT), (ANKLE_LEFT, FOOT_LEFT),
    # Right leg
    (PELVIS, HIP_RIGHT), (HIP_RIGHT, KNEE_RIGHT), (KNEE_RIGHT, ANKLE_RIGHT), (ANKLE_RIGHT, FOOT_RIGHT),
    # Left arm
    (NECK, COLLAR_LEFT), (COLLAR_LEFT, SHOULDER_LEFT),
    (SHOULDER_LEFT, ELBOW_LEFT), (ELBOW_LEFT, WRIST_LEFT), (WRIST_LEFT, HAND_LEFT),
    # Right arm
    (NECK, COLLAR_RIGHT), (COLLAR_RIGHT, SHOULDER_RIGHT),
    (SHOULDER_RIGHT, ELBOW_RIGHT), (ELBOW_RIGHT, WRIST_RIGHT), (WRIST_RIGHT, HAND_RIGHT),
    # Cross-connections
    (SHOULDER_LEFT, SHOULDER_RIGHT), (HIP_LEFT, HIP_RIGHT),
]

_UP = np.array([0.0, -1.0, 0.0], dtype=np.float64)  # camera optical: Y↓ → world-up = -Y


# ============================================================
#  NLF-specific PersonTrack — 24 Kalman3D filters per person
# ============================================================

class NLFPersonTrack:
    """Tracked person: 24 Kalman3D filters + metadata.  SMPL-adapted PersonTrack."""

    def __init__(self, track_id: int):
        self.track_id = track_id
        self.kf = [Kalman3D() for _ in range(NUM_JOINTS)]
        self.visible = [False] * NUM_JOINTS
        self.missing_count = [0] * NUM_JOINTS
        self.TORSO_len_ref = None
        self.centroid = None  # np.array([x,y,z])
        self.last_seen: float = time.monotonic()
        self._cached_pts = [None] * NUM_JOINTS

        for i, kf in enumerate(self.kf):
            if i in SMPL_TORSO_BODY:
                kf.Q *= 0.7; kf.R *= 0.7
            elif i in SMPL_LEGS:
                kf.Q *= 0.9; kf.R *= 0.9
            elif i in SMPL_HEAD_GROUP:
                kf.Q *= 1.5; kf.R *= 1.5


# ============================================================
#  SMPL-adapted utility functions (equivalent to person_tracking.py helpers)
# ============================================================

def _smpl_torso_length_constraint(pts, visible, L_ref, stiffness=0.35):
    """Soft constraint: keep shoulder-hip distance close to L_ref. Mutates pts in-place."""
    if L_ref is None:
        return pts
    idx = [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT]
    if any(pts[i] is None for i in idx):
        return pts
    if all(visible[i] for i in idx):
        return pts

    sh_mid = 0.5 * (pts[SHOULDER_LEFT] + pts[SHOULDER_RIGHT])
    hip_mid = 0.5 * (pts[HIP_LEFT] + pts[HIP_RIGHT])
    v = sh_mid - hip_mid
    dist = np.linalg.norm(v)
    if dist < 1e-6:
        return pts
    v_corr = (v / dist) * L_ref
    delta = hip_mid + v_corr - sh_mid
    pts[SHOULDER_LEFT] = pts[SHOULDER_LEFT] + stiffness * delta
    pts[SHOULDER_RIGHT] = pts[SHOULDER_RIGHT] + stiffness * delta
    return pts


def _smpl_torso_angle_deg(track):
    """Torso angle (°) between shoulder-hip vector and world-up. Returns None if unavailable."""
    pts = [kf.get_position() for kf in track.kf]
    for i in [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT]:
        if pts[i] is None:
            return None
    sh_mid = 0.5 * (pts[SHOULDER_LEFT] + pts[SHOULDER_RIGHT])
    hip_mid = 0.5 * (pts[HIP_LEFT] + pts[HIP_RIGHT])
    v = sh_mid - hip_mid
    n = np.linalg.norm(v)
    if n < 1e-6:
        return None
    cos_a = float(np.clip(np.dot(v / n, _UP), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_a)))


def _smpl_select_target(tracks, lying_angle_min,
                        current_target_id=None,
                        hysteresis_miss_count=0,
                        hysteresis_frames=10):
    """Select lying person closest to camera. SMPL-adapted select_target."""
    lying_candidates = []
    for track in tracks:
        angle = _smpl_torso_angle_deg(track)
        if angle is None:
            continue
        n_valid = sum(1 for kf in track.kf if kf.get_position() is not None)
        if angle > lying_angle_min and n_valid >= 4:
            depth = float(track.centroid[2]) if track.centroid is not None else float('inf')
            lying_candidates.append((depth, track.track_id))
    lying_candidates.sort()

    if any(tid == current_target_id for _, tid in lying_candidates):
        return current_target_id, 0
    if current_target_id is not None and hysteresis_miss_count < hysteresis_frames:
        if current_target_id in {t.track_id for t in tracks}:
            return current_target_id, hysteresis_miss_count + 1
    if lying_candidates:
        return lying_candidates[0][1], 0
    return None, 0


# ============================================================
#                       NLF Skeleton Node
# ============================================================

class NLFSkeletonNode(Node):
    """ROS2 Node: NLF-based 3D human skeleton. Drop-in replacement for YoloSkeletonNodeOrbbec."""

    def __init__(self):
        super().__init__("nlf_skeleton_node")

        # ── Parameters (from nlf_params.yaml) ─────────────────────────────────
        self.declare_parameter("model_path",        "nlf_s_multi.torchscript")
        self.declare_parameter("model_url",         "https://github.com/isarandi/nlf/releases/download/v0.2.0/nlf_s_multi_multi.torchscript")
        self.declare_parameter("device",            "cuda")
        self.declare_parameter("conf_threshold",     0.3)
        self.declare_parameter("max_depth_m",        5.0)
        self.declare_parameter("imgsz",              416)
        self.declare_parameter("publish_mesh",       True)
        self.declare_parameter("mesh_topic",         "/human_pose/smpl_mesh")
        self.declare_parameter("mesh_decimation",    4)
        self.declare_parameter("vel_damping",            0.5)
        self.declare_parameter("z_offset",               0.0)
        self.declare_parameter("max_track_distance",     0.6)
        self.declare_parameter("track_timeout",          1.5)
        self.declare_parameter("lying_torso_angle_min", 65.0)
        self.declare_parameter("max_tracks",             5)
        self.declare_parameter("target_hysteresis_frames", 10)
        self.declare_parameter("process_every_n_frames", 5)

        self._process_every = int(self.get_parameter("process_every_n_frames").value)
        self._frame_count = 0

        self._model_path     = str(self.get_parameter("model_path").value)
        self._device         = str(self.get_parameter("device").value)
        self._conf_thr       = float(self.get_parameter("conf_threshold").value)
        self._max_depth_m    = float(self.get_parameter("max_depth_m").value)
        self._imgsz          = int(self.get_parameter("imgsz").value)
        self._publish_mesh   = bool(self.get_parameter("publish_mesh").value)
        self._mesh_topic     = str(self.get_parameter("mesh_topic").value)
        self._mesh_decimation = int(self.get_parameter("mesh_decimation").value)
        self.vel_damping         = float(self.get_parameter("vel_damping").value)
        self.z_offset            = float(self.get_parameter("z_offset").value)
        self._max_track_distance = float(self.get_parameter("max_track_distance").value)
        self._track_timeout      = float(self.get_parameter("track_timeout").value)
        self._lying_angle_min    = float(self.get_parameter("lying_torso_angle_min").value)
        self._max_tracks         = int(self.get_parameter("max_tracks").value)
        self._hysteresis_frames  = int(self.get_parameter("target_hysteresis_frames").value)

        self.num_joints = NUM_JOINTS
        self.bridge = CvBridge()

        # Subscriptions — RGB-only, NO depth (NLF gives 3D directly)
        self.sub_color = self.create_subscription(
            Image, "/orbbec/color/image_raw", self._cb_color, 10)
        self.sub_info = self.create_subscription(
            CameraInfo, "/orbbec/color/camera_info", self._cb_caminfo, 10)

        # Publishers
        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10)
        if self._publish_mesh:
            self.pub_mesh = self.create_publisher(
                PoseArray, self._mesh_topic, 10)

        # State
        self.cam_info = None
        self.tracks: list = []             # list[NLFPersonTrack]
        self._next_track_id: int = 0
        self._target_track_id = None
        self._target_hysteresis_miss = 0
        self._published_track_ids: set = set()
        self._last_vertices = None          # (N,3) or None — for mesh publishing
        self.edges = _SMPL_EDGES
        self.KNEE_MIN_DEG = 30.0
        self.KNEE_MAX_DEG = 175.0

        # NLF model (lazy init, stub until nlf_s_multi.torchscript is available)
        self._nlf_model = None
        self._nlf_ready = False
        self._nlf_stub_warned = False

        self.get_logger().info(
            "\u2705 NLF skeleton node — 24 SMPL joints (RGB-only, multi-person tracking)")

    # ── NLF model loading ──────────────────────────────────────────────────────

    def _init_model(self):
        """Load NLF TorchScript model from self._model_path. Sets self._nlf_ready on success."""
        try:
            import torch  # noqa: F401
            import torchvision  # noqa: F401
        except ImportError as e:
            self.get_logger().error(
                f"PyTorch/torchvision not installed ({e}). "
                "Install with: pip install torch torchvision")
            self._nlf_stub_warned = True
            return

        try:
            self.get_logger().info(
                f"Loading NLF model: {self._model_path} on {self._device}")
            self._nlf_model = torch.jit.load(self._model_path, map_location=self._device)
            if self._device == 'cuda' and torch.cuda.is_available():
                self._nlf_model = self._nlf_model.cuda()
            else:
                self._nlf_model = self._nlf_model.cpu()
                if self._device == 'cuda':
                    self.get_logger().warn(
                        "CUDA requested but not available — falling back to CPU")
            self._nlf_model.eval()
            self._nlf_ready = True
            self.get_logger().info("NLF model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load NLF model: {e}")
            self._nlf_stub_warned = True

    def _cb_caminfo(self, msg):
        self.cam_info = msg

    # ── NLF inference ──────────────────────────────────────────────────────────

    def _run_nlf_inference(self, img_rgb):
        """
        Run NLF inference on RGB image (numpy H×W×3 uint8).
        Returns list[dict] per detected person:
          {'joints3d': (24,3) meters, 'conf': (24,) [0-1],
           'vertices3d': (6890,3) meters|None, 'bbox': ...}
        """
        if not self._nlf_ready:
            return []

        try:
            # HWC → CHW → add batch dim, keep uint8 [0-255]
            image_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).unsqueeze(0)

            with torch.inference_mode():
                pred = self._nlf_model.detect_smpl_batched(
                    image_tensor,
                    default_fov_degrees=55.0,
                    num_aug=1,
                    detector_threshold=self._conf_thr,
                    internal_batch_size=64,
                    suppress_implausible_poses=True,
                )

            # Extract results for the (single) batch element
            if (not pred.get('joints3d') or len(pred['joints3d']) == 0
                    or len(pred['joints3d'][0]) == 0):
                return []

            joints_mm = pred['joints3d'][0]          # (n_people, 24, 3) mm
            joints_m = joints_mm.cpu().numpy() / 1000.0  # m
            n_people = joints_m.shape[0]

            # Per-joint confidence from uncertainty (mm) → [0-1]
            if pred.get('joint_uncertainties') and len(pred['joint_uncertainties']) > 0:
                uncert_mm = pred['joint_uncertainties'][0].cpu().numpy()  # (n_people, 24)
                conf = np.exp(-uncert_mm / 50.0)  # 50 mm → ~0.37
            else:
                conf = np.ones((n_people, NUM_JOINTS), dtype=np.float64)

            # Box scores for person-level conf
            box_scores = np.ones(n_people, dtype=np.float64)
            if pred.get('boxes') and len(pred['boxes']) > 0:
                boxes = pred['boxes'][0].cpu().numpy()  # (n_people, 5)
                if boxes.shape[0] == n_people:
                    box_scores = boxes[:, 4]  # confidence score

            # Vertices for mesh (if available)
            vertices_list = [None] * n_people
            if pred.get('vertices3d') and len(pred['vertices3d']) > 0:
                verts_mm = pred['vertices3d'][0].cpu().numpy()  # (n_people, 6890, 3) mm
                if verts_mm.shape[0] == n_people:
                    for p in range(n_people):
                        vertices_list[p] = verts_mm[p] / 1000.0  # m

            # Build per-person detection dicts
            detections = []
            for p in range(n_people):
                det = {
                    'joints3d': joints_m[p],           # (24, 3) m
                    'conf': conf[p].astype(np.float64),  # (24,)
                    'bbox_score': float(box_scores[p]),
                }
                if vertices_list[p] is not None:
                    det['vertices3d'] = vertices_list[p]
                detections.append(det)

            return detections

        except Exception as e:
            self.get_logger().error(f"NLF inference failed: {e}", throttle_duration_sec=2.0)
            return []

    # ── Torso centroid (for track assignment) ──────────────────────────────────

    def _compute_torso_centroid(self, joints3d, conf):
        """3D centroid of SMPL_TORSO_CENTROID joints. Returns np.array([x,y,z]) or None."""
        pts = []
        for i in SMPL_TORSO_CENTROID:
            if conf[i] < self._conf_thr:
                continue
            X, Y, Z = float(joints3d[i, 0]), float(joints3d[i, 1]), float(joints3d[i, 2])
            Z += self.z_offset
            if Z > self._max_depth_m:
                continue
            pts.append(np.array([X, Y, Z], dtype=np.float64))
        return np.mean(pts, axis=0) if len(pts) >= 2 else None

    def _knee_angle_ok(self, hip, knee, ankle):
        """True if knee angle (hip→knee→ankle) is within [30°, 175°]."""
        if hip is None or knee is None or ankle is None:
            return True
        v1 = hip - knee
        v2 = ankle - knee
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return True
        c = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        ang = np.degrees(np.arccos(c))
        return self.KNEE_MIN_DEG <= ang <= self.KNEE_MAX_DEG

    def _adaptive_Q(self, kf, missing_count, joint_idx):
        """Scale Kalman Q by missing duration × body-part factor (SMPL-adapted)."""
        Q = kf.Q_base.copy()
        time_factor = min(1.0 + 0.15 * missing_count[joint_idx], 3.0)
        if joint_idx in SMPL_TORSO_BODY:
            part_factor = 0.7
        elif joint_idx in SMPL_LEGS:
            part_factor = 1.4
        elif joint_idx in SMPL_ARMS or joint_idx in SMPL_HANDS:
            part_factor = 1.2
        elif joint_idx in SMPL_HEAD_GROUP:
            part_factor = 1.8
        else:
            part_factor = 1.0
        kf.Q = Q * time_factor * part_factor

    # ── Main color callback ────────────────────────────────────────────────────

    def _cb_color(self, msg):
        """
        Inference → assign detections → Kalman update → target selection → publish.
        Mirrors YoloSkeletonNodeOrbbec.cb_color flow exactly.
        """
        # ── Frame-skip: only run NLF every N frames ───────────────────────
        self._frame_count += 1
        if self._frame_count % self._process_every != 0:
            return  # Kalman filter predicts on skipped frames

        if not self._nlf_ready and not self._nlf_stub_warned:
            self._init_model()
        if self.cam_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        detections = self._run_nlf_inference(img)
        now = time.monotonic()

        joints3d_all, conf_all = [], []
        for det in detections:
            joints3d_all.append(np.asarray(det["joints3d"], dtype=np.float64))
            conf_all.append(np.asarray(det.get("conf", np.ones(NUM_JOINTS)), dtype=np.float64))

        centroids = [
            self._compute_torso_centroid(j, c)
            for j, c in zip(joints3d_all, conf_all)
        ]

        matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(
            centroids, self.tracks, self._max_track_distance)

        for di, ti in matches:
            self._update_track(self.tracks[ti], joints3d_all[di], conf_all[di])
            self.tracks[ti].last_seen = now
            if centroids[di] is not None:
                self.tracks[ti].centroid = centroids[di]

        for ti in unmatched_tracks:
            self._predict_track(self.tracks[ti])

        self.tracks = [t for t in self.tracks
                       if (now - t.last_seen) < self._track_timeout]

        for di in unmatched_dets:
            if len(self.tracks) >= self._max_tracks:
                break
            new_track = NLFPersonTrack(self._next_track_id)
            self._next_track_id += 1
            self._update_track(new_track, joints3d_all[di], conf_all[di])
            new_track.last_seen = now
            if centroids[di] is not None:
                new_track.centroid = centroids[di]
            self.tracks.append(new_track)

        self._target_track_id, self._target_hysteresis_miss = _smpl_select_target(
            self.tracks,
            lying_angle_min=self._lying_angle_min,
            current_target_id=self._target_track_id,
            hysteresis_miss_count=self._target_hysteresis_miss,
            hysteresis_frames=self._hysteresis_frames,
        )

        target = next(
            (t for t in self.tracks if t.track_id == self._target_track_id), None)
        if target is not None:
            # Store vertices from the matching detection for mesh publishing
            target_det = None
            for det in detections:
                if det.get("track_id") == target.track_id:
                    target_det = det
                    break
            if target_det is None and len(detections) > 0:
                # Fallback: use first detection if track_id not set in detections
                target_det = detections[0]
            self._last_vertices = (
                target_det.get("vertices3d")
                if target_det is not None and target_det.get("vertices3d") is not None
                else None
            )
            self._publish_target_pose(target, msg.header.stamp)
        else:
            self._last_vertices = None
            self._publish_empty(msg.header.stamp)
        self._publish_all_markers(msg.header.stamp)

    # ── Per-track Kalman update ────────────────────────────────────────────────

    def _update_track(self, track, joints3d, conf):
        """Kalman update step using NLF 3D positions (already in camera space — no back-projection)."""
        track.visible = [False] * NUM_JOINTS
        pts = [None] * NUM_JOINTS

        for i in range(NUM_JOINTS):
            if i in SMPL_LEGS:
                damping = 0.5
            elif i in SMPL_ARMS:
                damping = 0.4
            elif i in SMPL_TORSO_BODY:
                damping = 0.2
            else:
                damping = self.vel_damping

            if conf[i] < self._conf_thr:
                continue

            X = float(joints3d[i, 0])
            Y = float(joints3d[i, 1])
            Z = float(joints3d[i, 2]) + self.z_offset
            if Z > self._max_depth_m:
                continue

            meas = np.array([X, Y, Z], dtype=np.float64)
            track.kf[i].predict(1.0)

            # Knee angle biomechanical validation
            if i == KNEE_LEFT and pts[HIP_LEFT] is not None and pts[ANKLE_LEFT] is not None:
                if not self._knee_angle_ok(pts[HIP_LEFT], meas, pts[ANKLE_LEFT]):
                    track.kf[i].predict(damping)
                    pts[i] = track.kf[i].get_position()
                    continue
            if i == KNEE_RIGHT and pts[HIP_RIGHT] is not None and pts[ANKLE_RIGHT] is not None:
                if not self._knee_angle_ok(pts[HIP_RIGHT], meas, pts[ANKLE_RIGHT]):
                    track.kf[i].predict(damping)
                    pts[i] = track.kf[i].get_position()
                    continue

            # Mahalanobis outlier rejection
            if track.kf[i].initialized:
                pred = track.kf[i].get_position()
                sigma = np.sqrt(np.trace(track.kf[i].P[0:3, 0:3]))
                threshold = 3.5 if i in SMPL_LEGS else 2.5
                if np.linalg.norm(meas - pred) < threshold * sigma:
                    track.kf[i].update(meas)
            else:
                track.kf[i].update(meas)

            track.visible[i] = True

        # Missing counts
        for i in range(NUM_JOINTS):
            if track.visible[i]:
                track.missing_count[i] = 0
            else:
                track.missing_count[i] += 1

        # Predict missing + collect positions
        for i in range(NUM_JOINTS):
            if not track.visible[i]:
                self._adaptive_Q(track.kf[i], track.missing_count, i)
                if i in SMPL_LEGS:
                    damp = 0.5
                elif i in SMPL_ARMS:
                    damp = 0.4
                elif i in SMPL_TORSO_BODY:
                    damp = 0.2
                else:
                    damp = self.vel_damping
                track.kf[i].predict(damp)
            else:
                track.kf[i].Q = track.kf[i].Q_base.copy()
            pts[i] = track.kf[i].get_position()

        # TORSO length constraint
        torso_idx = [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT]
        if all(pts[i] is not None for i in torso_idx):
            sh_mid = 0.5 * (pts[SHOULDER_LEFT] + pts[SHOULDER_RIGHT])
            hip_mid = 0.5 * (pts[HIP_LEFT] + pts[HIP_RIGHT])
            L = np.linalg.norm(sh_mid - hip_mid)
            if track.TORSO_len_ref is None:
                track.TORSO_len_ref = L
            else:
                track.TORSO_len_ref = 0.98 * track.TORSO_len_ref + 0.02 * L

        pts = _smpl_torso_length_constraint(pts, track.visible, track.TORSO_len_ref)

        # Head → NECK soft constraint (when head is predicted, not measured)
        if (pts[HEAD] is not None and pts[NECK] is not None
                and not track.visible[HEAD]):
            pts[HEAD] = pts[HEAD] + 0.55 * (pts[NECK] - pts[HEAD])

        track._cached_pts = pts
        return pts

    # ── Predict-only (unmatched track) ─────────────────────────────────────────

    def _predict_track(self, track):
        """Predict-only step for a track with no matching detection."""
        for i in range(NUM_JOINTS):
            if track.kf[i].initialized:
                self._adaptive_Q(track.kf[i], track.missing_count, i)
                if i in SMPL_LEGS:
                    damp = 0.5
                elif i in SMPL_ARMS:
                    damp = 0.4
                elif i in SMPL_TORSO_BODY:
                    damp = 0.2
                else:
                    damp = self.vel_damping
                track.kf[i].predict(damp)
                track.kf[i].Q = track.kf[i].Q_base.copy()
                track.missing_count[i] += 1
        track.visible = [False] * NUM_JOINTS
        track._cached_pts = [
            kf.get_position() if kf.initialized else None for kf in track.kf]

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _publish_empty(self, stamp):
        empty = PoseArray()
        empty.header.stamp = stamp
        empty.header.frame_id = "orbbec_color_optical_frame"
        self.pub_poses.publish(empty)

    def _publish_target_pose(self, track, stamp):
        """Publish PoseArray with 24 SMPL joints for the target person."""
        pa = PoseArray()
        pa.header.frame_id = "orbbec_color_optical_frame"
        pa.header.stamp = stamp
        for p in track._cached_pts:
            pose = Pose()
            if p is None:
                pose.position.x = pose.position.y = pose.position.z = float("nan")
            else:
                pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.pub_poses.publish(pa)

        # ── Mesh publishing ────────────────────────────────────────────────────
        if self._publish_mesh and self._last_vertices is not None:
            mesh_msg = PoseArray()
            mesh_msg.header.frame_id = "orbbec_color_optical_frame"
            mesh_msg.header.stamp = stamp
            dec = self._mesh_decimation
            for v in self._last_vertices[::dec]:
                pose = Pose()
                pose.position.x = float(v[0])
                pose.position.y = float(v[1])
                pose.position.z = float(v[2])
                pose.orientation.w = 1.0
                mesh_msg.poses.append(pose)
            self.pub_mesh.publish(mesh_msg)

    def _publish_all_markers(self, stamp):
        """MarkerArray: green (target) or grey (others) joints + bones. DELETE expired tracks."""
        ma = MarkerArray()
        current_ids = {t.track_id for t in self.tracks}

        for old_id in self._published_track_ids - current_ids:
            for offset in range(4):
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = "orbbec_color_optical_frame"
                m.ns = "multi_track"; m.id = old_id * 10 + offset
                m.action = Marker.DELETE
                ma.markers.append(m)

        for track in self.tracks:
            is_target = (track.track_id == self._target_track_id)
            pts = track._cached_pts
            base_id = track.track_id * 10
            r, g, b = (0.0, 1.0, 0.0) if is_target else (0.6, 0.6, 0.6)

            hdr = Header(stamp=stamp, frame_id="orbbec_color_optical_frame")

            # Visible joints
            jv = Marker()
            jv.header = hdr
            jv.ns = "multi_track"; jv.id = base_id + 0
            jv.type = Marker.SPHERE_LIST; jv.action = Marker.ADD
            jv.scale.x = jv.scale.y = jv.scale.z = 0.03
            jv.color.r, jv.color.g, jv.color.b, jv.color.a = r, g, b, 1.0

            # Predicted joints (dimmer)
            jp = Marker()
            jp.header = hdr
            jp.ns = "multi_track"; jp.id = base_id + 1
            jp.type = Marker.SPHERE_LIST; jp.action = Marker.ADD
            jp.scale.x = jp.scale.y = jp.scale.z = 0.03
            jp.color.r, jp.color.g, jp.color.b = r * 0.4, g * 0.4, b * 0.4 + 0.3
            jp.color.a = 0.5

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
            bn.header = hdr
            bn.ns = "multi_track"; bn.id = base_id + 2
            bn.type = Marker.LINE_LIST; bn.action = Marker.ADD
            bn.scale.x = 0.015
            bn.color.r, bn.color.g, bn.color.b, bn.color.a = r, g, b, 0.8
            for a, c in self.edges:
                if pts[a] is not None and pts[c] is not None:
                    bn.points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=float(pts[a][2])))
                    bn.points.append(Point(x=float(pts[c][0]), y=float(pts[c][1]), z=float(pts[c][2])))

            ma.markers.extend([jv, jp, bn])

            # TARGET label (or DELETE if not target)
            if is_target and pts[NECK] is not None:
                lbl = Marker()
                lbl.header = hdr
                lbl.ns = "multi_track"; lbl.id = base_id + 3
                lbl.type = Marker.TEXT_VIEW_FACING; lbl.action = Marker.ADD
                lbl.pose.position = Point(x=float(pts[NECK][0]), y=float(pts[NECK][1]) - 0.10,
                                          z=float(pts[NECK][2]))
                lbl.pose.orientation.w = 1.0
                lbl.scale.z = 0.12
                lbl.color.r, lbl.color.g, lbl.color.b, lbl.color.a = 0.0, 1.0, 0.0, 1.0
                lbl.text = "TARGET"
                ma.markers.append(lbl)
            else:
                lbl_del = Marker()
                lbl_del.header = hdr
                lbl_del.ns = "multi_track"; lbl_del.id = base_id + 3
                lbl_del.action = Marker.DELETE
                ma.markers.append(lbl_del)

        self._published_track_ids = current_ids
        self.pub_markers.publish(ma)


# ============================================================
#  Entry point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = NLFSkeletonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
