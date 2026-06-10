#!/usr/bin/env python3
"""
NLF Skeleton Node — drop-in replacement for yolo_skeleton_spot.py.
RGB-only (no depth), 24 SMPL joints natively, NO COCO→SMPL mapping.
Publishes on same ROS2 topics: /human_pose/points_3d, /human_pose/skeleton_markers.

Simplified: EMA smoothing replaces Kalman3D tracking — NLF gives accurate 3D joints
so complex multi-joint Kalman adds unnecessary complexity, latency, and drift.
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
from std_msgs.msg import Bool, Float32, Header

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


# ── Geometry helper ───────────────────────────────────────────────────────────

def _angle_between(a, b):
    """Angle between two 3D vectors in degrees."""
    an = a / np.linalg.norm(a)
    bn = b / np.linalg.norm(b)
    c = float(np.clip(np.dot(an, bn), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


# ============================================================
#                       NLF Skeleton Node
# ============================================================

class NLFSkeletonNode(Node):
    """ROS2 Node: NLF-based 3D human skeleton. Drop-in replacement for YoloSkeletonNodeOrbbec."""

    def __init__(self):
        super().__init__("nlf_skeleton_node")

        # ── Early-init guards ──────────────────────────────────────────────────
        self._nlf_ready = False
        self._last_color_msg = None
        self._streaming_paused = True

        # ── Burst streaming state ────────────────────────────────────────────
        self._burst_active = False
        self._burst_detection_count = 0
        self._burst_start_time = None
        self._latest_raw_torso = None  # stores raw torso joints for 1-detection fallback
        self._burst_conf_sum = 0.0     # sum of bbox_scores for confidence averaging
        self._burst_conf_count = 0     # number of bbox_scores accumulated

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
        self.declare_parameter("z_offset",               0.0)
        self.declare_parameter("lying_torso_angle_min", 65.0)
        self.declare_parameter("target_hysteresis_frames", 10)
        self.declare_parameter("process_every_n_frames", 5)
        self.declare_parameter("burst_min_detections", 2)
        self.declare_parameter("burst_timeout_s", 30.0)
        self.declare_parameter("burst_throttle_frames", 10)

        self._process_every = int(self.get_parameter("process_every_n_frames").value)
        self._burst_min_detections = int(self.get_parameter("burst_min_detections").value)
        self._burst_timeout_s = float(self.get_parameter("burst_timeout_s").value)
        self._burst_throttle_frames = int(self.get_parameter("burst_throttle_frames").value)
        self._frame_count = 0

        self._model_path     = str(self.get_parameter("model_path").value)
        self._device         = str(self.get_parameter("device").value)
        self._conf_thr       = float(self.get_parameter("conf_threshold").value)
        self._max_depth_m    = float(self.get_parameter("max_depth_m").value)
        self._imgsz          = int(self.get_parameter("imgsz").value)
        self._publish_mesh   = bool(self.get_parameter("publish_mesh").value)
        self._mesh_topic     = str(self.get_parameter("mesh_topic").value)
        self._mesh_decimation = int(self.get_parameter("mesh_decimation").value)
        self.z_offset            = float(self.get_parameter("z_offset").value)
        self._lying_angle_min    = float(self.get_parameter("lying_torso_angle_min").value)
        self._hysteresis_frames  = int(self.get_parameter("target_hysteresis_frames").value)

        self.num_joints = NUM_JOINTS
        self.bridge = CvBridge()

        # Subscriptions — RGB-only, NO depth (NLF gives 3D directly)
        self.sub_color = self.create_subscription(
            Image, "/orbbec/color/image_raw", self._cb_color, 10)
        self.sub_info = self.create_subscription(
            CameraInfo, "/orbbec/color/camera_info", self._cb_caminfo, 10)
        self.sub_trigger = self.create_subscription(
            Bool, '/nlf/trigger', self._cb_trigger, 10)

        # Publishers
        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10)
        if self._publish_mesh:
            self.pub_mesh = self.create_publisher(
                PoseArray, self._mesh_topic, 10)
        self.pub_nlf_prior = self.create_publisher(
            PoseArray, '/exposure/nlf_prior', 10)
        self.pub_nlf_confidence = self.create_publisher(
            Float32, '/exposure/nlf_confidence', 10)

        # ── EMA smoothing state ────────────────────────────────────────────────
        self._smoothed_kp: dict = {}   # keyed by NLF box ID → list of 24 numpy arrays
        self._ema_alpha = 0.4          # higher than YOLO (0.3) because NLF is more accurate

        # ── Target selection state ─────────────────────────────────────────────
        self._target_id = None
        self._hysteresis_miss = 0
        self._published_track_ids: set = set()
        self._last_vertices = None          # (N,3) or None — for mesh publishing
        self.edges = _SMPL_EDGES

        # NLF model (lazy init, stub until nlf_s_multi.torchscript is available)
        self._nlf_model = None
        self._nlf_stub_warned = False

        self.get_logger().info(
            "\u2705 NLF skeleton node — 24 SMPL joints (RGB-only, EMA smoothing)")

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
            self._nlf_model = torch.jit.load(self._model_path, map_location='cpu')
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

    # ── Trigger callback (one-shot NLF prior on /nlf/trigger) ───────────────────

    def _cb_trigger(self, msg):
        """Burst trigger: Bool(True) starts burst streaming with EMA accumulation.
        Bool(False) pauses streaming (ignored during active burst)."""
        if not msg.data:
            if self._burst_active:
                self.get_logger().warn('Bool(False) ignored during active burst')
                return
            self._streaming_paused = True
            self.get_logger().info('NLF streaming paused')
            return

        # Bool(True) — start burst
        if self._burst_active:
            self.get_logger().warn('NLF burst already active — ignoring duplicate trigger')
            return
        if not self._nlf_ready:
            self.get_logger().warn('NLF trigger received but model not ready')
            return
        if self._last_color_msg is None:
            self.get_logger().warn('NLF trigger received but no image available')
            return

        # Activate burst
        self._streaming_paused = False
        self._burst_active = True
        self._burst_detection_count = 0
        self._burst_start_time = time.time()
        self._smoothed_kp = {}
        self._latest_raw_torso = None
        self._burst_conf_sum = 0.0
        self._burst_conf_count = 0
        self.get_logger().info('NLF burst started (target: 2 detections, timeout: 30s)')

    def _finish_burst(self):
        """Complete the burst: publish refined/raw/empty prior and auto-pause."""
        self._streaming_paused = True
        self._burst_active = False
        elapsed = time.time() - self._burst_start_time if self._burst_start_time else 0

        pa = PoseArray()
        pa.header.frame_id = "orbbec_color_optical_frame"
        stamp = self._last_color_msg.header.stamp if self._last_color_msg else self.get_clock().now().to_msg()
        pa.header.stamp = stamp

        if self._burst_detection_count >= 2:
            # EMA-refined skeleton from smoothed_kp
            if self._target_id is not None and self._target_id in self._smoothed_kp:
                pts = self._smoothed_kp[self._target_id]
                for j in range(NUM_JOINTS):
                    pose = Pose()
                    pose.position.x = float(pts[j][0])
                    pose.position.y = float(pts[j][1])
                    pose.position.z = float(pts[j][2])
                    pose.orientation.w = 1.0
                    pa.poses.append(pose)
            self.get_logger().info(
                f'NLF burst finished: {self._burst_detection_count} detections in {elapsed:.1f}s (EMA refined)'
            )
        elif self._burst_detection_count == 1 and self._latest_raw_torso is not None:
            # Single raw detection (no EMA)
            for j in range(NUM_JOINTS):
                pose = Pose()
                pose.position.x = float(self._latest_raw_torso[j][0])
                pose.position.y = float(self._latest_raw_torso[j][1])
                pose.position.z = float(self._latest_raw_torso[j][2])
                pose.orientation.w = 1.0
                pa.poses.append(pose)
            self.get_logger().info(
                f'NLF burst finished: 1 detection in {elapsed:.1f}s (raw, no EMA)'
            )
        else:
            # 0 detections — empty
            self.get_logger().warn(
                f'NLF burst finished: 0 detections in {elapsed:.1f}s (empty prior)'
            )

        self.pub_nlf_prior.publish(pa)

        # Publish mean bbox confidence
        conf_msg = Float32()
        conf_msg.data = self._burst_conf_sum / max(self._burst_conf_count, 1)
        self.pub_nlf_confidence.publish(conf_msg)
        self.get_logger().info(f'NLF burst confidence: {conf_msg.data:.3f}')

    # ── NLF inference ──────────────────────────────────────────────────────────

    def _run_nlf_inference(self, img_rgb):
        """
        Run NLF inference on RGB image (numpy H×W×3 uint8).
        Returns list[dict] per detected person:
          {'joints3d': (24,3) meters, 'conf': (24,) [0-1],
           'vertices3d': (6890,3) meters|None, 'box_id': int}
        """
        if not self._nlf_ready:
            self.get_logger().debug(
                'NLF inference skipped — model not loaded',
                throttle_duration_sec=10.0
            )
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

            # Box scores & tracking IDs from NLF's built-in YOLO+ByteTrack detector
            box_scores = np.ones(n_people, dtype=np.float64)
            box_ids = list(range(n_people))  # fallback: person index
            if pred.get('boxes') and len(pred['boxes']) > 0:
                boxes = pred['boxes'][0].cpu().numpy()  # (n_people, ≥5)
                if boxes.shape[0] == n_people:
                    box_scores = boxes[:, 4]  # confidence score
                    if boxes.shape[1] > 5:    # ByteTrack tracking ID available
                        box_ids = [int(b) for b in boxes[:, 5]]

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
                    'box_id': box_ids[p],
                }
                if vertices_list[p] is not None:
                    det['vertices3d'] = vertices_list[p]
                detections.append(det)

            return detections

        except Exception as e:
            self.get_logger().error(f"NLF inference failed: {e}", throttle_duration_sec=2.0)
            return []

    # ── Main color callback ────────────────────────────────────────────────────

    def _cb_color(self, msg):
        """
        Inference → EMA smoothing → target selection → publish.
        Simple, direct pipeline — no Kalman filtering.
        """
        # ── Store last frame for trigger inference ────────────────────────
        self._last_color_msg = msg
        self._frame_count += 1

        # ── Burst throttle: prevent executor backlog on CPU ──────────────
        if self._burst_active and self._frame_count % self._burst_throttle_frames != 0:
            return

        # ── Pause guard: skip inference when streaming is paused ──────────
        if self._streaming_paused:
            self.get_logger().debug(
                f'NLF streaming paused — frame {self._frame_count} skipped',
                throttle_duration_sec=5.0
            )
            return

        # ── Frame-skip: only run NLF every N frames ───────────────────────
        if self._frame_count % self._process_every != 0:
            return

        if not self._nlf_ready and not self._nlf_stub_warned:
            self._init_model()
        if self.cam_info is None:
            self.get_logger().warn(
                'NLF waiting for camera info — /orbbec/color/camera_info not received yet',
                throttle_duration_sec=10.0
            )
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f'NLF CvBridge conversion failed: {e}', throttle_duration_sec=5.0)
            return
        detections = self._run_nlf_inference(img)

        # ── EMA smoothing per person ──────────────────────────────────────
        processed = []
        for det in detections:
            joints_m = det["joints3d"]     # (24, 3) numpy array, meters
            pid = det["box_id"]

            prev = self._smoothed_kp.get(pid)
            if prev is None:
                smoothed = [joints_m[j].copy() for j in range(NUM_JOINTS)]
            else:
                smoothed = []
                alpha = self._ema_alpha
                for j in range(NUM_JOINTS):
                    new_val = joints_m[j].copy()
                    smoothed.append(alpha * new_val + (1.0 - alpha) * prev[j])
            self._smoothed_kp[pid] = smoothed

            processed.append({
                "id": pid,
                "pts_3d": smoothed,
                "vertices3d": det.get("vertices3d"),
                "bbox_score": det.get("bbox_score", 1.0),
            })

        # Cleanup stale smoothed entries (people who left the frame)
        active_ids = {d["id"] for d in processed}
        for pid in list(self._smoothed_kp.keys()):
            if pid not in active_ids:
                del self._smoothed_kp[pid]

        # ── Target selection: closest lying person ────────────────────────
        lying = []
        for det in processed:
            pts = det["pts_3d"]
            sh_l = pts[SHOULDER_LEFT]
            sh_r = pts[SHOULDER_RIGHT]
            hi_l = pts[HIP_LEFT]
            hi_r = pts[HIP_RIGHT]
            if all(x is not None and not np.isnan(x[0]) for x in [sh_l, sh_r, hi_l, hi_r]):
                sh_mid = (sh_l + sh_r) / 2.0
                hi_mid = (hi_l + hi_r) / 2.0
                torso_vec = sh_mid - hi_mid
                angle = _angle_between(torso_vec, _UP)
                if angle > self._lying_angle_min:
                    depth = float(sh_mid[2])
                    lying.append((depth, det["id"]))

        # Hysteresis: keep current target through brief non-lying gaps
        lying.sort()  # closest first
        if lying:
            if self._target_id is not None and any(tid == self._target_id for _, tid in lying):
                self._hysteresis_miss = 0  # target still lying
            else:
                self._target_id = lying[0][1]
                self._hysteresis_miss = 0
        else:
            self._hysteresis_miss += 1
            if self._hysteresis_miss > self._hysteresis_frames:
                self._target_id = None

        # ── Burst detection counting ──────────────────────────────────────────
        if self._burst_active:
            target = next((d for d in processed if d["id"] == self._target_id), None)
            if target is None:
                self.get_logger().debug(
                    f'NLF burst: no target in frame {self._frame_count} '
                    f'(detections: {len(detections)})',
                    throttle_duration_sec=2.0
                )
            if target is not None:
                pts = target["pts_3d"]
                # Check 4 torso joints are valid (non-NaN)
                torso_valid = all(
                    not np.isnan(pts[j][0]) for j in [SPINE1, SPINE2, SPINE3, PELVIS]
                )
                if torso_valid:
                    self._burst_detection_count += 1
                    # Accumulate bbox_score for confidence averaging
                    bbox_score = target.get("bbox_score", 1.0)
                    self._burst_conf_sum += bbox_score
                    self._burst_conf_count += 1
                    # Store raw torso for 1-detection fallback
                    self._latest_raw_torso = [pts[j].copy() for j in range(NUM_JOINTS)]
                    self.get_logger().info(
                        f'NLF burst: detection {self._burst_detection_count}/{self._burst_min_detections}'
                    )

            # Check finish conditions
            if self._burst_detection_count >= self._burst_min_detections:
                self._finish_burst()
                return  # skip normal publish for this frame

            elapsed = time.time() - self._burst_start_time
            if elapsed > self._burst_timeout_s:
                self.get_logger().warn(f'NLF burst timeout ({elapsed:.1f}s)')
                self._finish_burst()
                return  # skip normal publish for this frame

        # ── Suppress publish during active burst ──────────────────────────────
        if self._burst_active:
            elapsed = time.time() - self._burst_start_time
            self.get_logger().debug(
                f'NLF burst in progress: {self._burst_detection_count}/'
                f'{self._burst_min_detections} detections, {elapsed:.1f}s elapsed',
                throttle_duration_sec=2.0
            )
            return  # skip _publish_target_pose, _publish_all_markers, mesh publish

        # ── Publish target pose ───────────────────────────────────────────
        target = next((d for d in processed if d["id"] == self._target_id), None)
        if target is not None:
            self._last_vertices = target.get("vertices3d")
            self._publish_target_pose(target["pts_3d"], msg.header.stamp)
        else:
            self._last_vertices = None
            self._publish_empty(msg.header.stamp)

        # ── Publish mesh (if available) ────────────────────────────────────
        if self._publish_mesh and self._last_vertices is not None:
            mesh_msg = PoseArray()
            mesh_msg.header.frame_id = "orbbec_color_optical_frame"
            mesh_msg.header.stamp = msg.header.stamp
            dec = self._mesh_decimation
            for v in self._last_vertices[::dec]:
                pose = Pose()
                pose.position.x = float(v[0])
                pose.position.y = float(v[1])
                pose.position.z = float(v[2])
                pose.orientation.w = 1.0
                mesh_msg.poses.append(pose)
            self.pub_mesh.publish(mesh_msg)

        # ── Publish all skeletons as markers ──────────────────────────────
        self._publish_all_markers(processed, self._target_id, msg.header.stamp)

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _publish_empty(self, stamp):
        empty = PoseArray()
        empty.header.stamp = stamp
        empty.header.frame_id = "orbbec_color_optical_frame"
        self.pub_poses.publish(empty)

    def _publish_target_pose(self, pts, stamp):
        """Publish PoseArray with 24 SMPL joints for the target person."""
        pa = PoseArray()
        pa.header.frame_id = "orbbec_color_optical_frame"
        pa.header.stamp = stamp
        for p in pts:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.pub_poses.publish(pa)

    def _publish_all_markers(self, detections, target_id, stamp):
        """MarkerArray: green (target) or grey (others) joints + bones. DELETE expired tracks."""
        ma = MarkerArray()
        current_ids = {d["id"] for d in detections}

        # DELETE markers for disappeared tracks
        for old_id in self._published_track_ids - current_ids:
            for offset in range(4):
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = "orbbec_color_optical_frame"
                m.ns = "multi_track"
                m.id = old_id * 10 + offset
                m.action = Marker.DELETE
                ma.markers.append(m)

        for det in detections:
            tid = det["id"]
            is_target = (tid == target_id)
            pts = det["pts_3d"]
            base_id = tid * 10
            r, g, b = (0.0, 1.0, 0.0) if is_target else (0.6, 0.6, 0.6)

            hdr = Header(stamp=stamp, frame_id="orbbec_color_optical_frame")

            # Visible joints (all EMA-smoothed — no Kalman prediction)
            jv = Marker()
            jv.header = hdr
            jv.ns = "multi_track"
            jv.id = base_id + 0
            jv.type = Marker.SPHERE_LIST
            jv.action = Marker.ADD
            jv.scale.x = jv.scale.y = jv.scale.z = 0.03
            jv.color.r, jv.color.g, jv.color.b, jv.color.a = r, g, b, 1.0

            # Predicted joints (empty — no Kalman, kept for marker ID consistency)
            jp = Marker()
            jp.header = hdr
            jp.ns = "multi_track"
            jp.id = base_id + 1
            jp.type = Marker.SPHERE_LIST
            jp.action = Marker.ADD
            jp.scale.x = jp.scale.y = jp.scale.z = 0.03
            jp.color.r, jp.color.g, jp.color.b = r * 0.4, g * 0.4, b * 0.4 + 0.3
            jp.color.a = 0.5

            for p in pts:
                pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                jv.points.append(pt)

            # Bones
            bn = Marker()
            bn.header = hdr
            bn.ns = "multi_track"
            bn.id = base_id + 2
            bn.type = Marker.LINE_LIST
            bn.action = Marker.ADD
            bn.scale.x = 0.015
            bn.color.r, bn.color.g, bn.color.b, bn.color.a = r, g, b, 0.8
            for a, c in self.edges:
                pa_pt = pts[a]
                pc_pt = pts[c]
                bn.points.append(Point(x=float(pa_pt[0]), y=float(pa_pt[1]), z=float(pa_pt[2])))
                bn.points.append(Point(x=float(pc_pt[0]), y=float(pc_pt[1]), z=float(pc_pt[2])))

            ma.markers.extend([jv, jp, bn])

            # TARGET label (or DELETE if not target)
            if is_target:
                sh_mid = (pts[SHOULDER_LEFT] + pts[SHOULDER_RIGHT]) / 2.0
                lbl = Marker()
                lbl.header = hdr
                lbl.ns = "multi_track"
                lbl.id = base_id + 3
                lbl.type = Marker.TEXT_VIEW_FACING
                lbl.action = Marker.ADD
                lbl.pose.position = Point(
                    x=float(sh_mid[0]), y=float(sh_mid[1]) - 0.10, z=float(sh_mid[2]))
                lbl.pose.orientation.w = 1.0
                lbl.scale.z = 0.12
                lbl.color.r, lbl.color.g, lbl.color.b, lbl.color.a = 0.0, 1.0, 0.0, 1.0
                lbl.text = "TARGET"
                ma.markers.append(lbl)
            else:
                lbl_del = Marker()
                lbl_del.header = hdr
                lbl_del.ns = "multi_track"
                lbl_del.id = base_id + 3
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
