#!/usr/bin/env python3
"""
NLF Torso Tracker — drop-in replacement for z1_yolo_torso_tracker.py

Uses NLF (Neural Localization Fields) for monocular 3D human pose estimation
from a single RGB image. No depth image required — NLF regresses 3D SMPL joints
directly from RGB pixels.

States:  IDLE → ESTIMATING → LOCKED   (same FSM as Z1YoloTorsoTracker)
Extra:   GUIDING, SCAN_IDLE, SCAN_COLLECTING, SCAN_POINT_LOCKED

Key differences from Z1YoloTorsoTracker:
  - SMPL 24-joint indices (16,17,1,2) instead of COCO (5,6,11,12)
  - No depth subscription — NLF provides 3D directly
  - Timer-driven FSM tick (20 Hz) instead of frame-synchronous callback
  - NLF model is a stub — replace with real torchscript when available

Published topics (EXACT SAME as z1_yolo_torso_tracker):
  /torso_target_ee          PoseStamped        (world frame, interpolated)
  /torso_target_ee_locked   PoseStamped        (world frame, locked target)
  /torso_tracker_state      String             (FSM state label)
  /torso_markers            MarkerArray        (visualization)
  /torso_scan_point         Float32MultiArray  (22 floats, SMPL indices)
  /torso_keypoint_conf      Float32MultiArray  (4 floats, SMPL torso kp conf)
  /exposure/body_keypoints  PoseArray          (24 Pose, SMPL joints)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, PoseArray
from std_msgs.msg import Bool, ColorRGBA, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np

import torch
import torchvision  # MANDATORY for NLF TorchScript model (YOLOv8x ops)

import cv2

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

# Reuse the same Kalman3D from the YOLO tracker (same 3D tracking logic)
from z1_vision.kalman_filter import Kalman3D

# SMPL 24-joint indices (single source of truth)
from spot_perception.sml_pose_indices import (
    SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT,
    HEAD, NECK, NUM_JOINTS,
)

# ── SMPL Torso keypoint group ──────────────────────────────────────────────
TORSO_KEYPOINTS = [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT]  # 16, 17, 1, 2

# In SMPL-24, face/head is represented by HEAD(=15).
# We also keep NECK(=12) for computing the body up-direction (fallback torso estimation).
FACE_KEYPOINTS = [HEAD]        # single head point for markers (yellow)
FACE_FALLBACK  = [HEAD, NECK]  # head + neck for up-direction fallback

TORSO_EDGES = [
    (SHOULDER_LEFT, SHOULDER_RIGHT),   # shoulders
    (SHOULDER_LEFT, HIP_LEFT),         # spine left
    (SHOULDER_RIGHT, HIP_RIGHT),       # spine right
    (HIP_LEFT, HIP_RIGHT),             # hips
]

STATE_COLORS = {
    'IDLE':       ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),
    'ESTIMATING': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),
    'LOCKED':     ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
    'GUIDING':    ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9),
    'SCANNING':   ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9),
}


class NLFTorsoTrackerNode(Node):
    """NLF-based torso tracker — drop-in replacement for Z1YoloTorsoTracker."""

    def __init__(self):
        super().__init__('nlf_torso_tracker')

        # ── Parameters (from nlf_torso_params.yaml) ──────────────────────
        self.declare_parameter('model_path',         'nlf_s_multi.torchscript')
        self.declare_parameter('device',             'cuda')
        self.declare_parameter('conf_threshold',     0.3)
        self.declare_parameter('max_depth_m',        2.0)
        self.declare_parameter('imgsz',              416)
        self.declare_parameter('num_joints',         24)
        self.declare_parameter('tracking_mode',      'torso')

        # ── Backward-compatible parameter aliases (match z1_yolo_torso_tracker) ──
        self.declare_parameter('conf_thr',           0.3)    # alias for conf_threshold
        self.declare_parameter('max_depth',          2.5)    # kept for compat
        self.declare_parameter('kf_process_noise',   0.00005)
        self.declare_parameter('kf_meas_noise',      1.0)
        self.declare_parameter('kf_vel_damping',     0.9)
        self.declare_parameter('min_detection_conf', 0.6)
        self.declare_parameter('min_keypoints',      3)
        self.declare_parameter('lock_stable_frames', 20)
        self.declare_parameter('lock_variance_thr',  0.005)
        self.declare_parameter('lock_stable_checks', 5)
        self.declare_parameter('lock_drift_thr',     0.25)
        self.declare_parameter('lock_drift_frames',  5)
        self.declare_parameter('recovery_frames',    10)
        self.declare_parameter('tracking_speed',     0.05)
        self.declare_parameter('use_face_fallback',  True)
        self.declare_parameter('chest_offset_from_shoulder', 0.15)
        self.declare_parameter('scan_min_frames',    8)
        self.declare_parameter('guidance_min_conf',      0.5)
        self.declare_parameter('guidance_recovery_frames', 15)

        # ── NLF-specific parameters ───────────────────────────────────────
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tick_rate_hz', 20.0)

        # ── Read parameters ───────────────────────────────────────────────
        self.conf_thr           = float(self.get_parameter('conf_thr').value)
        self.max_depth          = float(self.get_parameter('max_depth').value)
        self.imgsz              = int(self.get_parameter('imgsz').value)
        self.vel_damping        = float(self.get_parameter('kf_vel_damping').value)
        self.min_det_conf       = float(self.get_parameter('min_detection_conf').value)
        self.min_keypoints      = int(self.get_parameter('min_keypoints').value)
        self.lock_stable_frames = int(self.get_parameter('lock_stable_frames').value)
        self.lock_variance_thr  = float(self.get_parameter('lock_variance_thr').value)
        self.lock_stable_checks = int(self.get_parameter('lock_stable_checks').value)
        self.lock_drift_thr     = float(self.get_parameter('lock_drift_thr').value)
        self.lock_drift_frames  = int(self.get_parameter('lock_drift_frames').value)
        self.recovery_frames    = int(self.get_parameter('recovery_frames').value)
        self.tracking_speed     = float(self.get_parameter('tracking_speed').value)
        self.use_face_fallback  = bool(self.get_parameter('use_face_fallback').value)
        self.chest_offset_m     = float(self.get_parameter('chest_offset_from_shoulder').value)
        self._scan_min_frames   = int(self.get_parameter('scan_min_frames').value)
        self._guidance_min_conf = float(self.get_parameter('guidance_min_conf').value)
        self._guidance_recovery = int(self.get_parameter('guidance_recovery_frames').value)
        self._camera_frame      = str(self.get_parameter('camera_frame').value)
        tick_rate_hz            = float(self.get_parameter('tick_rate_hz').value)

        # ── NLF model ────────────────────────────────────────────────────
        self._model_path = str(self.get_parameter('model_path').value)
        self.nlf_device  = str(self.get_parameter('device').value)
        self.nlf_model = None

        try:
            self.get_logger().info(
                f"Loading NLF model: {self._model_path} on {self.nlf_device}")
            self.nlf_model = torch.jit.load(self._model_path)
            if self.nlf_device == 'cuda' and torch.cuda.is_available():
                self.nlf_model = self.nlf_model.cuda()
            else:
                self.nlf_model = self.nlf_model.cpu()
                if self.nlf_device == 'cuda':
                    self.get_logger().warn(
                        "CUDA requested but not available — falling back to CPU")
            self.nlf_model.eval()
            self.get_logger().info("NLF model loaded successfully")
        except Exception as e:
            self.get_logger().error(
                f"Failed to load NLF model ({e}). "
                "Place nlf_s_multi.torchscript in the workspace root and verify torch/torchvision are installed. "
                "Running in STUB mode (no detections)."
            )

        # ── Kalman filter ─────────────────────────────────────────────────
        self.kf = Kalman3D(
            dt=1.0 / tick_rate_hz,
            process_noise=float(self.get_parameter('kf_process_noise').value),
            measurement_noise=float(self.get_parameter('kf_meas_noise').value),
        )

        # ── Bridge + TF ───────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────
        # RGB image only — NLF regresses 3D from monocular RGB, no depth needed.
        self.sub_rgb = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._cb_image, 10)

        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self._cb_info, 1)

        # FSM external state (label/color — read-only)
        self.sub_fsm_state = self.create_subscription(
            String, '/torso_sm_state', self._cb_fsm_state, 10)

        # Reset tracker
        self.sub_tracker_reset = self.create_subscription(
            Bool, '/tracker_reset', self._cb_tracker_reset, 10)

        # Guidance mode
        self.sub_guidance_mode = self.create_subscription(
            Bool, '/tracker_guidance_mode', self._cb_guidance_mode, 10)

        # Scan mode commands
        self.sub_scan_mode = self.create_subscription(
            Bool, '/tracker_scan_mode', self._cb_scan_mode, 10)

        self.sub_scan_next = self.create_subscription(
            Bool, '/tracker_scan_next', self._cb_scan_next, 10)

        # Scan seed (multi-view fused estimate → direct LOCKED)
        self.sub_scan_seed = self.create_subscription(
            PointStamped, '/torso_scan_seed', self._cb_scan_seed, 10)

        # ── Publishers (EXACT SAME topic names and types as z1_yolo_torso_tracker) ──
        self.pub_torso_ee        = self.create_publisher(PoseStamped, '/torso_target_ee',        10)
        self.pub_torso_ee_locked = self.create_publisher(PoseStamped, '/torso_target_ee_locked', 10)
        self.pub_markers         = self.create_publisher(MarkerArray,  '/torso_markers',          10)
        self.pub_tracker_state   = self.create_publisher(String,       '/torso_tracker_state',    10)
        self.pub_scan_point      = self.create_publisher(Float32MultiArray, '/torso_scan_point',  10)
        self.pub_kp_conf         = self.create_publisher(Float32MultiArray, '/torso_keypoint_conf', 10)
        self.pub_body_kp         = self.create_publisher(PoseArray,    '/exposure/body_keypoints', 10)

        # ── Internal state (FSM) ──────────────────────────────────────────
        self.state            = 'IDLE'
        self.stable_counter   = 0
        self.recovery_counter = 0
        self.drift_counter    = 0
        self.position_history: list = []
        self.locked_target    = None   # world frame np.ndarray

        # ── Guidance mode state ────────────────────────────────────────────
        self._guidance_mode    = False
        self._guidance_counter = 0
        self._guidance_missed  = 0

        # ── Scan mode state ───────────────────────────────────────────────
        self._scan_mode        = False
        self._scan_state       = 'IDLE'  # IDLE | COLLECTING | POINT_LOCKED
        self._scan_valid       = 0
        self._scan_torso_world = None

        # Per-keypoint confidence and 3D positions (SMPL torso indices)
        self._last_kp_conf: dict = {}
        self._last_kp_3d_cam: dict = {}   # camera frame
        self._last_kp_3d_world: dict = {}  # world frame (computed on demand)

        # ── Tracking interpolation ────────────────────────────────────────
        self.tracking_current_pos = None

        # ── Latest detection from _cb_image ───────────────────────────────
        self._latest_detection: dict | None = None

        # ── Camera info ──────────────────────────────────────────────────
        self.cam_info = None

        # ── Tick timer ────────────────────────────────────────────────────
        self._last_tick = None
        tick_period = 1.0 / tick_rate_hz
        self._timer = self.create_timer(tick_period, self._tick)

        # FSM external label
        self.fsm_state_external = 'WAITING'

        self.get_logger().info(
            f'🚀 NLF Torso Tracker ready '
            f'(tick={tick_rate_hz:.0f}Hz, frame={self._camera_frame}, NLF=STUB).'
        )

    # ══════════════════════════════════════════════════════════════════════
    #  SUBSCRIPTION CALLBACKS
    # ══════════════════════════════════════════════════════════════════════

    def _cb_info(self, msg: CameraInfo):
        self.cam_info = msg

    def _cb_fsm_state(self, msg: String):
        self.fsm_state_external = msg.data

    def _cb_tracker_reset(self, msg: Bool):
        if not msg.data:
            return
        prev = self.state
        self._fsm_reset()
        self.get_logger().info(f'🔄 Tracker reset: {prev} → IDLE (FSM requested)')

    def _cb_guidance_mode(self, msg: Bool):
        self._guidance_mode = msg.data
        if msg.data:
            self._guidance_counter = 0
            self._guidance_missed  = 0
            self.get_logger().info('🎯 Guidance mode ON')
        else:
            if self.state == 'GUIDING':
                self.state = 'IDLE'
                self._publish_target_world(np.zeros(3))
            self.get_logger().info('🎯 Guidance mode OFF')

    def _cb_scan_mode(self, msg: Bool):
        if msg.data == self._scan_mode:
            return
        self._scan_mode = msg.data
        if msg.data:
            self._scan_state = 'IDLE'
            self._scan_valid = 0
            self.get_logger().info('🔍 Scan mode ON')
        else:
            self._scan_state = 'IDLE'
            self._scan_valid = 0
            self._fsm_reset()
            self.get_logger().info('🔍 Scan mode OFF → IDLE')

    def _cb_scan_next(self, msg: Bool):
        if not msg.data or not self._scan_mode:
            return
        self._scan_state = 'IDLE'
        self._scan_valid = 0
        self.get_logger().info('🔄 Scan next: reset point')

    def _cb_scan_seed(self, msg: PointStamped):
        if self._scan_mode:
            return
        seed = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.locked_target        = seed
        self.state                = 'LOCKED'
        self.drift_counter        = 0
        self.tracking_current_pos = None
        self.get_logger().info(
            f'🎯 Scan seed → LOCKED [{seed[0]:.3f}, {seed[1]:.3f}, {seed[2]:.3f}]'
        )

    # ══════════════════════════════════════════════════════════════════════
    #  IMAGE CALLBACK  (NLF inference + torso extraction)
    # ══════════════════════════════════════════════════════════════════════

    def _cb_image(self, msg: Image):
        """Process incoming RGB image through NLF, extract torso, store for _tick."""
        # ── Convert ROS image → numpy ─────────────────────────────────────
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # ── NLF inference (STUB) ──────────────────────────────────────────
        joints3d = self._nlf_infer(rgb, msg.header.stamp)
        # joints3d: np.ndarray (24, 3) in camera frame, or None if model unavailable

        if joints3d is None:
            self._latest_detection = None
            return

        # ── Extract torso joints ──────────────────────────────────────────
        torso_raw, kp_3d, n_valid, avg_conf = self._extract_torso(joints3d)

        # ── Extract all body keypoints (for /exposure/body_keypoints) ──────
        kp_all_3d = self._extract_all_body_keypoints(joints3d)

        # ── Extract guidance detection (any valid keypoint) ────────────────
        guidance_raw, guidance_n, guidance_conf = self._extract_guidance(joints3d)

        self._latest_detection = {
            'torso_raw':     torso_raw,
            'kp_3d':         kp_3d,
            'n_valid':       n_valid,
            'avg_conf':      avg_conf,
            'header':        msg.header,
            'kp_all_3d':     kp_all_3d,
            'guidance_raw':  guidance_raw,
            'guidance_n':    guidance_n,
            'guidance_conf': guidance_conf,
        }

    # ══════════════════════════════════════════════════════════════════════
    #  NLF INFERENCE  (STUB — replace with real model)
    # ══════════════════════════════════════════════════════════════════════

    def _nlf_infer(self, rgb: np.ndarray, stamp) -> np.ndarray | None:
        """
        Run NLF inference on an RGB image (numpy H×W×3 uint8).
        Returns (24, 3) in camera frame (meters) for the best detected person, or None.
        """
        if self.nlf_model is None:
            return None

        try:
            image_tensor = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0)

            with torch.inference_mode():
                pred = self.nlf_model.detect_smpl_batched(
                    image_tensor,
                    default_fov_degrees=55.0,
                    num_aug=1,
                    detector_threshold=self.conf_thr,
                    internal_batch_size=64,
                    suppress_implausible_poses=True,
                )

            if (not pred.get('joints3d') or len(pred['joints3d']) == 0
                    or len(pred['joints3d'][0]) == 0):
                return None

            joints_mm = pred['joints3d'][0]       # (n_people, 24, 3) mm
            joints_m = joints_mm.cpu().numpy() / 1000.0
            n_people = joints_m.shape[0]

            if n_people == 1:
                return joints_m[0]

            # Multi-person → pick highest-confidence detection
            best_idx = 0
            best_score = -1.0
            if pred.get('boxes') and len(pred['boxes']) > 0:
                boxes = pred['boxes'][0].cpu().numpy()  # (n_people, 5)
                if boxes.shape[0] == n_people:
                    best_idx = int(np.argmax(boxes[:, 4]))
                    best_score = float(boxes[best_idx, 4])

            self.get_logger().debug(
                f"NLF: {n_people} people, selected #{best_idx} (score={best_score:.2f})",
                throttle_duration_sec=2.0)
            return joints_m[best_idx]

        except Exception as e:
            self.get_logger().error(f"NLF inference failed: {e}", throttle_duration_sec=2.0)
            return None

    # ══════════════════════════════════════════════════════════════════════
    #  TORSO / KEYPOINT EXTRACTION
    # ══════════════════════════════════════════════════════════════════════

    def _extract_torso(self, joints3d: np.ndarray):
        """
        Extract torso centroid from NLF 3D joints (camera frame).

        Uses SMPL indices:
          SHOULDER_LEFT  = 16
          SHOULDER_RIGHT = 17
          HIP_LEFT       = 1
          HIP_RIGHT      = 2

        Primary method: mean of all 4 torso joints.
        Fallback:       shoulders + head/neck up-direction.
        """
        # TODO: When confidence scores are available from NLF, filter by threshold.
        # For now, assume all joints are valid if the model returned them.

        # Extract 3D positions for torso joints
        shoulder_pts = [joints3d[i] for i in (SHOULDER_LEFT, SHOULDER_RIGHT)
                        if not np.any(np.isnan(joints3d[i]))]
        hip_pts      = [joints3d[i] for i in (HIP_LEFT, HIP_RIGHT)
                        if not np.any(np.isnan(joints3d[i]))]

        # Assume uniform confidence of 1.0 for now (NLF stub has no confidence output)
        all_confs = [1.0] * (len(shoulder_pts) + len(hip_pts))

        # Build per-keypoint 3D dict for markers and scan
        kp_3d = {}
        for idx in TORSO_KEYPOINTS + FACE_FALLBACK:
            if not np.any(np.isnan(joints3d[idx])):
                kp_3d[idx] = joints3d[idx].tolist()

        # Store per-keypoint confidences (SMPL torso indices)
        self._last_kp_conf = {
            idx: 1.0 for idx in TORSO_KEYPOINTS
            if not np.any(np.isnan(joints3d[idx]))
        }
        self._last_kp_3d_cam = {
            idx: joints3d[idx].tolist() for idx in TORSO_KEYPOINTS
            if not np.any(np.isnan(joints3d[idx]))
        }

        # ── Primary: shoulders + hips ─────────────────────────────────────
        if len(shoulder_pts) >= 1 and len(hip_pts) >= 1:
            torso_pts = shoulder_pts + hip_pts
            return (np.mean(torso_pts, axis=0), kp_3d,
                    len(torso_pts), float(np.mean(all_confs)))

        # ── Fallback: shoulders + head/neck up-direction ──────────────────
        if self.use_face_fallback and len(shoulder_pts) >= 1:
            face_pts = [joints3d[i] for i in FACE_FALLBACK
                        if not np.any(np.isnan(joints3d[i]))]
            if len(face_pts) >= 1:
                shoulder_mid = np.mean(shoulder_pts, axis=0)
                face_center  = np.mean(face_pts, axis=0)
                up_dir = face_center - shoulder_mid
                up_norm = np.linalg.norm(up_dir)
                if up_norm > 0.01:
                    up_dir /= up_norm
                    torso_raw = shoulder_mid + self.chest_offset_m * (-up_dir)
                else:
                    torso_raw = shoulder_mid
                n_valid = len(shoulder_pts) + len(face_pts)
                self.get_logger().debug(
                    f'[NLF] fallback face: shoulder_mid→chest offset={self.chest_offset_m:.2f}m',
                    throttle_duration_sec=2.0)
                return torso_raw, kp_3d, n_valid, float(np.mean(all_confs))

        return None, kp_3d, 0, 0.0

    def _extract_guidance(self, joints3d: np.ndarray):
        """Extract centroid of ALL valid SMPL joints for rough body guidance."""
        valid_mask = ~np.any(np.isnan(joints3d), axis=1)  # (24,) bool
        valid_indices = np.where(valid_mask)[0]
        if len(valid_indices) < 1:
            return None, 0, 0.0

        pts = joints3d[valid_indices]
        kp_3d = {int(idx): joints3d[idx].tolist() for idx in valid_indices}
        n_valid = len(pts)
        avg_conf = 1.0  # TODO: use per-joint confidence when NLF provides it
        return np.mean(pts, axis=0), kp_3d, n_valid, avg_conf

    def _extract_all_body_keypoints(self, joints3d: np.ndarray) -> dict:
        """Extract all 24 SMPL joints in camera frame for /exposure/body_keypoints."""
        kp_3d = {}
        for idx in range(NUM_JOINTS):
            if not np.any(np.isnan(joints3d[idx])):
                kp_3d[idx] = joints3d[idx].tolist()
        return kp_3d

    # ══════════════════════════════════════════════════════════════════════
    #  TICK — FSM dispatch (20 Hz)
    # ══════════════════════════════════════════════════════════════════════

    def _tick(self):
        """Periodic FSM tick — reads latest detection, dispatches to state handlers."""

        # ── Adjust Kalman dt from real tick interval ───────────────────────
        now = self.get_clock().now()
        if self._last_tick is not None:
            dt = (now - self._last_tick).nanoseconds * 1e-9
            if 0.01 < dt < 0.5:
                self.kf.dt = dt
        self._last_tick = now

        det = self._latest_detection

        # ── Guidance mode ──────────────────────────────────────────────────
        if self._guidance_mode and not self._scan_mode:
            if det is not None and det['guidance_raw'] is not None:
                self._update_guidance(det['guidance_raw'], det['guidance_n'],
                                      det['guidance_conf'], det['header'])
            else:
                self._update_guidance(None, 0, 0.0, None)
            return

        # ── Extract current detection ──────────────────────────────────────
        torso_raw = det['torso_raw'] if det else None
        kp_3d     = det.get('kp_3d', {}) if det else {}
        n_valid   = det['n_valid'] if det else 0
        avg_conf  = det['avg_conf'] if det else 0.0
        header    = det.get('header') if det else None

        # ── Scan mode ──────────────────────────────────────────────────────
        if self._scan_mode:
            self._update_scan(torso_raw, n_valid, avg_conf)
            kp_all = det.get('kp_all_3d', {}) if det else {}
            if kp_all:
                self._publish_body_keypoints(kp_all)
        else:
            self._update_state(torso_raw, n_valid, avg_conf, header)

        # ── Markers ────────────────────────────────────────────────────────
        if self._scan_mode:
            target = self._scan_torso_world
        else:
            target = self.locked_target
            if target is None:
                target = (self._camera_to_world(self.kf.get_position())
                          if self.kf.initialized else None)

        if target is not None:
            self._publish_markers(target, kp_3d, header)

    # ══════════════════════════════════════════════════════════════════════
    #  GUIDANCE MODE
    # ══════════════════════════════════════════════════════════════════════

    def _update_guidance(self, torso_raw, n_valid, avg_conf, header):
        prev_state = self.state

        if torso_raw is not None and n_valid >= 2:
            self.state = 'GUIDING'
            self._guidance_missed = 0
            self._guidance_counter += 1
            torso_world = self._camera_to_world(torso_raw)
            if torso_world is not None:
                self._publish_target_world(torso_world)
        else:
            if self.state == 'GUIDING':
                self._guidance_missed += 1
                if self._guidance_missed >= self._guidance_recovery:
                    self.state = 'IDLE'
                    self._guidance_counter = 0
                    self.get_logger().info('Guidance lost → IDLE')

        if self.state != prev_state:
            self.get_logger().info(
                f'🎯 Guidance: {prev_state} → {self.state} '
                f'(kp={n_valid} conf={avg_conf:.2f})')

        self.pub_tracker_state.publish(String(data=self.state))

    # ══════════════════════════════════════════════════════════════════════
    #  SCAN MODE
    # ══════════════════════════════════════════════════════════════════════

    def _update_scan(self, torso_raw, n_valid, avg_conf):
        """
        Scan mode FSM: IDLE → COLLECTING → POINT_LOCKED.

        Publishes /torso_scan_point with 22 floats using SMPL indices:
          [score, n_kp, conf, x, y, z,
           kp16_conf, kp17_conf, kp1_conf, kp2_conf,
           kp16_x, kp16_y, kp16_z,
           kp17_x, kp17_y, kp17_z,
           kp1_x, kp1_y, kp1_z,
           kp2_x, kp2_y, kp2_z]
        """
        TORSO_KP_MAX = 4

        valid = (torso_raw is not None
                 and n_valid >= self.min_keypoints
                 and avg_conf >= self.min_det_conf)

        if valid:
            per_frame_score = (min(n_valid, TORSO_KP_MAX) / TORSO_KP_MAX) * avg_conf
            torso_world = self._camera_to_world(torso_raw)
        else:
            per_frame_score = 0.0
            torso_world = None

        if torso_world is not None:
            self._scan_torso_world = torso_world

        # ── Per-keypoint confidences (SMPL indices: 16, 17, 1, 2) ──────────
        kp16_c = float(self._last_kp_conf.get(SHOULDER_LEFT,  0.0))
        kp17_c = float(self._last_kp_conf.get(SHOULDER_RIGHT, 0.0))
        kp1_c  = float(self._last_kp_conf.get(HIP_LEFT,       0.0))
        kp2_c  = float(self._last_kp_conf.get(HIP_RIGHT,      0.0))

        # ── Per-keypoint 3D world positions (SMPL indices) ─────────────────
        def _kp_world(idx):
            cam = self._last_kp_3d_cam.get(idx)
            if cam is None:
                return [0.0, 0.0, 0.0]
            w = self._camera_to_world(np.array(cam))
            if w is None:
                return [0.0, 0.0, 0.0]
            return [float(w[0]), float(w[1]), float(w[2])]

        kp16_xyz = _kp_world(SHOULDER_LEFT)
        kp17_xyz = _kp_world(SHOULDER_RIGHT)
        kp1_xyz  = _kp_world(HIP_LEFT)
        kp2_xyz  = _kp_world(HIP_RIGHT)

        # ── Build and publish 22-float array ───────────────────────────────
        if torso_world is not None:
            data = [per_frame_score, float(n_valid), avg_conf,
                    float(torso_world[0]), float(torso_world[1]), float(torso_world[2]),
                    kp16_c, kp17_c, kp1_c, kp2_c,
                    *kp16_xyz, *kp17_xyz, *kp1_xyz, *kp2_xyz]
        else:
            data = [0.0] * 22
        msg = Float32MultiArray()
        msg.data = data
        self.pub_scan_point.publish(msg)

        # ── Update scan FSM ────────────────────────────────────────────────
        prev_scan = self._scan_state
        if self._scan_state == 'IDLE':
            if valid:
                self._scan_state = 'COLLECTING'
                self._scan_valid = 1
        elif self._scan_state == 'COLLECTING':
            if valid:
                self._scan_valid += 1
                if self._scan_valid >= self._scan_min_frames:
                    self._scan_state = 'POINT_LOCKED'
                    self.get_logger().info(
                        f'🔒 SCAN_POINT_LOCKED ({self._scan_valid} valid frames)')
            else:
                self._scan_valid = max(0, self._scan_valid - 1)
                if self._scan_valid == 0:
                    self._scan_state = 'IDLE'

        scan_label = f'SCAN_{self._scan_state}'
        if self._scan_state != prev_scan:
            self.get_logger().info(f'🔄 Scan state: {prev_scan} → {self._scan_state}')
        self.pub_tracker_state.publish(String(data=scan_label))

    # ══════════════════════════════════════════════════════════════════════
    #  MAIN FSM  (IDLE → ESTIMATING → LOCKED)
    # ══════════════════════════════════════════════════════════════════════

    def _update_state(self, torso_raw, n_valid, avg_conf, header):
        """Core FSM — identical logic to Z1YoloTorsoTracker._update_state."""
        prev_state = self.state

        # ── GUIDING fallback ───────────────────────────────────────────────
        if self.state == 'GUIDING':
            self.state = 'IDLE'
            self._guidance_counter = 0
            self._guidance_missed  = 0

        # ── IDLE ───────────────────────────────────────────────────────────
        if self.state == 'IDLE':
            if (torso_raw is not None
                    and n_valid >= self.min_keypoints
                    and avg_conf >= self.min_det_conf):
                self.state            = 'ESTIMATING'
                self.kf.reset()
                self.kf.initialize(torso_raw)
                self.position_history = [torso_raw.copy()]
                self.stable_counter   = 0
                self.tracking_current_pos = None

        # ── ESTIMATING ─────────────────────────────────────────────────────
        elif self.state == 'ESTIMATING':
            if (torso_raw is not None
                    and n_valid >= self.min_keypoints
                    and avg_conf >= self.min_det_conf):

                self.recovery_counter = 0

                self.kf.predict(self.vel_damping)
                self.kf.update(torso_raw)
                estimated_cam = self.kf.get_position()

                self.position_history.append(estimated_cam.copy())
                if len(self.position_history) > self.lock_stable_frames:
                    self.position_history.pop(0)

                target_world = self._camera_to_world(estimated_cam)
                if target_world is not None:
                    interpolated = self._interpolate_to_target(target_world)
                    self._publish_target_world(interpolated)
                    self._publish_kp_conf()

                if len(self.position_history) >= self.lock_stable_frames:
                    variance = np.var(self.position_history, axis=0).sum()
                    if variance < self.lock_variance_thr:
                        self.stable_counter += 1
                        if self.stable_counter >= self.lock_stable_checks:
                            locked_world = self._camera_to_world(estimated_cam)
                            if locked_world is not None:
                                self.locked_target        = locked_world
                                self.tracking_current_pos = None
                                self.state                = 'LOCKED'
                                self.drift_counter        = 0
                    else:
                        self.stable_counter = 0
            else:
                self.recovery_counter += 1
                if self.recovery_counter >= self.recovery_frames:
                    self.get_logger().warn(
                        f'⚠️ Torso lost during estimation '
                        f'({self.recovery_counter} consecutive frames) → IDLE')
                    self.state                = 'IDLE'
                    self.kf.reset()
                    self.position_history     = []
                    self.tracking_current_pos = None
                    self.recovery_counter     = 0
                else:
                    self.kf.predict(self.vel_damping)
                    self.get_logger().debug(
                        f'[ESTIMATING] recovery {self.recovery_counter}/{self.recovery_frames}',
                        throttle_duration_sec=0.5)

        # ── LOCKED ─────────────────────────────────────────────────────────
        elif self.state == 'LOCKED':
            if self.locked_target is None:
                self.get_logger().warn("LOCKED but locked_target is None", throttle_duration_sec=1.0)
                return
            interpolated = self._interpolate_to_target(self.locked_target)
            self._publish_target_world(interpolated)
            self._publish_target_world_locked(self.locked_target)
            self._publish_kp_conf()

            ok_det = (torso_raw is not None
                      and n_valid >= self.min_keypoints
                      and avg_conf >= self.min_det_conf)
            if ok_det and self.locked_target is not None:
                torso_world = self._camera_to_world(torso_raw)
                if torso_world is not None:
                    dist = float(np.linalg.norm(torso_world - self.locked_target))
                    if dist > self.lock_drift_thr:
                        self.drift_counter += 1
                    else:
                        self.drift_counter = 0
                    if self.drift_counter >= self.lock_drift_frames:
                        self.get_logger().info(
                            f'🔓 UNLOCK: torso moved {dist:.3f}m '
                            f'(> {self.lock_drift_thr:.3f}m) → ESTIMATING')
                        self.state                = 'ESTIMATING'
                        self.locked_target        = None
                        self.tracking_current_pos = None
                        self.position_history     = [torso_raw.copy()]
                        self.stable_counter       = 0
                        self.drift_counter        = 0
                        self.kf.reset()
                        self.kf.initialize(torso_raw)

        # ── Log state change ───────────────────────────────────────────────
        if self.state != prev_state:
            self.get_logger().info(
                f'🔄 {prev_state} → {self.state} (kp={n_valid} conf={avg_conf:.2f})')

        self.pub_tracker_state.publish(String(data=self.state))

    # ══════════════════════════════════════════════════════════════════════
    #  UTILS: TF, interpolation, publishing
    # ══════════════════════════════════════════════════════════════════════

    def _camera_to_world(self, point_camera: np.ndarray) -> np.ndarray | None:
        """Transform a 3D point from camera frame to world frame."""
        pt = PointStamped()
        pt.header.frame_id = self._camera_frame
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x = float(point_camera[0])
        pt.point.y = float(point_camera[1])
        pt.point.z = float(point_camera[2])
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', self._camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            transformed = do_transform_point(pt, transform)
            return np.array([transformed.point.x, transformed.point.y, transformed.point.z])
        except TransformException as e:
            self.get_logger().warn(
                f'TF {self._camera_frame}→world failed: {e}', throttle_duration_sec=2.0)
            return None

    def _interpolate_to_target(self, target_world: np.ndarray) -> np.ndarray:
        """Smoothly interpolate current tracking position toward target."""
        if self.tracking_current_pos is None:
            self.tracking_current_pos = target_world.copy()

        direction = target_world - self.tracking_current_pos
        distance  = np.linalg.norm(direction)

        dt       = self.kf.dt if hasattr(self.kf, 'dt') else 0.033
        max_step = self.tracking_speed * dt

        if distance <= max_step:
            self.tracking_current_pos = target_world.copy()
        else:
            self.tracking_current_pos = (
                self.tracking_current_pos + (direction / distance) * max_step)
        return self.tracking_current_pos

    def _fsm_reset(self):
        """Reset all FSM state to IDLE."""
        self.state                = 'IDLE'
        self.locked_target        = None
        self.tracking_current_pos = None
        self.position_history     = []
        self.stable_counter       = 0
        self.drift_counter        = 0
        self.recovery_counter     = 0
        self.kf.reset()
        self._guidance_counter    = 0
        self._guidance_missed     = 0

    # ── Publishers ─────────────────────────────────────────────────────────

    def _publish_target_world(self, point_world: np.ndarray):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee.publish(pose)

    def _publish_target_world_locked(self, point_world: np.ndarray):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee_locked.publish(pose)

    def _publish_kp_conf(self):
        """Publish 4-float torso keypoint confidences (SMPL indices: 16, 17, 1, 2)."""
        msg = Float32MultiArray()
        msg.data = [
            float(self._last_kp_conf.get(SHOULDER_LEFT,  0.0)),
            float(self._last_kp_conf.get(SHOULDER_RIGHT, 0.0)),
            float(self._last_kp_conf.get(HIP_LEFT,       0.0)),
            float(self._last_kp_conf.get(HIP_RIGHT,      0.0)),
        ]
        self.pub_kp_conf.publish(msg)

    def _publish_body_keypoints(self, kp_3d_cam: dict):
        """Publish all 24 SMPL joints as PoseArray in world frame."""
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.get_clock().now().to_msg()
        for idx in range(NUM_JOINTS):
            pose = Pose()
            if idx in kp_3d_cam:
                w = self._camera_to_world(np.array(kp_3d_cam[idx]))
                if w is not None:
                    pose.position.x = float(w[0])
                    pose.position.y = float(w[1])
                    pose.position.z = float(w[2])
                    pose.orientation.w = 1.0
                else:
                    pose.position.x = float('nan')
                    pose.position.y = float('nan')
                    pose.position.z = float('nan')
            else:
                pose.position.x = float('nan')
                pose.position.y = float('nan')
                pose.position.z = float('nan')
            pa.poses.append(pose)
        self.pub_body_kp.publish(pa)

    # ══════════════════════════════════════════════════════════════════════
    #  MARKERS
    # ══════════════════════════════════════════════════════════════════════

    def _publish_markers(self, torso_center_world: np.ndarray,
                         kp_3d: dict, header):
        """Publish RViz markers: center sphere, keypoint spheres, edges, state text."""
        kp_3d_w = {idx: self._camera_to_world(np.array(pt))
                   for idx, pt in kp_3d.items()}
        kp_3d_w = {idx: w for idx, w in kp_3d_w.items() if w is not None}

        frame   = 'world'
        stamp   = self.get_clock().now().to_msg()
        markers = MarkerArray()
        color   = (STATE_COLORS['SCANNING'] if self._scan_mode
                   else STATE_COLORS.get(self.state, STATE_COLORS['IDLE']))

        # 1. Center sphere
        cm = Marker()
        cm.header.frame_id    = frame
        cm.header.stamp       = stamp
        cm.ns                 = 'torso_center'
        cm.id                 = 0
        cm.type               = Marker.SPHERE
        cm.action             = Marker.ADD
        cm.pose.position.x    = float(torso_center_world[0])
        cm.pose.position.y    = float(torso_center_world[1])
        cm.pose.position.z    = float(torso_center_world[2])
        cm.pose.orientation.w = 1.0
        cm.scale.x = cm.scale.y = cm.scale.z = 0.08
        cm.color              = color
        cm.lifetime.nanosec   = 200_000_000
        markers.markers.append(cm)

        # 2a. Torso keypoint spheres (blue)
        for i, idx in enumerate(TORSO_KEYPOINTS):
            if idx not in kp_3d_w:
                continue
            kp = kp_3d_w[idx]
            m = Marker()
            m.header.frame_id    = frame
            m.header.stamp       = stamp
            m.ns                 = 'torso_keypoints'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(kp[0])
            m.pose.position.y    = float(kp[1])
            m.pose.position.z    = float(kp[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color   = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            m.lifetime.nanosec   = 200_000_000
            markers.markers.append(m)

        # 2b. Face keypoint sphere (HEAD, yellow)
        for i, idx in enumerate(FACE_KEYPOINTS):
            if idx not in kp_3d_w:
                continue
            kp = kp_3d_w[idx]
            m = Marker()
            m.header.frame_id    = frame
            m.header.stamp       = stamp
            m.ns                 = 'face_keypoints'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(kp[0])
            m.pose.position.y    = float(kp[1])
            m.pose.position.z    = float(kp[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.03
            m.color   = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.8)
            m.lifetime.nanosec   = 200_000_000
            markers.markers.append(m)

        # 3. Torso edge lines
        lm = Marker()
        lm.header.frame_id    = frame
        lm.header.stamp       = stamp
        lm.ns                 = 'torso_edges'
        lm.id                 = 10
        lm.type               = Marker.LINE_LIST
        lm.action             = Marker.ADD
        lm.scale.x            = 0.015
        lm.color              = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        lm.pose.orientation.w = 1.0
        lm.lifetime.nanosec   = 200_000_000
        for (a, b) in TORSO_EDGES:
            if a in kp_3d_w and b in kp_3d_w:
                lm.points.append(Point(x=float(kp_3d_w[a][0]),
                                       y=float(kp_3d_w[a][1]),
                                       z=float(kp_3d_w[a][2])))
                lm.points.append(Point(x=float(kp_3d_w[b][0]),
                                       y=float(kp_3d_w[b][1]),
                                       z=float(kp_3d_w[b][2])))
        if lm.points:
            markers.markers.append(lm)

        # 4. Center-to-keypoint spokes
        sm = Marker()
        sm.header.frame_id    = frame
        sm.header.stamp       = stamp
        sm.ns                 = 'torso_spokes'
        sm.id                 = 11
        sm.type               = Marker.LINE_LIST
        sm.action             = Marker.ADD
        sm.scale.x            = 0.008
        sm.color              = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.6)
        sm.pose.orientation.w = 1.0
        sm.lifetime.nanosec   = 200_000_000
        pc = Point(x=float(torso_center_world[0]),
                   y=float(torso_center_world[1]),
                   z=float(torso_center_world[2]))
        for idx in TORSO_KEYPOINTS:
            if idx in kp_3d_w:
                sm.points.append(pc)
                sm.points.append(Point(x=float(kp_3d_w[idx][0]),
                                       y=float(kp_3d_w[idx][1]),
                                       z=float(kp_3d_w[idx][2])))
        if sm.points:
            markers.markers.append(sm)

        # 5. State text label
        tm = Marker()
        tm.header.frame_id    = frame
        tm.header.stamp       = stamp
        tm.ns                 = 'torso_state'
        tm.id                 = 20
        tm.type               = Marker.TEXT_VIEW_FACING
        tm.action             = Marker.ADD
        tm.pose.position.x    = float(torso_center_world[0])
        tm.pose.position.y    = float(torso_center_world[1])
        tm.pose.position.z    = float(torso_center_world[2]) + 0.15
        tm.pose.orientation.w = 1.0
        tm.scale.z            = 0.05
        tm.color              = color
        tm.text               = f"INT:{self.state} | EXT:{self.fsm_state_external}"
        tm.lifetime.nanosec   = 200_000_000
        markers.markers.append(tm)

        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = NLFTorsoTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
