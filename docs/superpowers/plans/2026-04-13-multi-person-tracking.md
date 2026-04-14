# Multi-Person Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single-person Kalman tracker in `yolo_skeleton_spot.py` with a multi-person tracker that maintains separate Kalman filters per person, never mixes keypoints across people, and selects the LYING person closest to the camera as the target.

**Architecture:** Pure logic (`PersonTrack`, assignment, target selection) extracted to a new `person_tracking.py` (no ROS2 imports → unit-testable). The ROS2 node in `yolo_skeleton_spot.py` imports and uses these primitives. Downstream nodes (`posture_classifier.py`, `laying_human_detector.py`) are unchanged — they still receive the same single-person `/human_pose/points_3d` topic.

**Tech Stack:** Python 3, NumPy, YOLO (ultralytics), ROS2 Humble

---

## File Map

| Action | File | Responsibility |
|--------|------|----------------|
| Create | `src/spot_perception/spot_perception/person_tracking.py` | `PersonTrack`, `assign_detections_to_tracks`, `select_target`, `TORSO_length_constraint`, `Kalman3D` — no ROS2 |
| Modify | `src/spot_perception/spot_perception/yolo_skeleton_spot.py` | Multi-track `cb_color`, helpers, publishers |
| Create | `src/spot_perception/test/test_multi_person_tracking.py` | Unit tests for pure logic |
| Modify | `src/spot_perception/launch/spot_perception.launch.py` | Expose new parameters |

---

## Task 1: Create `person_tracking.py` with `PersonTrack`

**Files:**
- Create: `src/spot_perception/spot_perception/person_tracking.py`
- Create: `src/spot_perception/test/test_multi_person_tracking.py`

- [ ] **Step 1: Create `person_tracking.py`**

```python
# src/spot_perception/spot_perception/person_tracking.py
import time
import numpy as np


# ── Joint group sets (COCO 17-joint) ──────────────────────────
TORSO_JOINTS = {5, 6, 11, 12}
ARM_JOINTS   = {7, 8, 9, 10}
LEG_JOINTS   = {13, 14, 15, 16}
NOSE_JOINTS  = {0}
SKIP_JOINTS  = {1, 2, 3, 4}   # eyes / ears — not tracked

_UP = np.array([0.0, -1.0, 0.0], dtype=np.float64)  # camera optical: Y down → world-up = -Y


# ── Kalman3D ──────────────────────────────────────────────────
class Kalman3D:
    """3D Kalman filter [x,y,z,vx,vy,vz]."""

    def __init__(self, dt=1/15, q=0.2, r=0.002, p0=1.0):
        self.dt = float(dt)
        self.x = np.zeros((6, 1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * p0
        self.F = np.eye(6, dtype=np.float64)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.Q_base = np.eye(6, dtype=np.float64) * q
        self.Q = self.Q_base.copy()
        self.R = np.eye(3, dtype=np.float64) * r
        self.initialized = False

    def predict(self, vel_damping=1.0):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[3:, 0] *= float(vel_damping)

    def update(self, z):
        z = np.asarray(z, dtype=np.float64).reshape(3, 1)
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
        return self.x[0:3, 0].copy()

    def set_position(self, p):
        self.x[0:3, 0] = np.asarray(p, dtype=np.float64).reshape(3)


# ── PersonTrack ───────────────────────────────────────────────
class PersonTrack:
    """One tracked person: 17 independent Kalman filters + tracking metadata."""

    def __init__(self, track_id: int):
        self.track_id = track_id
        self.kf = [Kalman3D() for _ in range(17)]
        self.visible = [False] * 17
        self.missing_count = [0] * 17
        self.TORSO_len_ref = None
        self.centroid = None        # np.array([x,y,z]) — mean of torso joints, updated each frame
        self.last_seen: float = time.monotonic()

        # Per-joint Q/R tuning
        for i, kf in enumerate(self.kf):
            if i in TORSO_JOINTS:
                kf.Q *= 0.7;  kf.R *= 0.7
            elif i in LEG_JOINTS:
                kf.Q *= 0.9;  kf.R *= 0.9
            elif i in NOSE_JOINTS:
                kf.Q *= 1.5;  kf.R *= 1.5
            # ARM_JOINTS: factor 1.0 — no change


# ── TORSO length constraint ───────────────────────────────────
def TORSO_length_constraint(pts, visible, L_ref, stiffness=0.35):
    """Soft constraint: keep shoulder-hip distance close to reference length L_ref."""
    if L_ref is None:
        return pts
    idx = [5, 6, 11, 12]
    if any(pts[i] is None for i in idx):
        return pts
    if all(visible[i] for i in idx):
        return pts   # all visible — no correction needed
    sh_mid  = 0.5 * (pts[5] + pts[6])
    hip_mid = 0.5 * (pts[11] + pts[12])
    v = sh_mid - hip_mid
    dist = np.linalg.norm(v)
    if dist < 1e-6:
        return pts
    v_corr = (v / dist) * L_ref
    target_sh_mid = hip_mid + v_corr
    delta = target_sh_mid - sh_mid
    pts[5] += stiffness * delta
    pts[6] += stiffness * delta
    return pts


# ── Assignment ────────────────────────────────────────────────
def assign_detections_to_tracks(detection_centroids, tracks, max_dist):
    """
    Greedy nearest-neighbour assignment of YOLO detections to existing tracks.

    Args:
        detection_centroids: list of np.array([x,y,z]) or None — one per YOLO detection.
                             None means depth was unavailable for that detection.
        tracks:   list of PersonTrack
        max_dist: float — max centroid distance (metres) to accept a match

    Returns:
        matches:          list of (det_idx, track_idx)
        unmatched_dets:   list of det_idx that have a valid centroid but no matching track
        unmatched_tracks: list of track_idx with no matching detection this frame
    """
    if not tracks:
        unmatched_dets = [i for i, c in enumerate(detection_centroids) if c is not None]
        return [], unmatched_dets, []

    # Build sorted cost list (distance, det_idx, track_idx)
    costs = []
    for di, dc in enumerate(detection_centroids):
        if dc is None:
            continue
        for ti, track in enumerate(tracks):
            if track.centroid is None:
                continue
            dist = float(np.linalg.norm(dc - track.centroid))
            if dist < max_dist:
                costs.append((dist, di, ti))
    costs.sort()

    used_dets   = set()
    used_tracks = set()
    matches     = []

    for dist, di, ti in costs:
        if di not in used_dets and ti not in used_tracks:
            matches.append((di, ti))
            used_dets.add(di)
            used_tracks.add(ti)

    unmatched_dets = [
        i for i, c in enumerate(detection_centroids)
        if c is not None and i not in used_dets
    ]
    unmatched_tracks = [i for i in range(len(tracks)) if i not in used_tracks]

    return matches, unmatched_dets, unmatched_tracks


# ── Target selection ──────────────────────────────────────────
def _torso_angle_deg(track):
    """
    Compute torso angle (°) between the shoulder-hip vector and world-up.
    Returns None if any of joints 5, 6, 11, 12 are unavailable.
    """
    pts = [kf.get_position() for kf in track.kf]
    for i in [5, 6, 11, 12]:
        if pts[i] is None:
            return None
    sh_mid  = 0.5 * (pts[5] + pts[6])
    hip_mid = 0.5 * (pts[11] + pts[12])
    v = sh_mid - hip_mid
    n = np.linalg.norm(v)
    if n < 1e-6:
        return None
    cos_a = float(np.clip(np.dot(v / n, _UP), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_a)))


def select_target(tracks, lying_angle_min,
                  current_target_id=None,
                  hysteresis_miss_count=0,
                  hysteresis_frames=10):
    """
    Select the LYING person closest to the camera.

    A person is LYING if torso angle > lying_angle_min (default 65°) and has ≥ 4 valid joints.
    If the current target temporarily loses LYING status, it is kept for up to
    hysteresis_frames consecutive frames before switching.

    Returns:
        (target_id, new_hysteresis_miss_count)
        target_id is None if no LYING person is found and hysteresis has expired.
    """
    # Collect LYING candidates: (depth_z, track_id)
    lying_candidates = []
    for track in tracks:
        angle = _torso_angle_deg(track)
        if angle is None:
            continue
        valid_joints = sum(1 for kf in track.kf if kf.get_position() is not None)
        if angle > lying_angle_min and valid_joints >= 4:
            depth = float(track.centroid[2]) if track.centroid is not None else float('inf')
            lying_candidates.append((depth, track.track_id))

    lying_candidates.sort()   # closest first

    # Is current target still classified as LYING?
    current_still_lying = any(tid == current_target_id for _, tid in lying_candidates)

    if current_still_lying:
        return current_target_id, 0   # keep target, reset miss counter

    # Current target lost LYING status — apply hysteresis
    if current_target_id is not None and hysteresis_miss_count < hysteresis_frames:
        track_ids = {t.track_id for t in tracks}
        if current_target_id in track_ids:
            return current_target_id, hysteresis_miss_count + 1

    # Hysteresis expired or no previous target — pick best LYING candidate
    if lying_candidates:
        return lying_candidates[0][1], 0

    return None, 0
```

- [ ] **Step 2: Create test file with PersonTrack test**

```python
# src/spot_perception/test/test_multi_person_tracking.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'spot_perception'))

import numpy as np
import pytest
from person_tracking import PersonTrack, assign_detections_to_tracks, select_target


def test_person_track_init():
    t = PersonTrack(track_id=7)
    assert t.track_id == 7
    assert len(t.kf) == 17
    assert t.centroid is None
    assert t.TORSO_len_ref is None
    assert t.visible == [False] * 17
    assert t.missing_count == [0] * 17
```

- [ ] **Step 3: Run test — should PASS**

```bash
cd ~/Documents/GIT_Repositories/TERESA_ws
python -m pytest src/spot_perception/test/test_multi_person_tracking.py::test_person_track_init -v
```
Expected: `PASSED`

- [ ] **Step 4: Commit**

```bash
git add src/spot_perception/spot_perception/person_tracking.py \
        src/spot_perception/test/test_multi_person_tracking.py
git commit -m "feat(spot_perception): add person_tracking module with PersonTrack"
```

---

## Task 2: Unit tests for `assign_detections_to_tracks`

**Files:**
- Modify: `src/spot_perception/test/test_multi_person_tracking.py`

- [ ] **Step 1: Add tests**

Append to the test file:
```python
def test_assign_single_match():
    t = PersonTrack(0)
    t.centroid = np.array([1.0, 0.0, 2.0])
    dets = [np.array([1.05, 0.0, 2.0])]   # 0.05 m away — within threshold
    matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(dets, [t], max_dist=0.6)
    assert matches == [(0, 0)]
    assert unmatched_dets == []
    assert unmatched_tracks == []


def test_assign_no_match_too_far():
    t = PersonTrack(0)
    t.centroid = np.array([0.0, 0.0, 1.0])
    dets = [np.array([5.0, 0.0, 1.0])]   # 5 m away — beyond threshold
    matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(dets, [t], max_dist=0.6)
    assert matches == []
    assert unmatched_dets == [0]
    assert unmatched_tracks == [0]


def test_assign_two_people_no_confusion():
    """Two people 1 m apart — each detection matches its own track."""
    t0 = PersonTrack(0);  t0.centroid = np.array([0.0, 0.0, 2.0])
    t1 = PersonTrack(1);  t1.centroid = np.array([1.0, 0.0, 2.0])
    dets = [np.array([0.05, 0.0, 2.0]),   # close to t0
            np.array([0.95, 0.0, 2.0])]   # close to t1
    matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(
        dets, [t0, t1], max_dist=0.6)
    assert set(matches) == {(0, 0), (1, 1)}
    assert unmatched_dets == []
    assert unmatched_tracks == []


def test_assign_no_tracks():
    dets = [np.array([1.0, 0.0, 2.0])]
    matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(dets, [], max_dist=0.6)
    assert matches == []
    assert unmatched_dets == [0]
    assert unmatched_tracks == []


def test_assign_none_centroid_detection_skipped():
    """Detection with None centroid (no depth) is skipped, track stays unmatched."""
    t = PersonTrack(0);  t.centroid = np.array([0.0, 0.0, 2.0])
    dets = [None]
    matches, unmatched_dets, unmatched_tracks = assign_detections_to_tracks(dets, [t], max_dist=0.6)
    assert matches == []
    assert unmatched_dets == []
    assert unmatched_tracks == [0]
```

- [ ] **Step 2: Run tests**

```bash
python -m pytest src/spot_perception/test/test_multi_person_tracking.py -k "assign" -v
```
Expected: all 5 PASSED.

- [ ] **Step 3: Commit**

```bash
git add src/spot_perception/test/test_multi_person_tracking.py
git commit -m "test(spot_perception): unit tests for assign_detections_to_tracks"
```

---

## Task 3: Unit tests for `select_target`

**Files:**
- Modify: `src/spot_perception/test/test_multi_person_tracking.py`

- [ ] **Step 1: Add tests**

Append to the test file:
```python
def _set_torso(track, sh_l, sh_r, hip_l, hip_r):
    """Helper: initialise 4 torso Kalman filters with given 3D positions."""
    for idx, pos in [(5, sh_l), (6, sh_r), (11, hip_l), (12, hip_r)]:
        track.kf[idx].update(pos)


def test_select_target_picks_lying():
    """Standing vs lying — lying person should be chosen."""
    standing = PersonTrack(0)
    # Standing: torso vertical — shoulders high (low Y in optical), hips low (high Y)
    _set_torso(standing,
               sh_l=np.array([0.0, -0.4, 2.0]), sh_r=np.array([0.2, -0.4, 2.0]),
               hip_l=np.array([0.0,  0.4, 2.0]), hip_r=np.array([0.2,  0.4, 2.0]))
    standing.centroid = np.array([0.1, 0.0, 2.0])

    lying = PersonTrack(1)
    # Lying: torso horizontal — same Y, separated along X
    _set_torso(lying,
               sh_l=np.array([ 0.4, 0.0, 1.5]), sh_r=np.array([ 0.4, 0.2, 1.5]),
               hip_l=np.array([-0.4, 0.0, 1.5]), hip_r=np.array([-0.4, 0.2, 1.5]))
    lying.centroid = np.array([0.0, 0.1, 1.5])

    result_id, _ = select_target([standing, lying], lying_angle_min=65.0)
    assert result_id == 1


def test_select_target_picks_closest_lying():
    """Two lying people — pick the closer one (smaller Z)."""
    far_lying = PersonTrack(0)
    _set_torso(far_lying,
               sh_l=np.array([ 0.4, 0.0, 3.0]), sh_r=np.array([ 0.4, 0.2, 3.0]),
               hip_l=np.array([-0.4, 0.0, 3.0]), hip_r=np.array([-0.4, 0.2, 3.0]))
    far_lying.centroid = np.array([0.0, 0.1, 3.0])

    near_lying = PersonTrack(1)
    _set_torso(near_lying,
               sh_l=np.array([ 0.4, 0.0, 1.5]), sh_r=np.array([ 0.4, 0.2, 1.5]),
               hip_l=np.array([-0.4, 0.0, 1.5]), hip_r=np.array([-0.4, 0.2, 1.5]))
    near_lying.centroid = np.array([0.0, 0.1, 1.5])

    result_id, _ = select_target([far_lying, near_lying], lying_angle_min=65.0)
    assert result_id == 1   # near_lying


def test_select_target_no_lying_returns_none():
    """No lying person → target is None."""
    standing = PersonTrack(0)
    _set_torso(standing,
               sh_l=np.array([0.0, -0.4, 2.0]), sh_r=np.array([0.2, -0.4, 2.0]),
               hip_l=np.array([0.0,  0.4, 2.0]), hip_r=np.array([0.2,  0.4, 2.0]))
    standing.centroid = np.array([0.1, 0.0, 2.0])

    result_id, _ = select_target([standing], lying_angle_min=65.0)
    assert result_id is None


def test_select_target_hysteresis_keeps_current():
    """Target temporarily loses LYING — hysteresis keeps it for up to N frames."""
    standing = PersonTrack(0)
    _set_torso(standing,
               sh_l=np.array([0.0, -0.4, 2.0]), sh_r=np.array([0.2, -0.4, 2.0]),
               hip_l=np.array([0.0,  0.4, 2.0]), hip_r=np.array([0.2,  0.4, 2.0]))
    standing.centroid = np.array([0.1, 0.0, 2.0])

    # Track 0 was the target, miss_count=3, frames=10 → keep it
    result_id, new_miss = select_target(
        [standing], lying_angle_min=65.0,
        current_target_id=0, hysteresis_miss_count=3, hysteresis_frames=10
    )
    assert result_id == 0
    assert new_miss == 4
```

- [ ] **Step 2: Run all tests**

```bash
python -m pytest src/spot_perception/test/test_multi_person_tracking.py -v
```
Expected: all 9 tests PASSED.

- [ ] **Step 3: Commit**

```bash
git add src/spot_perception/test/test_multi_person_tracking.py
git commit -m "test(spot_perception): unit tests for select_target"
```

---

## Task 4: Refactor `YoloSkeletonNodeOrbbec.__init__`

**Files:**
- Modify: `src/spot_perception/spot_perception/yolo_skeleton_spot.py`

- [ ] **Step 1: Replace imports block (top of file)**

Replace lines 1–11 with:
```python
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
```

- [ ] **Step 2: Delete the `Kalman3D` class (lines 18–65) and `TORSO_length_constraint` function (lines 70–99)**

Both now live in `person_tracking.py`. Delete them entirely from `yolo_skeleton_spot.py`.

- [ ] **Step 3: Replace `__init__` body**

Replace the entire `__init__` method of `YoloSkeletonNodeOrbbec` with:
```python
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
```

- [ ] **Step 4: Build**

```bash
cd ~/Documents/GIT_Repositories/TERESA_ws
colcon build --packages-select spot_perception 2>&1 | tail -20
```
Expected: no errors (warnings about undefined methods like `cb_color` are OK at this stage).

- [ ] **Step 5: Commit**

```bash
git add src/spot_perception/spot_perception/yolo_skeleton_spot.py
git commit -m "refactor(spot_perception): replace flat Kalman state with multi-track __init__"
```

---

## Task 5: Add helper methods

**Files:**
- Modify: `src/spot_perception/spot_perception/yolo_skeleton_spot.py`

Add the following methods to `YoloSkeletonNodeOrbbec` after `cb_depth`.

- [ ] **Step 1: Add `_adaptive_Q`** (rename from `adaptive_Q`, take `missing_count` as arg)

```python
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
```

Delete the old `adaptive_Q` method (which used `self.missing_count`).

- [ ] **Step 2: Add `_compute_raw_centroid`**

```python
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
```

- [ ] **Step 3: Add `_update_track`**

```python
def _update_track(self, track, kp, conf, fx, fy, cx, cy):
    """
    Run one Kalman update step for the given PersonTrack using YOLO keypoints `kp`.
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
```

- [ ] **Step 4: Add `_predict_track`**

```python
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
```

- [ ] **Step 5: Build**

```bash
colcon build --packages-select spot_perception 2>&1 | tail -20
```
Expected: build succeeds.

- [ ] **Step 6: Commit**

```bash
git add src/spot_perception/spot_perception/yolo_skeleton_spot.py
git commit -m "feat(spot_perception): add _update_track, _predict_track, _compute_raw_centroid, _adaptive_Q"
```

---

## Task 6: Rewrite `cb_color`

**Files:**
- Modify: `src/spot_perception/spot_perception/yolo_skeleton_spot.py`

- [ ] **Step 1: Replace the entire `cb_color` method**

```python
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
```

- [ ] **Step 2: Delete the old `predict_only` method** (replaced by `_predict_track`)

- [ ] **Step 3: Build**

```bash
colcon build --packages-select spot_perception 2>&1 | tail -20
```
Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/spot_perception/spot_perception/yolo_skeleton_spot.py
git commit -m "feat(spot_perception): rewrite cb_color for multi-person tracking"
```

---

## Task 7: Rewrite publishers

**Files:**
- Modify: `src/spot_perception/spot_perception/yolo_skeleton_spot.py`

- [ ] **Step 1: Replace `publish_empty`**

```python
def publish_empty(self, stamp):
    """Publish empty PoseArray when no target is selected."""
    empty = PoseArray()
    empty.header.stamp = stamp
    empty.header.frame_id = "camera_color_optical_frame"
    self.pub_poses.publish(empty)
```

- [ ] **Step 2: Add `_publish_target_pose`** (replaces `publish_all` for the selected target)

```python
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
```

- [ ] **Step 3: Add `_publish_all_markers`**

```python
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
```

- [ ] **Step 4: Delete the old `publish_all` method**

- [ ] **Step 5: Build and run all unit tests**

```bash
colcon build --packages-select spot_perception 2>&1 | tail -20
python -m pytest src/spot_perception/test/test_multi_person_tracking.py -v
```
Expected: build succeeds, all 9 tests PASSED.

- [ ] **Step 6: Commit**

```bash
git add src/spot_perception/spot_perception/yolo_skeleton_spot.py
git commit -m "feat(spot_perception): multi-color skeleton markers with TARGET label"
```

---

## Task 8: Update launch parameters

**Files:**
- Modify: `src/spot_perception/launch/spot_perception.launch.py`

- [ ] **Step 1: Update `yolo_skeleton_node` parameters block**

Find the `yolo_skeleton_node = Node(...)` definition and update its `parameters` list:
```python
yolo_skeleton_node = Node(
    package='spot_perception',
    executable='yolo_skeleton_node_orbbec',
    name='yolo_skeleton_node',
    output='screen',
    parameters=[{
        'model_path':                'yolo11n-pose.pt',
        'conf_thr':                   0.25,
        'vel_damping':                0.5,
        'max_depth_m':                5.0,
        'max_track_distance':         0.6,
        'track_timeout':              1.5,
        'lying_torso_angle_min':     65.0,
        'max_tracks':                 5,
        'target_hysteresis_frames':  10,
    }]
)
```

- [ ] **Step 2: Build**

```bash
colcon build --packages-select spot_perception 2>&1 | tail -10
```
Expected: build succeeds.

- [ ] **Step 3: Commit**

```bash
git add src/spot_perception/launch/spot_perception.launch.py
git commit -m "feat(spot_perception): expose multi-track params in launch file"
```

---

## Task 9: Integration test

**Files:** None (manual)

- [ ] **Step 1: Launch**

Terminal 1:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch spot_perception spot_perception.launch.py
```

- [ ] **Step 2: Open RViz**

```bash
rviz2
# Fixed Frame: camera_color_optical_frame
# Add: MarkerArray → /human_pose/skeleton_markers
```

Expected:
- All people in frame shown as skeletons
- Standing/sitting people: grey
- LYING person (if present): green skeleton + "TARGET" label
- No keypoint mixing between people when multiple are in frame

- [ ] **Step 3: Verify downstream still works**

```bash
ros2 topic hz /human_pose/points_3d
ros2 topic echo /human_pose/posture
```
Expected: data at ~15 Hz; posture reflects the target person.

- [ ] **Step 4: Test lying detection**

Lie down in front of the camera. Expected:
- That person turns green with "TARGET" label
- `/human_pose/posture` → `"LYING"`
- Standing bystanders remain grey
