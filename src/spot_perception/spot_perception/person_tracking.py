# src/spot_perception/spot_perception/person_tracking.py
import time
import numpy as np

from spot_perception.sml_pose_indices import (
    PELVIS, HIP_LEFT, HIP_RIGHT, SPINE1, KNEE_LEFT, KNEE_RIGHT,
    SPINE2, ANKLE_LEFT, ANKLE_RIGHT, SPINE3, FOOT_LEFT, FOOT_RIGHT,
    NECK, COLLAR_LEFT, COLLAR_RIGHT, HEAD,
    SHOULDER_LEFT, SHOULDER_RIGHT, ELBOW_LEFT, ELBOW_RIGHT,
    WRIST_LEFT, WRIST_RIGHT, HAND_LEFT, HAND_RIGHT,
    NUM_JOINTS,
    TORSO_JOINTS, ARM_JOINTS, LEG_JOINTS,
    SPINE_JOINTS, HEAD_JOINTS, FEET_JOINTS,
)


# ── Joint group sets (SMPL-24) ────────────────────────────────
# TORSO_JOINTS, ARM_JOINTS, LEG_JOINTS imported from sml_pose_indices
NOSE_JOINTS  = {0}  # PELVIS used as nose proxy
SKIP_JOINTS  = set()  # all 24 joints are tracked now

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
    """One tracked person: NUM_JOINTS independent Kalman filters + tracking metadata."""

    def __init__(self, track_id: int):
        self.track_id = track_id
        self.kf = [Kalman3D() for _ in range(NUM_JOINTS)]
        self.visible = [False] * NUM_JOINTS
        self.missing_count = [0] * NUM_JOINTS
        self.TORSO_len_ref = None
        self.centroid = None        # np.array([x,y,z]) — mean of torso joints, updated each frame
        self.last_seen: float = time.monotonic()
        self._cached_pts = [None] * NUM_JOINTS  # last corrected positions (after constraints)

        # Per-joint Q/R tuning
        for i, kf in enumerate(self.kf):
            if i in TORSO_JOINTS:
                kf.Q *= 0.7
                kf.R *= 0.7
            elif i in LEG_JOINTS:
                kf.Q *= 0.9
                kf.R *= 0.9
            elif i in NOSE_JOINTS:
                kf.Q *= 1.5
                kf.R *= 1.5
            elif i in SPINE_JOINTS:
                kf.Q *= 0.25   # process_noise=0.00005 relative to base 0.0002
                kf.R *= 0.125  # meas_noise=0.25 relative to base 2.0
            elif i in HEAD_JOINTS:
                kf.Q *= 0.75   # process_noise=0.00015
                kf.R *= 0.375  # meas_noise=0.75
            elif i in FEET_JOINTS:
                kf.Q *= 0.35   # process_noise=0.00007
                kf.R *= 0.175  # meas_noise=0.35
            # HAND_LEFT, HAND_RIGHT: factor 0.5 → process_noise=0.0001, meas_noise=0.5
            elif i in {HAND_LEFT, HAND_RIGHT}:
                kf.Q *= 0.5
                kf.R *= 0.25
            # ARM_JOINTS (shoulders/elbows/wrists): factor 1.0 — no change


# ── TORSO length constraint ───────────────────────────────────
def TORSO_length_constraint(pts, visible, L_ref, stiffness=0.35):
    """Soft constraint: keep shoulder-hip distance close to reference length L_ref."""
    # NOTE: mutates pts[SHOULDER_LEFT] and pts[SHOULDER_RIGHT] in-place. Callers must pass mutable arrays.
    if L_ref is None:
        return pts
    idx = [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT]
    if any(pts[i] is None for i in idx):
        return pts
    if all(visible[i] for i in idx):
        return pts   # all visible — no correction needed
    sh_mid  = 0.5 * (pts[SHOULDER_LEFT] + pts[SHOULDER_RIGHT])
    hip_mid = 0.5 * (pts[HIP_LEFT] + pts[HIP_RIGHT])
    v = sh_mid - hip_mid
    dist = np.linalg.norm(v)
    if dist < 1e-6:
        return pts
    v_corr = (v / dist) * L_ref
    target_sh_mid = hip_mid + v_corr
    delta = target_sh_mid - sh_mid
    pts[SHOULDER_LEFT] += stiffness * delta
    pts[SHOULDER_RIGHT] += stiffness * delta
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
def torso_angle_deg(track):
    """
    Compute torso angle (°) between the SPINE vector (SPINE3→SPINE1) and world-up.
    Returns None if SPINE1 or SPINE3 are unavailable.
    """
    pts = [kf.get_position() for kf in track.kf]
    if pts[SPINE1] is None or pts[SPINE3] is None:
        return None
    v = pts[SPINE3] - pts[SPINE1]  # SPINE3→SPINE1 (upward in body frame)
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

    A person is LYING if torso angle > lying_angle_min (default 65°) and has >= 4 valid joints.
    If the current target temporarily loses LYING status, it is kept for up to
    hysteresis_frames consecutive frames before switching.

    Returns:
        (target_id, new_hysteresis_miss_count)
        target_id is None if no LYING person is found and hysteresis has expired.
    """
    # Collect LYING candidates: (depth_z, track_id)
    lying_candidates = []
    for track in tracks:
        angle = torso_angle_deg(track)
        if angle is None:
            continue
        valid_joints = sum(1 for kf in track.kf if kf.get_position() is not None)
        if angle > lying_angle_min and valid_joints >= 4:
            depth = float(track.centroid[2]) if track.centroid is not None else float('inf')  # Z = depth in camera frame
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
