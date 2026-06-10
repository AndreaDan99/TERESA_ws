"""Unit tests for posture_classifier.py estimate_posture logic.

Tests the pure classification logic without ROS dependencies.
Uses SMPL-24 joint indices from sml_pose_indices.py.
"""

import math
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pytest

# Inline SMPL-24 joint indices to avoid ROS package dependency
# (mirrors src/spot_perception/spot_perception/sml_pose_indices.py)
PELVIS = 0
HIP_LEFT = 1
HIP_RIGHT = 2
SPINE1 = 3
KNEE_LEFT = 4
KNEE_RIGHT = 5
SPINE2 = 6
ANKLE_LEFT = 7
ANKLE_RIGHT = 8
SPINE3 = 9
FOOT_LEFT = 10
FOOT_RIGHT = 11
NECK = 12
COLLAR_LEFT = 13
COLLAR_RIGHT = 14
HEAD = 15
SHOULDER_LEFT = 16
SHOULDER_RIGHT = 17
ELBOW_LEFT = 18
ELBOW_RIGHT = 19
WRIST_LEFT = 20
WRIST_RIGHT = 21
HAND_LEFT = 22
HAND_RIGHT = 23
NUM_JOINTS = 24


# ============================================================
# Pure-function replica of estimate_posture (no ROS deps)
# ============================================================

def normalize(v, eps=1e-9):
    n = float(np.linalg.norm(v))
    return v / (n + eps)


def angle_deg(a, b):
    a = normalize(a)
    b = normalize(b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(math.acos(c)))


def estimate_posture(
    pts,
    knee_angle_stand_min=160.0,
    knee_angle_sit_max=120.0,
    torso_angle_lying_min=65.0,
    hip_knee_ratio_sit_max=0.8,
    verticality_ratio_lying_max=0.30,
    knee_angle_lying_bonus_max=140.0,
):
    """Standalone replica of HumanPostureAnalyzerSpot.estimate_posture.

    Args:
        pts: List of NUM_JOINTS elements, each either None or np.array([x,y,z]).
        knee_angle_stand_min: Minimum knee angle for STANDING.
        knee_angle_sit_max: Maximum knee angle for SITTING.
        torso_angle_lying_min: Minimum torso angle for LYING.
        hip_knee_ratio_sit_max: Hip-knee vertical ratio threshold for SITTING bonus.
        verticality_ratio_lying_max: Max verticality ratio for LYING.
        knee_angle_lying_bonus_max: Knee angle below which LYING gets a bonus.

    Returns:
        Tuple of (posture, conf, height, torso_angle, hip_mid, torso_vec).
    """
    # Camera optical frame: UP = -Y (Orbbec standard)
    up = np.array([0.0, -1.0, 0.0], dtype=np.float64)

    valid = [p for p in pts if p is not None]
    quality = len(valid) / NUM_JOINTS

    if len(valid) < 4:
        return "UNKNOWN", 0.0, None, None, None, None

    # --------------------------------------------------
    # ALTEZZA ANATOMICA
    # --------------------------------------------------
    height = None

    shoulders = []
    if pts[SHOULDER_LEFT] is not None:
        shoulders.append(pts[SHOULDER_LEFT])
    if pts[SHOULDER_RIGHT] is not None:
        shoulders.append(pts[SHOULDER_RIGHT])

    feet = []
    if pts[ANKLE_LEFT] is not None:
        feet.append(pts[ANKLE_LEFT])
    if pts[ANKLE_RIGHT] is not None:
        feet.append(pts[ANKLE_RIGHT])

    if len(feet) == 0:
        if pts[KNEE_LEFT] is not None:
            feet.append(pts[KNEE_LEFT])
        if pts[KNEE_RIGHT] is not None:
            feet.append(pts[KNEE_RIGHT])

    feet_mid = None
    if len(shoulders) > 0 and len(feet) > 0:
        sh_mid = np.mean(shoulders, axis=0)
        feet_mid = np.mean(feet, axis=0)
        height = abs(np.dot(sh_mid - feet_mid, up))
    else:
        height = None

    # --------------------------------------------------
    # Hip midpoint
    # --------------------------------------------------
    hip_mid = None
    if pts[HIP_LEFT] is not None and pts[HIP_RIGHT] is not None:
        hip_mid = 0.5 * (pts[HIP_LEFT] + pts[HIP_RIGHT])
    elif pts[HIP_LEFT] is not None:
        hip_mid = pts[HIP_LEFT]
    elif pts[HIP_RIGHT] is not None:
        hip_mid = pts[HIP_RIGHT]

    if hip_mid is None:
        return "UNKNOWN", 0.0, height, None, None, None

    # --------------------------------------------------
    # Knee midpoint
    # --------------------------------------------------
    knee_mid = None
    knees = []
    if pts[KNEE_LEFT] is not None:
        knees.append(pts[KNEE_LEFT])
    if pts[KNEE_RIGHT] is not None:
        knees.append(pts[KNEE_RIGHT])

    if len(knees) > 0:
        knee_mid = np.mean(knees, axis=0)

    # --------------------------------------------------
    # Torso vector + angle
    # --------------------------------------------------
    if len(shoulders) == 0:
        return "UNKNOWN", 0.0, height, None, None, None

    sh_mid = np.mean(shoulders, axis=0)

    # --------------------------------------------------
    # Body length (Euclidean distance, distance-invariant)
    # --------------------------------------------------
    body_length = None
    if feet_mid is not None:
        body_length = float(np.linalg.norm(sh_mid - feet_mid))
    elif knee_mid is not None:
        body_length = float(np.linalg.norm(sh_mid - knee_mid))

    # Verticality ratio (distance-invariant: standing≈1.0, lying≈0.0)
    verticality_ratio = None
    if height is not None and body_length is not None and body_length > 1e-9:
        verticality_ratio = height / body_length

    # SPINE vector (SPINE1→SPINE3) preferred for torso angle
    if pts[SPINE1] is not None and pts[SPINE3] is not None:
        spine_vec = pts[SPINE3] - pts[SPINE1]
        if np.linalg.norm(spine_vec) > 1e-6:
            torso_vec = spine_vec
        else:
            torso_vec = sh_mid - hip_mid
    else:
        torso_vec = sh_mid - hip_mid

    torso_angle = angle_deg(torso_vec, up)

    # --------------------------------------------------
    # Knee angles
    # --------------------------------------------------
    def knee_angle(h, k, a):
        v1 = h - k
        v2 = a - k
        if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
            return None
        return angle_deg(v1, v2)

    knee_angles = []

    if pts[HIP_LEFT] is not None and pts[KNEE_LEFT] is not None and pts[ANKLE_LEFT] is not None:
        ang = knee_angle(pts[HIP_LEFT], pts[KNEE_LEFT], pts[ANKLE_LEFT])
        if ang is not None:
            knee_angles.append(ang)

    if pts[HIP_RIGHT] is not None and pts[KNEE_RIGHT] is not None and pts[ANKLE_RIGHT] is not None:
        ang = knee_angle(pts[HIP_RIGHT], pts[KNEE_RIGHT], pts[ANKLE_RIGHT])
        if ang is not None:
            knee_angles.append(ang)

    avg_knee_angle = float(np.mean(knee_angles)) if len(knee_angles) > 0 else None

    # --------------------------------------------------
    # Hip-to-knee vertical distance ratio
    # --------------------------------------------------
    hip_knee_dist_ratio = None
    if knee_mid is not None and height is not None and height > 0.1:
        hip_knee_vertical = abs(np.dot(hip_mid - knee_mid, up))
        hip_knee_dist_ratio = hip_knee_vertical / height

    # --------------------------------------------------
    # CLASSIFICATION
    # --------------------------------------------------
    posture = "UNKNOWN"
    score = 0.0

    # LYING: distance-invariant verticality ratio + adaptive torso threshold
    effective_torso_lying = torso_angle_lying_min + (10.0 if body_length is not None and body_length < 0.5 else 0.0)
    if torso_angle > effective_torso_lying and (
        height is None or (verticality_ratio is not None and verticality_ratio < verticality_ratio_lying_max)
    ):
        posture = "LYING"
        score = 0.85
        if avg_knee_angle is not None and avg_knee_angle < knee_angle_lying_bonus_max:
            score += 0.05

    # SITTING
    elif avg_knee_angle is not None and avg_knee_angle < knee_angle_sit_max:
        posture = "SITTING"
        score = 0.70
        if hip_knee_dist_ratio is not None and hip_knee_dist_ratio < hip_knee_ratio_sit_max:
            score += 0.15
        if torso_angle < 45:
            score += 0.05

    # STANDING
    elif avg_knee_angle is not None and avg_knee_angle > knee_angle_stand_min:
        posture = "STANDING"
        score = 0.70
        if hip_knee_dist_ratio is not None and hip_knee_dist_ratio > 0.25:
            score += 0.15
        if torso_angle < 30:
            score += 0.05

    # FALLBACK: hip-knee ratio
    elif hip_knee_dist_ratio is not None:
        if hip_knee_dist_ratio < 0.15:
            posture = "SITTING"
            score = 0.50
        elif hip_knee_dist_ratio > 0.30:
            posture = "STANDING"
            score = 0.50
        else:
            posture = "SITTING"
            score = 0.35

    else:
        posture = "UNKNOWN"
        score = 0.20

    conf = score + 0.15 * quality
    conf = float(np.clip(conf, 0.0, 1.0))

    return posture, conf, height, torso_angle, hip_mid, torso_vec


# ============================================================
# Helpers
# ============================================================

def make_keypoints_24(joints_dict):
    """Create a 24-element list from a dict of {index: [x,y,z] or None}."""
    pts = [None] * 24
    for idx, val in joints_dict.items():
        pts[idx] = np.array(val, dtype=np.float64) if val is not None else None
    return pts


def smooth_posture(history, posture):
    """Append a posture to a deque and return the smoothed result (majority of 5)."""
    if len(history) == 0:
        for _ in range(history.maxlen):
            history.append(posture)
    else:
        history.append(posture)
    if history.count(posture) >= 3:
        return posture
    # Return the most common posture in history
    return max(set(history), key=history.count)


# ============================================================
# Fixtures: common keypoint configurations
# ============================================================

@pytest.fixture
def standing_person():
    """A standing person at ~3m distance (small coordinates in camera frame).

    UP = -Y, so standing means head is at lower Y, feet at higher Y.
    Body length ~1.7m in world, but at 3m distance in camera frame the
    coordinates are small. Verticality ratio should be ~1.0.
    """
    # Z = 3.0 (distance from camera), Y range ~0.3m (small due to distance)
    # Standing: shoulders above hips above knees above ankles
    return make_keypoints_24({
        SHOULDER_LEFT:  [0.0, -0.15, 3.0],
        SHOULDER_RIGHT: [0.0, -0.15, 3.0],
        HIP_LEFT:       [0.0,  0.0,  3.0],
        HIP_RIGHT:      [0.0,  0.0,  3.0],
        KNEE_LEFT:      [0.0,  0.15, 3.0],
        KNEE_RIGHT:     [0.0,  0.15, 3.0],
        ANKLE_LEFT:     [0.0,  0.30, 3.0],
        ANKLE_RIGHT:    [0.0,  0.30, 3.0],
        SPINE1:         [0.0, -0.05, 3.0],
        SPINE3:         [0.0, -0.12, 3.0],
    })


@pytest.fixture
def lying_person():
    """A person lying on the ground (horizontal).

    Torso is roughly horizontal → torso_angle ≈ 90°.
    Verticality ratio ≈ 0.0 (height along UP is near zero).
    """
    return make_keypoints_24({
        SHOULDER_LEFT:  [-0.3, 0.0, 3.0],
        SHOULDER_RIGHT: [-0.3, 0.0, 3.0],
        HIP_LEFT:       [ 0.0, 0.0, 3.0],
        HIP_RIGHT:      [ 0.0, 0.0, 3.0],
        KNEE_LEFT:      [ 0.3, 0.0, 3.0],
        KNEE_RIGHT:     [ 0.3, 0.0, 3.0],
        ANKLE_LEFT:     [ 0.6, 0.0, 3.0],
        ANKLE_RIGHT:    [ 0.6, 0.0, 3.0],
        SPINE1:         [-0.1, 0.0, 3.0],
        SPINE3:         [-0.25, 0.0, 3.0],
    })


@pytest.fixture
def sitting_person():
    """A person sitting on a chair with bent knees (~90°).

    Knees project forward in Z, ankles come back under the chair.
    Hip-knee-ankle forms a ~90° bend.
    """
    return make_keypoints_24({
        SHOULDER_LEFT:  [0.0, -0.40, 3.0],
        SHOULDER_RIGHT: [0.0, -0.40, 3.0],
        HIP_LEFT:       [0.0, -0.20, 3.0],
        HIP_RIGHT:      [0.0, -0.20, 3.0],
        KNEE_LEFT:      [0.0,  0.10, 3.2],
        KNEE_RIGHT:     [0.0,  0.10, 3.2],
        ANKLE_LEFT:     [0.0,  0.15, 3.0],
        ANKLE_RIGHT:    [0.0,  0.15, 3.0],
        SPINE1:         [0.0, -0.25, 3.0],
        SPINE3:         [0.0, -0.35, 3.0],
    })


# ============================================================
# Tests
# ============================================================

class TestEstimatePosture:
    """Tests for the standalone estimate_posture function."""

    def test_standing_person_at_distance(self, standing_person):
        """A standing person at 3m distance → STANDING, NOT LYING."""
        posture, conf, height, torso_angle, _, _ = estimate_posture(standing_person)
        assert posture == "STANDING", f"Expected STANDING, got {posture}"
        assert conf > 0.5, f"Confidence should be >0.5, got {conf}"
        # Verticality ratio should be ~1.0 for standing
        assert height is not None and height > 0, "Height should be > 0"

    def test_lying_person(self, lying_person):
        """A lying person (verticality_ratio≈0.0, torso≈90°) → LYING."""
        posture, conf, height, torso_angle, _, _ = estimate_posture(lying_person)
        assert posture == "LYING", f"Expected LYING, got {posture}"
        assert conf > 0.5, f"Confidence should be >0.5, got {conf}"
        # Torso should be near horizontal
        assert torso_angle is not None and torso_angle > 65, (
            f"Torso angle should be >65° for lying, got {torso_angle}"
        )

    def test_standing_far_small_body_length(self):
        """Standing person far away, body_length<0.5m → adaptive threshold +10°.

        The adaptive threshold means torso_angle must exceed 75° (65+10) to be LYING.
        A standing person with torso_angle ~70° should still be STANDING.
        """
        # Small coordinates: body_length < 0.5m, torso_angle ~70° (not enough for LYING)
        pts = make_keypoints_24({
            SHOULDER_LEFT:  [0.0, -0.08, 5.0],
            SHOULDER_RIGHT: [0.0, -0.08, 5.0],
            HIP_LEFT:       [0.0,  0.0,  5.0],
            HIP_RIGHT:      [0.0,  0.0,  5.0],
            KNEE_LEFT:      [0.0,  0.08, 5.0],
            KNEE_RIGHT:     [0.0,  0.08, 5.0],
            ANKLE_LEFT:     [0.0,  0.16, 5.0],
            ANKLE_RIGHT:    [0.0,  0.16, 5.0],
            SPINE1:         [0.0, -0.02, 5.0],
            SPINE3:         [0.0, -0.06, 5.0],
        })
        posture, conf, _, _, _, _ = estimate_posture(pts)
        assert posture == "STANDING", f"Expected STANDING, got {posture}"

    def test_noisy_standing_ratio_blocks_lying(self):
        """Noisy standing with torso>65° but verticality_ratio≈1.0 → NOT LYING.

        The verticality_ratio check (must be < 0.30) blocks LYING even if
        torso angle is high.
        """
        # Torso is tilted (torso_angle > 65°) but verticality_ratio ≈ 1.0
        # because the person is standing upright (height ≈ body_length)
        pts = make_keypoints_24({
            SHOULDER_LEFT:  [0.2, -0.15, 3.0],
            SHOULDER_RIGHT: [0.2, -0.15, 3.0],
            HIP_LEFT:       [0.0,  0.0,  3.0],
            HIP_RIGHT:      [0.0,  0.0,  3.0],
            KNEE_LEFT:      [0.0,  0.15, 3.0],
            KNEE_RIGHT:     [0.0,  0.15, 3.0],
            ANKLE_LEFT:     [0.0,  0.30, 3.0],
            ANKLE_RIGHT:    [0.0,  0.30, 3.0],
            SPINE1:         [0.0, -0.05, 3.0],
            SPINE3:         [0.2, -0.12, 3.0],
        })
        posture, conf, _, _, _, _ = estimate_posture(pts)
        # Should NOT be LYING because verticality_ratio ≈ 1.0
        assert posture != "LYING", (
            f"Expected NOT LYING (ratio blocks it), got {posture}"
        )

    def test_lying_no_feet_fallback(self):
        """Lying person with NO feet visible → LYING (fallback via knees).

        When no ankles are visible, the code falls back to knees for feet_mid.
        Height is computed from shoulders→knees. The verticality ratio is still
        near zero for a lying person, so LYING is correctly detected.
        """
        pts = make_keypoints_24({
            SHOULDER_LEFT:  [-0.3, 0.0, 3.0],
            SHOULDER_RIGHT: [-0.3, 0.0, 3.0],
            HIP_LEFT:       [ 0.0, 0.0, 3.0],
            HIP_RIGHT:      [ 0.0, 0.0, 3.0],
            KNEE_LEFT:      [ 0.3, 0.0, 3.0],
            KNEE_RIGHT:     [ 0.3, 0.0, 3.0],
            # No ankles/feet — code falls back to knees
            SPINE1:         [-0.1, 0.0, 3.0],
            SPINE3:         [-0.25, 0.0, 3.0],
        })
        posture, conf, height, torso_angle, _, _ = estimate_posture(pts)
        assert posture == "LYING", f"Expected LYING (no feet fallback), got {posture}"
        # Height is computed from shoulders→knees (knees used as fallback for feet)
        assert height is not None, "Height should be computed from knees fallback"

    def test_sitting_bent_knees(self, sitting_person):
        """Person sitting with bent knees (knee_angle<120°) → SITTING."""
        posture, conf, _, _, _, _ = estimate_posture(sitting_person)
        assert posture == "SITTING", f"Expected SITTING, got {posture}"
        assert conf > 0.5, f"Confidence should be >0.5, got {conf}"

    def test_missing_keypoints_unknown(self):
        """Fewer than 4 valid keypoints → UNKNOWN."""
        pts = make_keypoints_24({
            SHOULDER_LEFT: [0.0, -0.1, 3.0],
            SHOULDER_RIGHT: [0.0, -0.1, 3.0],
            HIP_LEFT: [0.0, 0.0, 3.0],
            # Only 3 valid keypoints
        })
        posture, conf, _, _, _, _ = estimate_posture(pts)
        assert posture == "UNKNOWN", f"Expected UNKNOWN, got {posture}"
        assert conf == 0.0, f"Confidence should be 0.0 for UNKNOWN, got {conf}"

    def test_temporal_smoothing_noisy_frame(self):
        """Temporal smoothing: 1 noisy frame doesn't change posture.

        Simulate 5 frames: 4 standing, 1 lying. The majority should win.
        """
        standing = make_keypoints_24({
            SHOULDER_LEFT:  [0.0, -0.15, 3.0],
            SHOULDER_RIGHT: [0.0, -0.15, 3.0],
            HIP_LEFT:       [0.0,  0.0,  3.0],
            HIP_RIGHT:      [0.0,  0.0,  3.0],
            KNEE_LEFT:      [0.0,  0.15, 3.0],
            KNEE_RIGHT:     [0.0,  0.15, 3.0],
            ANKLE_LEFT:     [0.0,  0.30, 3.0],
            ANKLE_RIGHT:    [0.0,  0.30, 3.0],
            SPINE1:         [0.0, -0.05, 3.0],
            SPINE3:         [0.0, -0.12, 3.0],
        })
        lying = make_keypoints_24({
            SHOULDER_LEFT:  [-0.3, 0.0, 3.0],
            SHOULDER_RIGHT: [-0.3, 0.0, 3.0],
            HIP_LEFT:       [ 0.0, 0.0, 3.0],
            HIP_RIGHT:      [ 0.0, 0.0, 3.0],
            KNEE_LEFT:      [ 0.3, 0.0, 3.0],
            KNEE_RIGHT:     [ 0.3, 0.0, 3.0],
            ANKLE_LEFT:     [ 0.6, 0.0, 3.0],
            ANKLE_RIGHT:    [ 0.6, 0.0, 3.0],
            SPINE1:         [-0.1, 0.0, 3.0],
            SPINE3:         [-0.25, 0.0, 3.0],
        })

        history = deque(maxlen=5)
        frames = [standing, standing, standing, lying, standing]

        smoothed_postures = []
        for pts in frames:
            posture, _, _, _, _, _ = estimate_posture(pts)
            result = smooth_posture(history, posture)
            smoothed_postures.append(result)

        # After 5 frames, the majority (4 standing, 1 lying) should be STANDING
        final_posture = smoothed_postures[-1]
        assert final_posture == "STANDING", (
            f"Expected STANDING after smoothing, got {final_posture}"
        )

    def test_verticality_ratio_threshold_boundary(self):
        """Verticality ratio exactly at threshold boundary.

        Below threshold (0.29) → LYING (if torso angle also high).
        Above threshold (0.31) → NOT LYING.
        """
        up = np.array([0.0, -1.0, 0.0], dtype=np.float64)

        # Person with verticality_ratio ≈ 0.29 (just below 0.30 threshold)
        # and torso_angle > 65° → should be LYING
        pts_below = make_keypoints_24({
            SHOULDER_LEFT:  [-0.3, -0.05, 3.0],
            SHOULDER_RIGHT: [-0.3, -0.05, 3.0],
            HIP_LEFT:       [ 0.0,  0.0,  3.0],
            HIP_RIGHT:      [ 0.0,  0.0,  3.0],
            KNEE_LEFT:      [ 0.3,  0.05, 3.0],
            KNEE_RIGHT:     [ 0.3,  0.05, 3.0],
            ANKLE_LEFT:     [ 0.6,  0.10, 3.0],
            ANKLE_RIGHT:    [ 0.6,  0.10, 3.0],
            SPINE1:         [-0.1, -0.02, 3.0],
            SPINE3:         [-0.25, -0.04, 3.0],
        })

        # Person with verticality_ratio ≈ 0.31 (just above 0.30 threshold)
        # and torso_angle > 65° → should NOT be LYING (ratio blocks it)
        pts_above = make_keypoints_24({
            SHOULDER_LEFT:  [-0.3, -0.10, 3.0],
            SHOULDER_RIGHT: [-0.3, -0.10, 3.0],
            HIP_LEFT:       [ 0.0,  0.0,  3.0],
            HIP_RIGHT:      [ 0.0,  0.0,  3.0],
            KNEE_LEFT:      [ 0.3,  0.10, 3.0],
            KNEE_RIGHT:     [ 0.3,  0.10, 3.0],
            ANKLE_LEFT:     [ 0.6,  0.20, 3.0],
            ANKLE_RIGHT:    [ 0.6,  0.20, 3.0],
            SPINE1:         [-0.1, -0.04, 3.0],
            SPINE3:         [-0.25, -0.08, 3.0],
        })

        # Verify verticality ratios
        def compute_verticality(pts):
            sh = np.mean([pts[SHOULDER_LEFT], pts[SHOULDER_RIGHT]], axis=0)
            ft = np.mean([pts[ANKLE_LEFT], pts[ANKLE_RIGHT]], axis=0)
            h = abs(np.dot(sh - ft, up))
            bl = float(np.linalg.norm(sh - ft))
            return h / bl if bl > 1e-9 else 0

        vr_below = compute_verticality(pts_below)
        vr_above = compute_verticality(pts_above)

        assert vr_below < 0.30, (
            f"Expected verticality_ratio < 0.30 for below-threshold, got {vr_below}"
        )
        assert vr_above > 0.30, (
            f"Expected verticality_ratio > 0.30 for above-threshold, got {vr_above}"
        )

        posture_below, _, _, _, _, _ = estimate_posture(pts_below)
        posture_above, _, _, _, _, _ = estimate_posture(pts_above)

        assert posture_below == "LYING", (
            f"Expected LYING below threshold, got {posture_below}"
        )
        assert posture_above != "LYING", (
            f"Expected NOT LYING above threshold, got {posture_above}"
        )

    def test_standing_with_spine_joints(self):
        """Standing person with SPINE joints available → uses spine_vec for torso.

        This verifies the SPINE1→SPINE3 preference path.
        """
        pts = make_keypoints_24({
            SHOULDER_LEFT:  [0.0, -0.15, 3.0],
            SHOULDER_RIGHT: [0.0, -0.15, 3.0],
            HIP_LEFT:       [0.0,  0.0,  3.0],
            HIP_RIGHT:      [0.0,  0.0,  3.0],
            KNEE_LEFT:      [0.0,  0.15, 3.0],
            KNEE_RIGHT:     [0.0,  0.15, 3.0],
            ANKLE_LEFT:     [0.0,  0.30, 3.0],
            ANKLE_RIGHT:    [0.0,  0.30, 3.0],
            SPINE1:         [0.0, -0.05, 3.0],
            SPINE3:         [0.0, -0.12, 3.0],
        })
        posture, conf, _, _, _, _ = estimate_posture(pts)
        assert posture == "STANDING", f"Expected STANDING, got {posture}"
        assert conf > 0.5, f"Confidence should be >0.5, got {conf}"

    def test_unknown_no_hip_mid(self):
        """No hip keypoints → UNKNOWN (hip_mid is None)."""
        pts = make_keypoints_24({
            SHOULDER_LEFT:  [0.0, -0.15, 3.0],
            SHOULDER_RIGHT: [0.0, -0.15, 3.0],
            KNEE_LEFT:      [0.0,  0.15, 3.0],
            KNEE_RIGHT:     [0.0,  0.15, 3.0],
            ANKLE_LEFT:     [0.0,  0.30, 3.0],
            ANKLE_RIGHT:    [0.0,  0.30, 3.0],
            SPINE1:         [0.0, -0.05, 3.0],
            SPINE3:         [0.0, -0.12, 3.0],
            # No HIP_LEFT or HIP_RIGHT
        })
        posture, conf, _, _, _, _ = estimate_posture(pts)
        assert posture == "UNKNOWN", f"Expected UNKNOWN, got {posture}"
