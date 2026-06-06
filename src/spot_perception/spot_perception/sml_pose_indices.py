"""SMPL-24 joint index constants. Single source of truth for all TERESA perception modules."""

import math

# ── SMPL-24 joint indices ──────────────────────────────────────────────────
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

# ── Anatomical groups ───────────────────────────────────────────────────────
TORSO_JOINTS = [
    SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT,
    SPINE1, SPINE2, SPINE3, PELVIS, NECK,
]
ARM_JOINTS = [
    SHOULDER_LEFT, ELBOW_LEFT, WRIST_LEFT,
    SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT,
]
LEG_JOINTS = [
    HIP_LEFT, KNEE_LEFT, ANKLE_LEFT,
    HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT,
]
SPINE_JOINTS = [SPINE1, SPINE2, SPINE3]
HEAD_JOINTS = [NECK, HEAD]
FEET_JOINTS = [FOOT_LEFT, FOOT_RIGHT]

# ── Joints that are always NaN in YOLO-only mode ────────────────────────────
NEVER_AVAILABLE_YOLO = [
    SPINE1, SPINE2, SPINE3, FOOT_LEFT, FOOT_RIGHT,
    NECK, HEAD, HAND_LEFT, HAND_RIGHT,
]


# ── Validation ──────────────────────────────────────────────────────────────
def is_valid(pose):
    """Return True if the Pose has a valid (non-NaN) position."""
    return not math.isnan(pose.position.x)
