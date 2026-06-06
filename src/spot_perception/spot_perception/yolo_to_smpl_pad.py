"""Adapter to convert 17 COCO keypoint poses to 24 SMPL poses for YOLO fallback mode."""

# SMPL-24 constants (synchronized with sml_pose_indices.py)
NECK = 12
SHOULDER_LEFT = 16
SHOULDER_RIGHT = 17
ELBOW_LEFT = 18
ELBOW_RIGHT = 19
WRIST_LEFT = 20
WRIST_RIGHT = 21
HIP_LEFT = 1
HIP_RIGHT = 2
KNEE_LEFT = 4
KNEE_RIGHT = 5
ANKLE_LEFT = 7
ANKLE_RIGHT = 8
NUM_JOINTS = 24

COCO_TO_SMPL = {
    0:  NECK,           # nose → neck (approximation)
    1:  None,           # eye_left → no SMPL equivalent
    2:  None,           # eye_right → no SMPL equivalent
    3:  None,           # ear_left → no SMPL equivalent
    4:  None,           # ear_right → no SMPL equivalent
    5:  SHOULDER_LEFT,  # shoulder_left
    6:  SHOULDER_RIGHT, # shoulder_right
    7:  ELBOW_LEFT,     # elbow_left
    8:  ELBOW_RIGHT,    # elbow_right
    9:  WRIST_LEFT,     # wrist_left
    10: WRIST_RIGHT,    # wrist_right
    11: HIP_LEFT,       # hip_left
    12: HIP_RIGHT,      # hip_right
    13: KNEE_LEFT,      # knee_left
    14: KNEE_RIGHT,     # knee_right
    15: ANKLE_LEFT,     # ankle_left
    16: ANKLE_RIGHT,    # ankle_right
}


def _make_nan_pose():
    """Create a Pose-like object with NaN position."""
    return type(
        "Pose",
        (),
        {
            "position": type(
                "Point",
                (),
                {
                    "x": float("nan"),
                    "y": float("nan"),
                    "z": float("nan"),
                },
            )()
        },
    )()


def coco_to_smpl_24(coco_poses, PoseClass=None):
    """Convert 17 COCO keypoint poses to 24 SMPL poses.
    Uses proper ROS2 Pose objects if PoseClass is provided."""
    if PoseClass is not None:
        def _nan_pose():
            p = PoseClass()
            p.position.x = p.position.y = p.position.z = float('nan')
            p.orientation.w = 1.0
            return p
        output = [_nan_pose() for _ in range(NUM_JOINTS)]
    else:
        output = [_make_nan_pose() for _ in range(NUM_JOINTS)]

    for coco_idx, smpl_idx in COCO_TO_SMPL.items():
        if smpl_idx is not None:
            output[smpl_idx] = coco_poses[coco_idx]

    return output
