"""Unit tests for yolo_to_smpl_pad.py — COCO 17 → SMPL 24 adapter."""

import math
import pytest
from spot_perception.yolo_to_smpl_pad import coco_to_smpl_24, COCO_TO_SMPL
from spot_perception.sml_pose_indices import NUM_JOINTS


class MockPoint:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class MockPose:
    def __init__(self, x, y, z):
        self.position = MockPoint(x, y, z)


# ── Output size ────────────────────────────────────────────────────────────────

def test_output_size():
    """coco_to_smpl_24 must always return exactly 24 poses."""
    poses = [MockPose(1.0, 2.0, 3.0) for _ in range(17)]
    result = coco_to_smpl_24(poses)
    assert len(result) == NUM_JOINTS


# ── Mapping correctness ────────────────────────────────────────────────────────

def test_shoulder_mapping():
    """COCO index 5 (shoulder_left) → SMPL index 16 (SHOULDER_LEFT)."""
    poses = [MockPose(0, 0, 0) for _ in range(17)]
    poses[5] = MockPose(1.0, 2.0, 3.0)
    result = coco_to_smpl_24(poses)
    assert result[16].position.x == 1.0
    assert result[16].position.y == 2.0
    assert result[16].position.z == 3.0


def test_hip_mapping():
    """COCO index 11 (hip_left) → SMPL index 1 (HIP_LEFT)."""
    poses = [MockPose(0, 0, 0) for _ in range(17)]
    poses[11] = MockPose(5.0, 6.0, 7.0)
    result = coco_to_smpl_24(poses)
    assert result[1].position.x == 5.0


# ── NaN padding ────────────────────────────────────────────────────────────────

def test_spine_nan():
    """SMPL spine joints (indices 3, 6, 9) must be NaN in YOLO output."""
    poses = [MockPose(1.0, 1.0, 1.0) for _ in range(17)]
    result = coco_to_smpl_24(poses)
    for i in [3, 6, 9]:
        assert math.isnan(result[i].position.x), f"SMPL index {i} should be NaN"


def test_head_nan():
    """SMPL HEAD (index 15) must be NaN in YOLO output."""
    poses = [MockPose(1.0, 1.0, 1.0) for _ in range(17)]
    result = coco_to_smpl_24(poses)
    assert math.isnan(result[15].position.x)
