"""Unit tests for sml_pose_indices.py — SMPL-24 joint index constants."""

import math
import pytest
from spot_perception.sml_pose_indices import (
    PELVIS, HIP_LEFT, HIP_RIGHT, SPINE1, KNEE_LEFT, KNEE_RIGHT,
    SPINE2, ANKLE_LEFT, ANKLE_RIGHT, SPINE3, FOOT_LEFT, FOOT_RIGHT,
    NECK, COLLAR_LEFT, COLLAR_RIGHT, HEAD, SHOULDER_LEFT, SHOULDER_RIGHT,
    ELBOW_LEFT, ELBOW_RIGHT, WRIST_LEFT, WRIST_RIGHT, HAND_LEFT, HAND_RIGHT,
    NUM_JOINTS, TORSO_JOINTS, ARM_JOINTS, LEG_JOINTS,
    NEVER_AVAILABLE_YOLO, is_valid,
)


class MockPoint:
    def __init__(self, x):
        self.x = x


class MockPose:
    def __init__(self, x):
        self.position = MockPoint(x)


# ── Constants ──────────────────────────────────────────────────────────────────

def test_num_joints():
    """NUM_JOINTS must be exactly 24."""
    assert NUM_JOINTS == 24


def test_torso_joints_count():
    """TORSO_JOINTS must contain exactly 9 joints."""
    assert len(TORSO_JOINTS) == 9


def test_arm_joints_count():
    """ARM_JOINTS must contain exactly 6 joints."""
    assert len(ARM_JOINTS) == 6


def test_leg_joints_count():
    """LEG_JOINTS must contain exactly 6 joints."""
    assert len(LEG_JOINTS) == 6


def test_never_available_yolo():
    """NEVER_AVAILABLE_YOLO must contain 9 joints always NaN in YOLO."""
    assert len(NEVER_AVAILABLE_YOLO) == 9


# ── Validation ─────────────────────────────────────────────────────────────────

def test_is_valid():
    """is_valid must return True for valid poses and False for NaN poses."""
    assert is_valid(MockPose(1.0)) is True
    assert is_valid(MockPose(float("nan"))) is False


# ── Uniqueness ─────────────────────────────────────────────────────────────────

def test_all_constants_unique():
    """All 24 joint index constants must be unique (no duplicates)."""
    all_joints = [
        PELVIS, HIP_LEFT, HIP_RIGHT, SPINE1, KNEE_LEFT, KNEE_RIGHT,
        SPINE2, ANKLE_LEFT, ANKLE_RIGHT, SPINE3, FOOT_LEFT, FOOT_RIGHT,
        NECK, COLLAR_LEFT, COLLAR_RIGHT, HEAD, SHOULDER_LEFT, SHOULDER_RIGHT,
        ELBOW_LEFT, ELBOW_RIGHT, WRIST_LEFT, WRIST_RIGHT, HAND_LEFT, HAND_RIGHT,
    ]
    assert len(all_joints) == 24
    assert len(set(all_joints)) == 24, "Duplicate joint indices found!"
