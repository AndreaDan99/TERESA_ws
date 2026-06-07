"""Tests for NLF prior gate logic: _nlf_prior_valid() replicated in isolation.

ROS2-free — no imports from spot_control. The gate logic is self-contained
and testable with only numpy and unittest.mock.
"""

import numpy as np
import pytest
from unittest.mock import Mock

# ═══════════════════════════════════════════════════════════════════════════════
#  Replicated logic from wbc_coordinator.py  (ROS2-free for testing)
# ═══════════════════════════════════════════════════════════════════════════════

# SMPL torso joint indices (source: sml_pose_indices.py)
PELVIS = 0
SPINE1 = 3
SPINE2 = 6
SPINE3 = 9


def _nlf_prior_valid(nlf_prior) -> bool:
    """Replicated from WBCCoordinatorNode._nlf_prior_valid() for testing."""
    if nlf_prior is None:
        return False
    if nlf_prior == 'timeout':
        return False
    if len(nlf_prior) != 24:
        return False
    valid_torso = sum(1 for j in [SPINE1, SPINE2, SPINE3, PELVIS]
                      if not np.any(np.isnan(nlf_prior[j])))
    return valid_torso >= 4


def _torso_center_from_prior(nlf_prior) -> np.ndarray:
    """Replicated from WBCCoordinatorNode._torso_center_from_prior()."""
    pts = [nlf_prior[j] for j in [SPINE1, SPINE2, SPINE3, PELVIS]
           if not np.any(np.isnan(nlf_prior[j]))]
    return np.mean(pts, axis=0) if pts else np.zeros(3)


# ═══════════════════════════════════════════════════════════════════════════════
#  _nlf_prior_valid()  —  8 test cases
# ═══════════════════════════════════════════════════════════════════════════════

def test_none_returns_false():
    """_nlf_prior = None → False."""
    assert _nlf_prior_valid(None) is False


def test_timeout_returns_false():
    """_nlf_prior = 'timeout' → False."""
    assert _nlf_prior_valid('timeout') is False


def test_wrong_length_returns_false():
    """12 joints instead of 24 → False."""
    prior = [np.zeros(3) for _ in range(12)]
    assert _nlf_prior_valid(prior) is False


def test_all_torso_nan_returns_false():
    """SPINE1(3), SPINE2(6), SPINE3(9), PELVIS(0) all NaN → False."""
    joints = [np.zeros(3) for _ in range(24)]
    for idx in (0, 3, 6, 9):  # PELVIS, SPINE1, SPINE2, SPINE3
        joints[idx] = np.array([np.nan, np.nan, np.nan])
    assert _nlf_prior_valid(joints) is False


def test_valid_prior_returns_true():
    """24 joints, all 4 torso joints valid → True."""
    joints = [np.zeros(3) for _ in range(24)]
    assert _nlf_prior_valid(joints) is True


def test_three_torso_valid_returns_false():
    """Threshold is ≥4 (all 4 torso joints must be valid). 3/4 → False."""
    joints = [np.zeros(3) for _ in range(24)]
    joints[9] = np.array([np.nan, np.nan, np.nan])  # SPINE3 NaN
    assert _nlf_prior_valid(joints) is False


def test_nan_in_non_torso_joint_returns_true():
    """NaN in a non-torso joint (e.g. index 15) does NOT affect gate."""
    joints = [np.zeros(3) for _ in range(24)]
    joints[15] = np.array([np.nan, np.nan, np.nan])  # non-torso joint
    assert _nlf_prior_valid(joints) is True


def test_partial_nan_in_torso():
    """A torso joint with [nan, 0, 0] is still NaN → counts as invalid."""
    joints = [np.zeros(3) for _ in range(24)]
    joints[3] = np.array([np.nan, 0.0, 0.0])  # SPINE1 with mixed NaN
    # only 3 torso joints are fully non-NaN → threshold not met
    assert _nlf_prior_valid(joints) is False


# ═══════════════════════════════════════════════════════════════════════════════
#  _torso_center_from_prior()  —  3 extra test cases
# ═══════════════════════════════════════════════════════════════════════════════

def test_torso_center_four_valid():
    joints = [np.zeros(3) for _ in range(24)]
    joints[0] = np.array([0.0, 0.0, 0.0])   # PELVIS
    joints[3] = np.array([0.0, 0.0, 0.1])   # SPINE1
    joints[6] = np.array([0.0, 0.0, 0.2])   # SPINE2
    joints[9] = np.array([0.0, 0.0, 0.3])   # SPINE3
    center = _torso_center_from_prior(joints)
    np.testing.assert_allclose(center, [0.0, 0.0, 0.15])


def test_torso_center_excludes_nan():
    """SPINE1 is NaN → excluded from mean (3 valid points used)."""
    joints = [np.zeros(3) for _ in range(24)]
    joints[0] = np.array([0.0, 0.0, 0.0])        # PELVIS
    joints[3] = np.array([np.nan, np.nan, np.nan])  # SPINE1 — excluded
    joints[6] = np.array([0.0, 0.0, 0.2])        # SPINE2
    joints[9] = np.array([0.0, 0.0, 0.3])        # SPINE3
    center = _torso_center_from_prior(joints)
    np.testing.assert_allclose(center, [0.0, 0.0, (0.0 + 0.2 + 0.3) / 3.0])


def test_torso_center_all_nan_returns_zeros():
    """All 4 torso joints NaN → returns np.zeros(3)."""
    joints = [np.zeros(3) for _ in range(24)]
    for idx in (0, 3, 6, 9):
        joints[idx] = np.array([np.nan, np.nan, np.nan])
    center = _torso_center_from_prior(joints)
    np.testing.assert_allclose(center, [0.0, 0.0, 0.0])
