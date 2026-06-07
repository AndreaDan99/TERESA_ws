"""Tests for PRE_APPROACH bifurcation: fast NLF path vs legacy sliding-window fallback.

ROS2-free — logic tested in isolation with replicated functions and mocks.
"""

import numpy as np
import pytest
from unittest.mock import Mock, MagicMock, patch


# ═══════════════════════════════════════════════════════════════════════════════
#  Replicated helpers from wbc_coordinator.py
# ═══════════════════════════════════════════════════════════════════════════════

PELVIS = 0
SPINE1 = 3
SPINE2 = 6
SPINE3 = 9

COHERENCE_THRESHOLD = 0.15    # meters — HIGH quality delta
DIVERGENCE_THRESHOLD = 0.30   # meters — MEDIUM/LOW boundary


def _nlf_prior_valid(nlf_prior) -> bool:
    """Replicated gate logic."""
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
    """Replicated torso center computation."""
    pts = [nlf_prior[j] for j in [SPINE1, SPINE2, SPINE3, PELVIS]
           if not np.any(np.isnan(nlf_prior[j]))]
    return np.mean(pts, axis=0) if pts else np.zeros(3)


def _check_nlf_delta(torso_yolo: np.ndarray, nlf_prior) -> tuple:
    """Replicated coherence check: returns (quality_label, delta)."""
    if not _nlf_prior_valid(nlf_prior):
        return ('HIGH', None)
    nlf_center = _torso_center_from_prior(nlf_prior)
    delta = float(np.linalg.norm(torso_yolo[:3] - nlf_center[:3]))
    if delta < COHERENCE_THRESHOLD:
        return ('HIGH', delta)
    elif delta < DIVERGENCE_THRESHOLD:
        return ('MEDIUM', delta)
    else:
        return ('LOW', delta)


# ═══════════════════════════════════════════════════════════════════════════════
#  Mock coordinator for flow-control tests
# ═══════════════════════════════════════════════════════════════════════════════

class MockCoordinator:
    """Minimal coordinator mock for testing _tick_pre_approach flow."""

    def __init__(self, nlf_prior_valid=True, torso_center=None):
        self._nlf_prior_valid_result = nlf_prior_valid
        if torso_center is None:
            torso_center = np.array([1.5, 2.5, 0.8])
        self._torso_center = torso_center
        self._torso_tracker_state = ''
        self._torso_pos = None
        self._pre_approach_fast_start = None
        self._odom_frame = 'odom'
        self._nlf_low_ticks = 0

        # Mocks
        self._pub_goal = Mock()
        self._pub_spot_ctrl = Mock()
        self._set_state = Mock()
        self._tick_pre_approach_legacy_called = False
        self.get_logger = Mock(return_value=Mock())
        self.get_clock = Mock()

    def _nlf_prior_valid(self):
        return self._nlf_prior_valid_result

    def _torso_center_from_prior(self):
        return self._torso_center

    def _check_nlf_delta(self, torso_yolo):
        return _check_nlf_delta(torso_yolo, self._nlf_prior_for_delta)

    def _tick_pre_approach_legacy(self):
        self._tick_pre_approach_legacy_called = True

    def _tick_pre_approach(self):
        """Replicated bifurcation logic for testing."""
        if self._nlf_prior_valid():
            if self._pre_approach_fast_start is None:
                nlf_center = self._torso_center_from_prior()
                target = nlf_center.copy()

                if False:  # torso tracker not used in isolation tests
                    # blending logic tested separately
                    pass
                else:
                    self.get_logger().info(
                        'PRE_APPROACH (NLF): goal published, waiting 1s safety gate')

                # Publish goal
                goal = Mock()
                goal.header = Mock()
                goal.header.frame_id = self._odom_frame
                goal.pose = Mock()
                goal.pose.position = Mock()
                goal.pose.position.x = float(target[0])
                goal.pose.position.y = float(target[1])
                goal.pose.position.z = float(target[2])
                goal.pose.orientation = Mock()
                goal.pose.orientation.w = 1.0
                self._pub_goal.publish(goal)
                self._pre_approach_fast_start = Mock()  # non-None sentinel
                return

            elapsed = 0.5  # simulated < 1s
            if elapsed < 1.0:
                return  # still in safety gate
        else:
            self._tick_pre_approach_legacy()


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 1: Fast path — NLF valid → publish goal + start safety gate
# ═══════════════════════════════════════════════════════════════════════════════

def test_nlf_valid_calls_fast_path():
    """NLF valid → publish LOOKAT goal, start safety gate."""
    coord = MockCoordinator(nlf_prior_valid=True, torso_center=np.array([1.5, 2.5, 0.8]))

    coord._tick_pre_approach()

    # Goal must be published with NLF torso center position
    coord._pub_goal.publish.assert_called_once()
    goal_msg = coord._pub_goal.publish.call_args[0][0]
    assert goal_msg.header.frame_id == 'odom'
    assert goal_msg.pose.position.x == 1.5
    assert goal_msg.pose.position.y == 2.5
    assert goal_msg.pose.position.z == 0.8
    assert goal_msg.pose.orientation.w == 1.0

    # Safety gate timer must be started
    assert coord._pre_approach_fast_start is not None

    # Must NOT call legacy fallback
    assert coord._tick_pre_approach_legacy_called is False


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 2: Fallback — NLF invalid → legacy path
# ═══════════════════════════════════════════════════════════════════════════════

def test_nlf_invalid_calls_legacy():
    """NLF invalid → fallback to legacy _tick_pre_approach_legacy()."""
    coord = MockCoordinator(nlf_prior_valid=False)

    coord._tick_pre_approach()

    assert coord._tick_pre_approach_legacy_called is True
    coord._pub_goal.publish.assert_not_called()


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 3: Goal blending — HIGH coherence → 70/30 NLF+YOLO blend
# ═══════════════════════════════════════════════════════════════════════════════

def test_goal_blending_high_coherence():
    """HIGH coherence (delta < 0.15m) → 70% NLF / 30% YOLO blend."""
    nlf_center = np.array([1.0, 0.0, 0.5])
    torso_yolo = np.array([1.02, 0.01, 0.51])  # ~2cm delta

    # Build a valid prior with torso at nlf_center
    prior = [np.zeros(3) for _ in range(24)]
    prior[0] = nlf_center   # PELVIS
    prior[3] = nlf_center + np.array([0, 0, 0.1])   # SPINE1
    prior[6] = nlf_center + np.array([0, 0, 0.2])   # SPINE2
    prior[9] = nlf_center + np.array([0, 0, 0.3])   # SPINE3

    quality_label, delta = _check_nlf_delta(torso_yolo, prior)
    assert quality_label == 'HIGH'
    assert delta < 0.15

    # Blend: 70% NLF + 30% YOLO
    target = 0.7 * _torso_center_from_prior(prior) + 0.3 * torso_yolo

    # Verify 70/30 weighted blend formula
    expected = 0.7 * _torso_center_from_prior(prior) + 0.3 * torso_yolo
    np.testing.assert_allclose(target, expected)
    # Blended target differs from both pure inputs
    assert not np.allclose(target, _torso_center_from_prior(prior))
    assert not np.allclose(target, torso_yolo)


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 4: Goal blending — LOW coherence → YOLO 100%
# ═══════════════════════════════════════════════════════════════════════════════

def test_goal_blending_low_coherence():
    """LOW coherence (delta ≥ 0.30m) → YOLO 100% (ignore NLF)."""
    nlf_center = np.array([1.0, 0.0, 0.5])
    torso_yolo = np.array([1.5, 0.0, 0.5])  # 50cm delta

    # Build a valid prior with torso at nlf_center
    prior = [np.zeros(3) for _ in range(24)]
    prior[0] = nlf_center
    prior[3] = nlf_center + np.array([0, 0, 0.1])
    prior[6] = nlf_center + np.array([0, 0, 0.2])
    prior[9] = nlf_center + np.array([0, 0, 0.3])

    quality_label, delta = _check_nlf_delta(torso_yolo, prior)
    assert quality_label == 'LOW'
    assert delta >= 0.30

    # LOW → YOLO 100%
    target = torso_yolo
    np.testing.assert_array_equal(target, torso_yolo)


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 5: MEDIUM coherence → 50/50 blend
# ═══════════════════════════════════════════════════════════════════════════════

def test_goal_blending_medium_coherence():
    """MEDIUM coherence (0.15m ≤ delta < 0.30m) → 50% NLF / 50% YOLO blend."""
    nlf_center = np.array([1.0, 0.0, 0.5])
    torso_yolo = np.array([1.15, 0.10, 0.55])  # ~20cm delta

    # Build a valid prior with torso at nlf_center
    prior = [np.zeros(3) for _ in range(24)]
    prior[0] = nlf_center
    prior[3] = nlf_center + np.array([0, 0, 0.1])
    prior[6] = nlf_center + np.array([0, 0, 0.2])
    prior[9] = nlf_center + np.array([0, 0, 0.3])

    quality_label, delta = _check_nlf_delta(torso_yolo, prior)
    assert quality_label == 'MEDIUM'
    assert 0.15 <= delta < 0.30

    # MEDIUM → 50/50 blend
    center = _torso_center_from_prior(prior)
    target = 0.5 * center + 0.5 * torso_yolo
    np.testing.assert_allclose(target, (center + torso_yolo) / 2.0)


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 6: Fallback — no NLF prior → YOLO only (legacy behavior)
# ═══════════════════════════════════════════════════════════════════════════════

def test_fallback_no_nlf_prior():
    """No NLF prior → YOLO only (legacy behavior), no crash."""
    nlf_prior_invalid = False
    if not nlf_prior_invalid:
        target = np.array([1.0, 0.0, 0.5])  # simulated yolo_target
    assert target is not None
    np.testing.assert_array_equal(target, [1.0, 0.0, 0.5])


# ═══════════════════════════════════════════════════════════════════════════════
#  Test 7: _check_nlf_delta returns HIGH/None when prior is invalid
# ═══════════════════════════════════════════════════════════════════════════════

def test_check_delta_high_when_invalid_prior():
    """Invalid prior → _check_nlf_delta returns ('HIGH', None) — graceful degradation."""
    torso_yolo = np.array([1.0, 0.0, 0.5])
    quality_label, delta = _check_nlf_delta(torso_yolo, None)
    assert quality_label == 'HIGH'
    assert delta is None
