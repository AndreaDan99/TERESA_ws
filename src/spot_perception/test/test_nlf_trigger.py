"""Tests for NLF skeleton trigger callback logic — replicated in isolation.

ROS2-free — no imports from spot_perception. Uses a minimal FakeNLFSkeleton
class with mocked dependencies to test the 3-gate trigger behavior.
"""

import numpy as np
import pytest
from unittest.mock import Mock

# ═══════════════════════════════════════════════════════════════════════════════
#  SMPL joint index constants (source: sml_pose_indices.py)
# ═══════════════════════════════════════════════════════════════════════════════

NUM_JOINTS = 24
SHOULDER_LEFT = 16
SHOULDER_RIGHT = 17
HIP_LEFT = 1
HIP_RIGHT = 2

_UP = np.array([0.0, 1.0, 0.0])


def _angle_between(v1, v2):
    """Angle between two vectors in degrees."""
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    if norm < 1e-9:
        return 0.0
    cos = np.clip(np.dot(v1, v2) / norm, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos)))


# ═══════════════════════════════════════════════════════════════════════════════
#  FakeNLFSkeleton  —  minimal mock of NLFSkeletonNode for trigger testing
# ═══════════════════════════════════════════════════════════════════════════════

class FakeNLFSkeleton:
    """Minimal mock of NLFSkeletonNode without ROS2."""

    def __init__(self):
        self._nlf_ready = False
        self._last_color_msg = None
        self._lying_angle_min = 65.0
        self.bridge = Mock()
        self.bridge.imgmsg_to_cv2 = Mock(return_value=np.zeros((480, 640, 3), dtype=np.uint8))
        self.pub_nlf_prior = Mock()
        self._run_nlf_inference = Mock()
        self.get_logger = Mock(return_value=Mock())

    def _cb_trigger(self, msg):
        """Replicated from NLFSkeletonNode._cb_trigger() for testing."""
        if not msg.data:
            return
        if not self._nlf_ready:
            self.get_logger().warn('NLF trigger received but model not ready')
            return
        if self._last_color_msg is None:
            self.get_logger().warn('NLF trigger received but no image available')
            return

        img = self.bridge.imgmsg_to_cv2(self._last_color_msg, "rgb8")
        detections = self._run_nlf_inference(img)

        lying = []
        for det in detections:
            joints = det['joints3d']
            sh_l = joints[SHOULDER_LEFT]
            sh_r = joints[SHOULDER_RIGHT]
            hi_l = joints[HIP_LEFT]
            hi_r = joints[HIP_RIGHT]
            if all(x is not None and not np.isnan(x[0])
                   for x in [sh_l, sh_r, hi_l, hi_r]):
                sh_mid = (sh_l + sh_r) / 2.0
                hi_mid = (hi_l + hi_r) / 2.0
                torso_vec = sh_mid - hi_mid
                angle = _angle_between(torso_vec, _UP)
                if angle > self._lying_angle_min:
                    depth = float(sh_mid[2])
                    lying.append((depth, det))

        target_det = lying[0][1] if lying else (
            detections[0] if detections else None)

        # Fake PoseArray — use a Mock with a .poses list
        pa = Mock()
        pa.header = Mock()
        pa.header.frame_id = "orbbec_color_optical_frame"
        pa.header.stamp = self._last_color_msg.header.stamp
        pa.poses = []

        if target_det is not None:
            for j in range(NUM_JOINTS):
                pose = Mock()
                pose.position = Mock()
                pose.position.x = float(target_det['joints3d'][j][0])
                pose.position.y = float(target_det['joints3d'][j][1])
                pose.position.z = float(target_det['joints3d'][j][2])
                pose.orientation = Mock()
                pose.orientation.w = 1.0
                pa.poses.append(pose)

        self.pub_nlf_prior.publish(pa)
        if target_det is not None:
            self.get_logger().info('NLF prior published: 24 joints')


# ═══════════════════════════════════════════════════════════════════════════════
#  Helper: create a lying-person detection
# ═══════════════════════════════════════════════════════════════════════════════

def _make_lying_detection():
    """Return a detection dict with joints3d for a clearly lying person.

    Shoulders at +X, hips at -X, same Y/Z → torso horizontal → angle ≈90° > 65°.
    """
    joints = np.zeros((NUM_JOINTS, 3))
    joints[SHOULDER_LEFT]  = [0.2, 0.0, 2.0]
    joints[SHOULDER_RIGHT] = [0.4, 0.0, 2.0]
    joints[HIP_LEFT]       = [-0.4, 0.0, 2.0]
    joints[HIP_RIGHT]      = [-0.2, 0.0, 2.0]
    return {'joints3d': joints}


# ═══════════════════════════════════════════════════════════════════════════════
#  Gate 1: msg.data
# ═══════════════════════════════════════════════════════════════════════════════

def test_trigger_false_ignored():
    """msg.data = False → immediate return, no publish, no inference."""
    node = FakeNLFSkeleton()
    msg = Mock(data=False)
    node._cb_trigger(msg)
    node._run_nlf_inference.assert_not_called()
    node.pub_nlf_prior.publish.assert_not_called()


# ═══════════════════════════════════════════════════════════════════════════════
#  Gate 2: _nlf_ready
# ═══════════════════════════════════════════════════════════════════════════════

def test_trigger_model_not_ready():
    """msg.data = True, _nlf_ready = False → log warning, no publish."""
    node = FakeNLFSkeleton()
    node._nlf_ready = False
    msg = Mock(data=True)
    node._cb_trigger(msg)
    node.get_logger().warn.assert_called_once()
    node._run_nlf_inference.assert_not_called()
    node.pub_nlf_prior.publish.assert_not_called()


# ═══════════════════════════════════════════════════════════════════════════════
#  Gate 3: _last_color_msg
# ═══════════════════════════════════════════════════════════════════════════════

def test_trigger_no_image():
    """msg.data = True, model ready, but no image → log warning, no publish."""
    node = FakeNLFSkeleton()
    node._nlf_ready = True
    node._last_color_msg = None
    msg = Mock(data=True)
    node._cb_trigger(msg)
    node.get_logger().warn.assert_called_once()
    node._run_nlf_inference.assert_not_called()
    node.pub_nlf_prior.publish.assert_not_called()


# ═══════════════════════════════════════════════════════════════════════════════
#  Happy path: publish prior
# ═══════════════════════════════════════════════════════════════════════════════

def test_trigger_publishes_prior():
    """All conditions met → inference runs, nlf_prior published with 24 joints."""
    node = FakeNLFSkeleton()
    node._nlf_ready = True
    node._last_color_msg = Mock()
    node._last_color_msg.header = Mock()
    node._last_color_msg.header.stamp = Mock()

    node._run_nlf_inference.return_value = [_make_lying_detection()]

    msg = Mock(data=True)
    node._cb_trigger(msg)

    node._run_nlf_inference.assert_called_once()
    node.pub_nlf_prior.publish.assert_called_once()
    published = node.pub_nlf_prior.publish.call_args[0][0]
    assert len(published.poses) == 24
    assert published.header.frame_id == 'orbbec_color_optical_frame'


def test_trigger_publishes_prior_no_detections():
    """Inference returns empty list → publishes PoseArray with 0 poses."""
    node = FakeNLFSkeleton()
    node._nlf_ready = True
    node._last_color_msg = Mock()
    node._last_color_msg.header = Mock()
    node._last_color_msg.header.stamp = Mock()

    node._run_nlf_inference.return_value = []  # no detections

    msg = Mock(data=True)
    node._cb_trigger(msg)

    node.pub_nlf_prior.publish.assert_called_once()
    published = node.pub_nlf_prior.publish.call_args[0][0]
    assert len(published.poses) == 0


def test_trigger_skips_nan_torso_joints():
    """Detection with NaN in torso joints → not classified as lying.
    Falls through to first-detection fallback if other dets exist."""
    node = FakeNLFSkeleton()
    node._nlf_ready = True
    node._last_color_msg = Mock()
    node._last_color_msg.header = Mock()
    node._last_color_msg.header.stamp = Mock()

    # Detection with NaN in hip_right → not classified as lying
    joints = np.zeros((NUM_JOINTS, 3))
    joints[SHOULDER_LEFT]  = [0.2, 0.0, 2.0]
    joints[SHOULDER_RIGHT] = [0.4, 0.0, 2.0]
    joints[HIP_LEFT]       = [-0.4, 0.0, 2.0]
    joints[HIP_RIGHT]      = [np.nan, np.nan, np.nan]  # NaN → torso not valid
    non_lying = {'joints3d': joints}

    # Second detection: a valid lying person (used as fallback)
    lying = _make_lying_detection()

    node._run_nlf_inference.return_value = [non_lying, lying]

    msg = Mock(data=True)
    node._cb_trigger(msg)

    # Should still publish (lying detection exists)
    node.pub_nlf_prior.publish.assert_called_once()
    published = node.pub_nlf_prior.publish.call_args[0][0]
    assert len(published.poses) == 24
