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
