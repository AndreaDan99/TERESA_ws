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
