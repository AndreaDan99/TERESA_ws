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
