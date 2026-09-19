# SPDX-License-Identifier: Apache-2.0
"""Offline tracker + association tests with synthetic detections."""
import numpy as np
import pytest

from human_safety.messages import Detection3D, Method
from human_safety.tracker import MultiTargetTracker


def det(cam, stamp, pos, sigma=0.1):
    p = np.asarray(pos, dtype=float)
    return Detection3D(camera=cam, stamp=stamp, score=0.9, method=Method.DEPTH,
                       position=p, nearest_point=p.copy(), sigma=sigma,
                       n_depth_valid=500)


def test_single_target_confirms_and_tracks_velocity():
    trk = MultiTargetTracker(n_confirm=3)
    pos = np.array([2.0, 0.0, 0.9])
    vel = np.array([-0.5, 0.0, 0.0])
    t = 0.0
    tracks = []
    for i in range(6):
        tracks = trk.step([det("shelf", t, pos + vel * t)], t)
        t += 0.1
    assert len(tracks) == 1
    tk = tracks[0]
    assert tk.confirmed
    assert tk.velocity[0] == pytest.approx(-0.5, abs=0.2)
    assert tk.position[0] == pytest.approx(pos[0] + vel[0] * 0.5, abs=0.1)


def test_two_targets_keep_distinct_stable_ids():
    trk = MultiTargetTracker(n_confirm=2)
    a = np.array([2.0, -1.0, 0.9])
    b = np.array([2.0, 1.0, 0.9])
    ids_a, ids_b = set(), set()
    t = 0.0
    for i in range(5):
        tracks = trk.step([det("shelf", t, a), det("table", t, b)], t)
        t += 0.1
        # map by y sign
        for tk in tracks:
            (ids_a if tk.position[1] < 0 else ids_b).add(tk.id)
    assert len(ids_a) == 1 and len(ids_b) == 1
    assert ids_a != ids_b


def test_multi_camera_fusion_updates_same_track():
    trk = MultiTargetTracker(n_confirm=2)
    p = np.array([1.5, 0.2, 0.9])
    t = 0.0
    for i in range(4):
        cam = "shelf" if i % 2 == 0 else "table"
        tracks = trk.step([det(cam, t, p)], t)
        t += 0.1
    assert len(tracks) == 1
    assert set(tracks[0].cameras) == {"shelf", "table"}


def test_unmatched_detection_spawns_new_track():
    trk = MultiTargetTracker(n_confirm=2)
    trk.step([det("shelf", 0.0, [2.0, 0.0, 0.9])], 0.0)
    tracks = trk.step([det("shelf", 0.1, [2.0, 0.0, 0.9]),
                       det("table", 0.1, [-2.0, 0.0, 0.9])], 0.1)
    assert len(tracks) == 2


def test_far_detection_does_not_hijack_track_gating():
    trk = MultiTargetTracker(n_confirm=2)
    trk.step([det("shelf", 0.0, [2.0, 0.0, 0.9])], 0.0)
    # a detection 5 m away is outside the gate -> new track, not an update
    tracks = trk.step([det("shelf", 0.1, [7.0, 0.0, 0.9])], 0.1)
    assert len(tracks) == 2


def test_track_coasts_then_deleted():
    trk = MultiTargetTracker(n_confirm=2, max_coast_time=0.5,
                             max_confirmed_coast_time=0.5)
    p = [2.0, 0.0, 0.9]
    trk.step([det("shelf", 0.0, p)], 0.0)
    trk.step([det("shelf", 0.1, p)], 0.1)   # confirmed
    # now miss frames
    assert len(trk.step([], 0.2)) == 1      # coasting
    assert len(trk.step([], 0.4)) == 1      # still within budget
    assert len(trk.step([], 0.8)) == 0      # exceeded budget -> deleted


def test_presence_only_detections_ignored_by_tracker():
    trk = MultiTargetTracker()
    pres = Detection3D(camera="wrist", stamp=0.0, score=0.9,
                       method=Method.PRESENCE, present=True, position=None)
    assert trk.step([pres], 0.0) == []
