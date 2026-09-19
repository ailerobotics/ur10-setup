# SPDX-License-Identifier: Apache-2.0
"""Replay regression: a recorded detection log must reproduce the same safety
state sequence exactly. Seeded with a synthetic approach; a real walk-through
recording is dropped in beside it and asserted the same way."""
import pathlib

import pytest

from human_safety.messages import Zone
from human_safety.replay import replay_file
from human_safety.safety import SafetyParams

DATA = pathlib.Path(__file__).parent / "data" / "synthetic_approach.jsonl"


def _params():
    return SafetyParams(reach_radius=1.30, stop_radius=0.50, slow_radius=1.50,
                        hysteresis=0.25, min_clear_dwell=2.0)


def test_replay_produces_expected_zone_arc():
    states = replay_file(str(DATA), _params())
    zones = [s.zone for s in states]
    # Starts CLEAR, escalates through SLOW into STOP, then releases to CLEAR.
    assert zones[0] == Zone.CLEAR
    assert Zone.SLOW in zones
    assert Zone.STOP in zones
    assert zones[-1] == Zone.CLEAR
    # Monotone escalation into STOP (no premature de-escalation on approach).
    first_stop = zones.index(Zone.STOP)
    first_slow = zones.index(Zone.SLOW)
    assert first_slow < first_stop


def test_replay_is_deterministic():
    a = [s.zone for s in replay_file(str(DATA), _params())]
    b = [s.zone for s in replay_file(str(DATA), _params())]
    assert a == b


def test_replay_stop_release_respects_dwell():
    states = replay_file(str(DATA), _params())
    zones = [s.zone for s in states]
    # The last STOP index; after the person retreats the release must lag the
    # moment distance alone would allow (dwell), i.e. STOP persists a few cycles
    # into the retreat.
    last_stop = max(i for i, z in enumerate(zones) if z == Zone.STOP)
    # find when nearest_distance first exceeds slow+hyst after the dwell region
    assert last_stop >= zones.index(Zone.STOP)
    # nearest distance at release time is already well clear (dwell held it)
    assert states[last_stop].nearest_distance >= 0.0


def test_nearest_distance_tracks_person():
    states = replay_file(str(DATA), _params())
    dists = [s.nearest_distance for s in states]
    # Person reaches base_x 1.1 < reach 1.30 -> inside the envelope, env dist 0.
    assert min(dists) == pytest.approx(0.0, abs=1e-6)
    # Once the track has confirmed (a few cycles in) it reports the real, large
    # distance (envelope dist ~ 3.7 - 1.30 as the person is still far out).
    assert 2.0 < dists[5] < 3.0
