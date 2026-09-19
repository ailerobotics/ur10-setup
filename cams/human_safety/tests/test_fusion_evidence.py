# SPDX-License-Identifier: Apache-2.0
"""Regression for finding ``latency-basis``: the STOP-ring evidence stamp and the
image->state latency (fault 5) must be derived from the positioned detections
that actually localise a person, using the OLDEST contributing stamp -- never
from a camera streaming fresh empty frames. Fails on the old max-over-every-frame
basis, passes on the oldest-positioned basis."""
import numpy as np
import pytest

from human_safety.fusion import Fusion, gather_evidence
from human_safety.messages import Detection3D, Method, Zone
from human_safety.safety import SafetyEvaluator, SafetyParams


def test_lagging_localiser_escalates_despite_fresh_empty_frames():
    now = 1000.0
    lag = 0.9  # the localising camera's frame arrives now but is ~0.9 s old
    # 'wide' localises a person (positioned) on a frame whose image stamp lags
    # 0.9 s (capture/transport lag the per-camera frame_age does not see).
    person = Detection3D(camera="wide", stamp=now - lag, score=0.9,
                         method=Method.DEPTH,
                         position=np.array([2.5, 0.0, 0.9]), sigma=0.15)
    # 'shelf' streams a FRESH but empty frame (zero detections) this cycle.
    dets_by_role = {
        "wide": (now, [person], now - lag),
        "shelf": (now, [], now - 0.02),
    }
    all_dets, evidence_stamp, latency = gather_evidence(
        dets_by_role, now, frame_age_budget=0.5)

    # The fresh empty frame must not make the older person evidence look fresh.
    assert evidence_stamp == now - lag
    assert latency == pytest.approx(lag)

    ev = SafetyEvaluator(SafetyParams())   # reaction_time_budget default 0.7
    fusion = Fusion()
    tracks, presence_cams = fusion.step(all_dets, now)
    state = ev.evaluate(tracks, [], now, any_presence=bool(presence_cams),
                        source_stamp=evidence_stamp, latency=latency)

    assert state.zone == Zone.STOP
    assert state.evidence_stamp == now - lag   # the lagging localiser, not 'shelf'
    assert any("latency" in r for r in state.reasons)


def test_oldest_positioned_stamp_wins_over_a_fresher_localiser():
    now = 2000.0
    # Two cameras both localise the same region; take the OLDER stamp (conservative).
    older = Detection3D(camera="wide", stamp=now - 0.8, score=0.9,
                        method=Method.DEPTH, position=np.array([2.0, 0.0, 0.9]))
    fresher = Detection3D(camera="close", stamp=now - 0.1, score=0.9,
                          method=Method.DEPTH, position=np.array([2.05, 0.0, 0.9]))
    dets_by_role = {
        "wide": (now, [older], now - 0.8),
        "close": (now, [fresher], now - 0.1),
    }
    _, evidence_stamp, latency = gather_evidence(
        dets_by_role, now, frame_age_budget=0.5)
    assert evidence_stamp == now - 0.8
    assert latency == pytest.approx(0.8)


def test_no_localised_person_reports_no_latency():
    now = 500.0
    # Only fresh empty / presence-only frames -> nobody is being localised, so
    # there is no evidence stamp and no latency that could imply fresh evidence.
    presence = Detection3D(camera="shelf", stamp=now - 0.03, score=0.8,
                           method=Method.PRESENCE, present=True, position=None)
    dets_by_role = {
        "shelf": (now, [presence], now - 0.03),
        "wide": (now, [], now - 0.01),
    }
    _, evidence_stamp, latency = gather_evidence(
        dets_by_role, now, frame_age_budget=0.5)
    assert evidence_stamp is None
    assert latency is None
