# SPDX-License-Identifier: Apache-2.0
"""compare_fix.reconstruct must replay recorded live states through the safety
machine with old (centre) vs new (nearest) logic, differing only by the distance
rule."""
import compare_fix
from human_safety.config import load_config
from human_safety.safety import SafetyParams


def _cam(role="wrist"):
    return {"camera": role, "status": "fresh", "present_in_graph": True,
            "frame_age": 0.05, "detector_alive": True, "positioned": True,
            "fresh": True}


def _rec(t, centre_base, nearest_base):
    # position on +x so distance_to_base == x; z arbitrary.
    return {
        "kind": "state", "t": t, "evidence_stamp": t - 0.1,
        "distance_status": "known",
        "tracks": [{"id": 1, "position": [centre_base, 0.0, 0.9],
                    "nearest_point": [nearest_base, 0.0, 0.9], "confirmed": True}],
        "cameras": [_cam()],
    }


def test_new_uses_nearest_old_uses_centre():
    p = SafetyParams()   # reach 1.30, stop 0.70 -> STOP boundary env 0.70
    # centre base 2.3 -> env 1.0 (SLOW); hand base 1.5 -> env 0.2 (STOP).
    recs = [_rec(0.1 * i, 2.3, 1.5) for i in range(1, 6)]
    old = compare_fix.reconstruct(recs, p, use_nearest=False)
    new = compare_fix.reconstruct(recs, p, use_nearest=True)
    assert old[-1]["zone"] == "SLOW"        # centre never brings STOP
    assert new[-1]["zone"] == "STOP"        # the hand does
    # published distance: new reflects the hand (~0.2 env), old the centre (~1.0)
    assert new[-1]["nearest_distance"] < 0.5
    assert old[-1]["nearest_distance"] > 0.8
    # more STOP time under the fix; the old logic misses it entirely
    assert compare_fix.stop_time_s(new) > compare_fix.stop_time_s(old)


def test_reconstruction_matches_without_nearest_gap():
    # When nearest == centre every frame, old and new agree (no drift).
    p = SafetyParams()
    recs = [_rec(0.1 * i, 2.3, 2.3) for i in range(1, 6)]
    old = compare_fix.reconstruct(recs, p, use_nearest=False)
    new = compare_fix.reconstruct(recs, p, use_nearest=True)
    assert [s["zone"] for s in old] == [s["zone"] for s in new]


def _rec_lat(t, centre_base, nearest_base, latency):
    r = _rec(t, centre_base, nearest_base)
    r["evidence_stamp"] = t - latency
    return r


def test_latency_fault_masks_stop_until_disabled():
    # A person FAR outside the STOP ring (no distance STOP), but the pipeline
    # latency blows the reaction budget every frame -> the live/latency-ON replay
    # STOPs (fail-safe), which would MASK the distance behaviour. With the latency
    # fault disabled, the same records are CLEAR/SLOW (distance-driven). This is
    # the confound the distance-only mode exists to expose.
    p = SafetyParams()   # reaction_time_budget 0.7; person at base 3.0 -> env 1.7
    recs = [_rec_lat(0.1 * i, 3.0, 3.0, latency=1.2) for i in range(1, 6)]
    on = compare_fix.reconstruct(recs, p, use_nearest=True, ignore_latency_fault=False)
    off = compare_fix.reconstruct(recs, p, use_nearest=True, ignore_latency_fault=True)
    assert on[-1]["zone"] == "STOP"        # latency fault forces fail-safe STOP
    assert off[-1]["zone"] != "STOP"       # distance-only: not a STOP (person far)


def test_distance_only_reveals_a_late_stop_the_latency_fault_hid():
    # Person walks inside the ring, but latency is over budget the whole time, so
    # the latency-ON replay is STOP throughout (looks perfect). Distance-only shows
    # the truth: with a persistent +0.6 m over-stating nearest, STOP engages late.
    p = SafetyParams()
    # centre far, nearest reads 2.7 base while the person is actually at 2.0 (env
    # 0.7 == exactly the ring) -> distance-only would NOT STOP (nearest over-states)
    recs = [_rec_lat(0.1 * i, 4.0, 2.7, latency=1.2) for i in range(1, 8)]
    on = compare_fix.reconstruct(recs, p, use_nearest=True, ignore_latency_fault=False)
    off = compare_fix.reconstruct(recs, p, use_nearest=True, ignore_latency_fault=True)
    assert all(s["zone"] == "STOP" for s in on)      # latency masks everything
    assert all(s["zone"] != "STOP" for s in off)     # distance-only: nearest 2.7 -> env 1.4, no STOP
