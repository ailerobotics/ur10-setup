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
