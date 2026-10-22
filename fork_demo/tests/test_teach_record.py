#!/usr/bin/env python3
"""Offline tests for teach dwell detection. No ROS.

Run:  python3 -I fork_demo/tests/test_teach_record.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # fork_demo/

from teach_record import detect_dwells, _rotate  # noqa: E402


def _samples(segments):
    """segments: list of (duration_s, (x,y,z), dt). Build evenly-spaced samples."""
    out = []
    t = 0.0
    for dur, pos, dt in segments:
        n = int(dur / dt) + 1
        for _ in range(n):
            out.append({"t": round(t, 4), "fork": pos, "joints": [0, 0, 0, 0, 0, 0]})
            t += dt
    return out


def test_single_dwell():
    s = _samples([(3.0, (1.0, 0.0, 0.3), 0.1)])
    d = detect_dwells(s, dwell_sec=2.0, move_tol_m=0.005)
    assert len(d) == 1
    assert d[0]["duration_s"] >= 2.0
    assert abs(d[0]["fork_mean"][0] - 1.0) < 1e-6


def test_motion_between_two_dwells_is_not_a_dwell():
    s = _samples([
        (3.0, (0.0, 0.0, 0.3), 0.1),     # dwell 1
        (0.5, (0.2, 0.0, 0.3), 0.1),     # moving (too short + will be split)
        (2.5, (0.5, 0.0, 0.3), 0.1),     # dwell 2
    ])
    d = detect_dwells(s, dwell_sec=2.0, move_tol_m=0.005)
    assert len(d) == 2
    assert abs(d[0]["fork_mean"][0] - 0.0) < 1e-6
    assert abs(d[1]["fork_mean"][0] - 0.5) < 1e-6


def test_brief_pause_below_threshold_is_ignored():
    s = _samples([(1.0, (0.0, 0.0, 0.3), 0.1)])  # only 1 s < 2 s dwell
    assert detect_dwells(s, dwell_sec=2.0, move_tol_m=0.005) == []


def test_jitter_within_tolerance_still_counts():
    # positions wobble by <= 2 mm (4 mm p2p < 5 mm tol) for 3 s -> one dwell
    s = []
    t = 0.0
    for i in range(31):
        wob = 0.002 if i % 2 else -0.002
        s.append({"t": round(t, 4), "fork": (1.0 + wob, 0.0, 0.3),
                  "joints": [0] * 6})
        t += 0.1
    d = detect_dwells(s, dwell_sec=2.0, move_tol_m=0.005)
    assert len(d) == 1


def test_fork_offset_rotation():
    # identity orientation: fork tip = tool0 + offset straight
    assert _rotate((0, 0, 0, 1), (0, -0.038, 0.427)) == (0, -0.038, 0.427)


def _run():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} tests passed")


if __name__ == "__main__":
    _run()
