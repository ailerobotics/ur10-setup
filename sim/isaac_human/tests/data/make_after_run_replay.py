#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate the compact `after_run_replay/` fixture (record.jsonl +
ground_truth.jsonl) used by `test_after_run_replay.py`.

This is a SCRIPTED, deterministic stand-in for one Isaac after-run recording of
the nearest-point safety fix (task human-safety-nearest-point-fix-v1, report
`data/human-safety-nearest-point-fix-v1/report.md`). It is written in the SAME
schema the live Isaac recorder emits (`record.jsonl` state records +
`ground_truth.jsonl`), so it feeds the REAL consumer `compare_fix.build_comparison`
unchanged. It reproduces the after-run's headline behaviour so the fix can be
regression-guarded offline (no ROS, no Isaac, no GPU):

  * one person walks in toward the base past the 2.0 m STOP ring, dwells inside,
    and retreats back out;
  * every state carries BOTH the body-centre (`position`) and the nearest body
    point (`nearest_point`); the centre over-states the true nearest body
    distance more than the nearest point does (the +0.24 vs +0.19 m of the
    report's after-run ablation), so replaying the SAME records through the old
    (centre) vs new (nearest) zone rule reproduces the fix's before/after.

IMPORTANT (honesty): this is NOT a raw dump of the Isaac run and NOT a real-rig
walk-through. No real-rig human walk-through recording exists yet; the raw Isaac
bags/frames were intentionally not committed (report "Reproduce" section). Replace
this fixture with a real recording when one is captured and extend the test.

Re-run to regenerate:

    python3 sim/isaac_human/tests/data/make_after_run_replay.py
"""
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(os.path.dirname(_HERE))
_REPO = os.path.dirname(os.path.dirname(_HARNESS))
for _p in (_HARNESS, os.path.join(_REPO, "cams", "human_safety")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402

from human_safety.messages import CameraHealth, Track  # noqa: E402
from human_safety.safety import SafetyEvaluator, SafetyParams  # noqa: E402

OUT_DIR = os.path.join(_HERE, "after_run_replay")

# --- scenario geometry (base_link metres, +x toward the person) ------------- #
DT = 0.1                     # 10 Hz publish
V = 0.24                     # m/s approach/retreat speed
LATENCY = 0.35               # healthy img->state latency (< 0.70 s budget)
# Over-statement of the TRUE nearest body distance by the published estimates.
# Centre is dragged farther back than the nearest-point estimate: this gap is the
# fix. (Report after-run ablation: centre +0.24 m, nearest +0.19 m vs true near.)
CENTRE_OVER = 0.30
NEAREST_OVER = 0.22
TRUE_CENTRE_BEHIND = 0.30    # true body centre sits this far behind true nearest
Z = 0.9


def _frames():
    """Yield (present, true_nearest_dist) per cycle: empty -> approach 2.60->1.40
    -> dwell -> retreat 1.40->2.80 -> empty."""
    step = V * DT
    for _ in range(8):                       # person absent (CLEAR)
        yield False, None
    tn = 2.60
    while tn > 1.40 + 1e-9:                   # approach past the 2.0 m ring
        yield True, round(tn, 4)
        tn -= step
    for _ in range(25):                       # dwell inside the ring (~2.5 s)
        yield True, 1.40
    tn = 1.40
    while tn < 2.80 - 1e-9:                    # retreat back out
        tn += step
        yield True, round(tn, 4)
    for _ in range(8):                        # person gone (CLEAR)
        yield False, None


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    # Fill each record's "live" zone with what the NEW (nearest-point) pipeline
    # actually publishes, so the fixture is a self-consistent recording and
    # build_comparison's fidelity check sees the new reconstruction reproduce it.
    ev = SafetyEvaluator(SafetyParams())
    states, truth = [], []
    t = 0.0
    for present, tn in _frames():
        ev_stamp = round(t - LATENCY, 4)
        if present:
            pc = tn + CENTRE_OVER            # published centre distance
            pn = tn + NEAREST_OVER           # published nearest-point distance
            tracks = [Track(id=1, position=np.array([pc, 0.0, Z]),
                            velocity=np.zeros(3), covariance=np.eye(6), stamp=t,
                            confirmed=True, nearest_point=np.array([pn, 0.0, Z]))]
            cams = [CameraHealth(camera="wrist", present_in_graph=True,
                                 frame_age=0.05, detector_alive=True,
                                 positioned=True, fresh=True)]
            s = ev.evaluate(tracks, cams, t, any_presence=False,
                            source_stamp=ev_stamp, latency=t - ev_stamp)
            states.append({
                "kind": "state", "t": round(t, 4), "evidence_stamp": ev_stamp,
                "zone": s.zone.value, "distance_status": "known",
                "tracks": [{"id": 1, "position": [round(pc, 4), 0.0, Z],
                            "nearest_point": [round(pn, 4), 0.0, Z],
                            "confirmed": True}],
                "cameras": [{"camera": "wrist", "status": "fresh",
                             "present_in_graph": True, "frame_age": 0.05,
                             "detector_alive": True, "positioned": True,
                             "fresh": True}],
            })
            truth.append({
                "t": round(t, 4), "present": True,
                "root": [round(tn + TRUE_CENTRE_BEHIND, 4), 0.0, 0.0],
                "centre": [round(tn + TRUE_CENTRE_BEHIND, 4), 0.0, Z],
                "dist_base": round(tn + TRUE_CENTRE_BEHIND, 4),
                "dist_base_nearest": round(tn, 4),
            })
        else:
            s = ev.evaluate([], [CameraHealth(camera="wrist", present_in_graph=True,
                                              frame_age=0.05, detector_alive=True,
                                              positioned=True, fresh=True)],
                            t, any_presence=False, source_stamp=None, latency=None)
            states.append({
                "kind": "state", "t": round(t, 4), "evidence_stamp": None,
                "zone": s.zone.value, "distance_status": "none", "tracks": [],
                "cameras": [{"camera": "wrist", "status": "fresh",
                             "present_in_graph": True, "frame_age": 0.05,
                             "detector_alive": True, "positioned": True,
                             "fresh": True}],
            })
            truth.append({"t": round(t, 4), "present": False,
                          "root": [0.0, 0.0, 0.0], "centre": [0.0, 0.0, 0.0],
                          "dist_base": 99.0, "dist_base_nearest": 99.0})
        t += DT

    with open(os.path.join(OUT_DIR, "record.jsonl"), "w") as f:
        for r in states:
            f.write(json.dumps(r) + "\n")
    with open(os.path.join(OUT_DIR, "ground_truth.jsonl"), "w") as f:
        for r in truth:
            f.write(json.dumps(r) + "\n")
    print(f"wrote {len(states)} state records + {len(truth)} truth records "
          f"-> {OUT_DIR}")


if __name__ == "__main__":
    main()
