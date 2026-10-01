# SPDX-License-Identifier: Apache-2.0
"""Generate multicam_approach.jsonl — a deterministic, frame-level multi-camera
sequence used to compare the every-frame BASELINE against the EFFICIENT
frame-selection path (``human_safety.frameproc_eval``). Re-run to regenerate.

    python3 make_multicam_approach.py

The cell's three cameras (shelf, table, wrist) each produce one frame per 0.1 s
tick. shelf + table are (pretend-calibrated) depth cameras that localise the
person; wrist is presence-only (seen, no trustworthy pose), exactly as in the
real cell. Each record carries, per camera, whether a new frame arrived, a cheap
change/motion score (what ``frameselect.change_score`` would return on the raw
frame), and the ground-truth detections a full detector would yield that frame.
``persons`` is the ground-truth count of people in the monitored volume.

The script is the ground truth for BOTH paths, so the comparison is fair: the
only difference between baseline and efficient is which frames actually get run
through the (here, simulated) detector.
"""
import json
import pathlib

OUT = pathlib.Path(__file__).with_name("multicam_approach.jsonl")

DT = 0.1
LOW = 0.001      # quiet-scene change score (well below motion_threshold 0.02)
STILL = 0.004    # a present-but-still person (below threshold; near-ring covers it)
MOVE = 0.045     # a person walking (above threshold -> motion trigger)
ENTER = 0.18     # a person entering/leaving view (big block of pixels changes)


def depth_det(x):
    return {"position": [round(x, 3), 0.0, 0.9],
            "nearest_point": [round(x, 3), 0.0, 0.9],
            "method": "depth", "score": 0.9, "sigma": 0.1, "n_depth_valid": 420}


def presence_det():
    return {"method": "presence", "present": True, "score": 0.8}


def rec(t, persons, shelf=None, table=None, wrist=None):
    def cam(score, dets):
        return {"have_new_frame": True, "change_score": score,
                "dets": dets}
    return {"t": round(t, 3), "persons": persons,
            "cams": {"shelf": cam(*shelf), "table": cam(*table),
                     "wrist": cam(*wrist)}}


def main():
    records = []
    t = 0.0

    # Phase 0 — empty cell, 4 s. Quiet: the efficient path should drop to the
    # t_max floor here; both paths CLEAR.
    for _ in range(40):
        records.append(rec(t, 0, (LOW, []), (LOW, []), (LOW, [])))
        t += DT

    # Phase 1 — a person enters and approaches from 4.0 m to 1.1 m of base over
    # 3 s. The first frame they appear is a big change (ENTER); walking frames
    # are MOVE (motion trigger keeps every camera hot). shelf+table localise;
    # wrist sees presence only.
    xs = [4.0 - (4.0 - 1.1) * i / 29 for i in range(30)]
    for i, x in enumerate(xs):
        score = ENTER if i == 0 else MOVE
        records.append(rec(t, 1, (score, [depth_det(x)]),
                           (score, [depth_det(x)]),
                           (score, [presence_det()])))
        t += DT

    # Phase 2 — the person dwells inside the stop ring (x = 1.1 m) for 1.5 s,
    # standing still (STILL < threshold). Motion alone would let the camera idle;
    # the near-robot-track priority must keep it at full rate anyway.
    for _ in range(15):
        records.append(rec(t, 1, (STILL, [depth_det(1.1)]),
                           (STILL, [depth_det(1.1)]),
                           (STILL, [presence_det()])))
        t += DT

    # Phase 3 — the person retreats from 1.1 m back out to 3.6 m over 2.5 s.
    xs = [1.1 + (3.6 - 1.1) * i / 24 for i in range(25)]
    for x in xs:
        records.append(rec(t, 1, (MOVE, [depth_det(x)]),
                           (MOVE, [depth_det(x)]),
                           (MOVE, [presence_det()])))
        t += DT

    # Phase 4 — the person leaves the cell (no detections). The exit frame is a
    # big change; then the cell is quiet again. Tracks coast, delete, and both
    # paths return to CLEAR after the dwell.
    records.append(rec(t, 0, (ENTER, []), (ENTER, []), (ENTER, [])))
    t += DT
    for _ in range(49):
        records.append(rec(t, 0, (LOW, []), (LOW, []), (LOW, [])))
        t += DT

    with open(OUT, "w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")
    print(f"wrote {len(records)} ticks -> {OUT}")


if __name__ == "__main__":
    main()
