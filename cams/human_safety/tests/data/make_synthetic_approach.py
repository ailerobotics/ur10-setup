# SPDX-License-Identifier: Apache-2.0
"""Generate synthetic_approach.jsonl — a deterministic detection log of one
person walking in toward the robot and back out, seen by the (pretend-
calibrated) shelf and table cameras. Used as the seed replay regression until a
real walk-through recording is captured. Re-run to regenerate.

    python3 make_synthetic_approach.py
"""
import json
import pathlib

OUT = pathlib.Path(__file__).with_name("synthetic_approach.jsonl")


def health(cam):
    return {"camera": cam, "present_in_graph": True, "frame_age": 0.05,
            "detector_alive": True, "positioned": True, "fresh": True}


def det(cam, stamp, x, sigma=0.1):
    return {"camera": cam, "stamp": stamp, "score": 0.9, "method": "depth",
            "present": True, "position": [x, 0.0, 0.9],
            "nearest_point": [x, 0.0, 0.9], "sigma": sigma, "n_depth_valid": 400}


def main():
    records = []
    t = 0.0
    dt = 0.1
    # base_x path while the person is IN VIEW: far (4.0) -> approach to 1.1 ->
    # dwell inside the stop ring -> retreat back out to 3.4. Then the person
    # LEAVES the cell (no detections) so tracks coast, delete, and the state
    # returns to CLEAR (report §3: CLEAR = no person in the monitored volume).
    xs = ([4.0 - 0.1 * i for i in range(30)]      # 4.0 -> 1.1 approach
          + [1.1] * 10                             # dwell inside stop
          + [1.1 + 0.1 * i for i in range(24)])    # retreat 1.1 -> 3.4
    for x in xs:
        records.append({
            "now": round(t, 3),
            "detections": [det("shelf", round(t, 3), x),
                           det("table", round(t, 3), x)],
            "health": [health("shelf"), health("table")],
            "any_presence": False,
        })
        t += dt
    # person has left: empty detections, cameras still healthy.
    for _ in range(45):
        records.append({
            "now": round(t, 3),
            "detections": [],
            "health": [health("shelf"), health("table")],
            "any_presence": False,
        })
        t += dt
    with open(OUT, "w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")
    print(f"wrote {len(records)} cycles -> {OUT}")


if __name__ == "__main__":
    main()
