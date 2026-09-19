#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""In-container driver for the ESDF appear/decay proof.

Assumes nvblox (dynamic layer) and synth_depth_pub.py are already running on this
ROS domain (the shell wrapper starts them). This node:

  * polls nvblox's get_esdf_and_gradient over a small AABB around where each
    timeline obstacle is,
  * reads /fm_dyn/ground_truth to know when each obstacle is actually present,
  * logs one aligned JSON record per poll,
  * then ASSERTS: each obstacle is (near-)absent before it appears, occupied while
    present, and decays back down after it leaves — and records the measured
    time-to-appear and time-to-clear.

Exit code 0 = all obstacles proved appear+decay; non-zero = a property failed.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import rclpy
from nvblox_msgs.srv import EsdfAndGradients
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from esdf_probe import query_occupied  # noqa: E402

# Probe AABBs (base_link), disjoint in x so box and person are measured apart.
# The z-floor is ABOVE the ground plane (ground_z = -0.02) on purpose: the ground
# is permanently occupied and would swamp the obstacle's own voxels, so we measure
# only the obstacle BODY that sits above the floor. The box is 0..0.30 m tall, the
# person column 0..1.70 m.
PROBES = {
    "box":    dict(aabb_min=[-0.25, 0.45, 0.06], aabb_size=[0.50, 0.50, 0.26]),
    "person": dict(aabb_min=[0.30, 0.40, 0.10], aabb_size=[0.70, 0.60, 1.55]),
}


class Driver:
    def __init__(self, node, occ_thresh):
        self.node = node
        self.occ_thresh = occ_thresh
        self.client = node.create_client(
            EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")
        self.gt = {}
        node.create_subscription(String, "/fm_dyn/ground_truth", self._on_gt, 10)

    def _on_gt(self, msg):
        try:
            self.gt = json.loads(msg.data)
        except Exception:
            pass

    def wait_service(self, timeout=60.0):
        return self.client.wait_for_service(timeout_sec=timeout)

    def poll(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        rec = {"t_wall": round(time.time(), 3),
               "gt_t": self.gt.get("t_s"), "obstacles": {}}
        for name, box in PROBES.items():
            q = query_occupied(self.node, self.client, box["aabb_min"],
                               box["aabb_size"], self.occ_thresh, "base_link", 5.0)
            gt_present = bool(self.gt.get("obstacles", {}).get(name, {}).get("present"))
            rec["obstacles"][name] = {
                "gt_present": gt_present,
                "occ": None if q is None else q.occupied_voxels,
                "min_dist": None if q is None else round(q.min_distance_m, 3),
                "latency_ms": None if q is None else round(q.call_latency_s * 1e3, 1),
            }
        return rec


def _assert_appear_decay(records, name, min_occ=4):
    """Return (ok, detail) for one obstacle: absent-before, occupied-while, decays
    after. Times are relative to the ground-truth clock in the records."""
    series = [(r["gt_t"], r["obstacles"][name]) for r in records
              if r["gt_t"] is not None and r["obstacles"][name]["occ"] is not None]
    if not series:
        return False, "no samples"
    present = [(t, o["occ"]) for t, o in series if o["gt_present"]]
    before = [(t, o["occ"]) for t, o in series
              if not o["gt_present"] and present and t < present[0][0]]
    after = [(t, o["occ"]) for t, o in series
             if not o["gt_present"] and present and t > present[-1][0]]
    if not present:
        return False, "obstacle never present in ground truth"
    peak = max(o for _, o in present)
    # appeared: occupancy crossed the min while present
    appeared = peak >= min_occ
    t_appear = next((t for t, o in present if o >= min_occ), None)
    appear_lat = None if (t_appear is None or not present) else round(t_appear - present[0][0], 2)
    # absent-before: was low before it appeared
    before_ok = (not before) or max(o for _, o in before) < min_occ
    # decayed: after leaving, occupancy dropped below half the peak
    half = max(min_occ, peak * 0.5)
    decayed = bool(after) and (after[-1][1] < half)
    t_clear = next((t for t, o in after if o < half), None)
    clear_lat = None if (t_clear is None or not present) else round(t_clear - present[-1][0], 2)
    ok = appeared and before_ok and decayed
    return ok, {"peak_occ": peak, "appeared": appeared, "before_ok": before_ok,
                "decayed": decayed, "appear_lat_s": appear_lat,
                "clear_lat_s": clear_lat}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=42.0)
    ap.add_argument("--poll-hz", type=float, default=2.0)
    ap.add_argument("--occ-thresh", type=float, default=0.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--require", nargs="*", default=["box", "person"],
                    help="obstacles that must prove appear+decay")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("fm_dyn_esdf_proof_driver")
    drv = Driver(node, args.occ_thresh)
    if not drv.wait_service(60.0):
        print("FAIL: esdf service never came up", flush=True)
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    records = []
    period = 1.0 / args.poll_hz
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rec = drv.poll()
        records.append(rec)
        print(json.dumps(rec), flush=True)
        time.sleep(period)

    summary = {"n_records": len(records), "results": {}, "ok": True}
    for name in args.require:
        ok, detail = _assert_appear_decay(records, name)
        summary["results"][name] = {"ok": ok, "detail": detail}
        summary["ok"] = summary["ok"] and ok
    with open(args.out, "w") as f:
        json.dump({"summary": summary, "records": records}, f, indent=2)
    print("SUMMARY " + json.dumps(summary), flush=True)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if summary["ok"] else 1)


if __name__ == "__main__":
    main()
