#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline analysis of a sim run: ground truth vs published safety state.

Reads the ground-truth JSONL (Isaac) and the recorder JSONL (the real graph's
state + detections), runs the pure ``analysis_core`` judgement, and writes
``metrics.json`` plus two plots (top-down true-vs-tracked path with zone rings,
and distance-vs-time with the zone bands). No ROS, no Isaac — pure post-hoc.

  python3 sim/isaac_human/analyze.py --run <RUNDIR> \
      --config sim/isaac_human/params/human_safety.sim.yaml
"""
import argparse
import dataclasses
import json
import os
import sys

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import analysis_core as ac  # noqa: E402
import cell_layout as layout  # noqa: E402


def _load_jsonl(path):
    out = []
    if not os.path.isfile(path):
        return out
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return out


def _in_fov(role, centre):
    try:
        return layout.camera(role).in_fov(centre, margin=0.0)
    except KeyError:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="RUNDIR with the logs")
    ap.add_argument("--config", required=True, help="sim human_safety yaml")
    ap.add_argument("--out", default=None, help="output dir (default RUNDIR)")
    args = ap.parse_args()

    outdir = args.out or args.run
    os.makedirs(outdir, exist_ok=True)

    gt = _load_jsonl(os.path.join(args.run, "ground_truth.jsonl"))
    rec = _load_jsonl(os.path.join(args.run, "record.jsonl"))
    states = [r for r in rec if r.get("kind") == "state"]
    dets = [r for r in rec if r.get("kind") == "det"]

    cfg = yaml.safe_load(open(args.config)) or {}
    safety = cfg.get("safety", {})
    reach = float(safety.get("reach_radius", 1.30))
    stop = float(safety.get("stop_radius", 0.70))
    slow = float(safety.get("slow_radius", 2.00))
    budget = float(safety.get("reaction_time_budget", 0.7))

    report = ac.build_report(states, dets, gt, _in_fov, reach, stop, budget)

    metrics = dataclasses.asdict(report)
    metrics["counts"] = {
        "gt_samples": len(gt), "state_msgs": len(states),
        "detection_msgs": len(dets),
        "detection_by_camera": {
            role: sum(1 for d in dets if d.get("camera") == role)
            for role in {d.get("camera") for d in dets}},
    }
    metrics["zones"] = {"stop_ring_base_m": reach + stop,
                        "slow_ring_base_m": reach + slow,
                        "reach_radius_m": reach}
    with open(os.path.join(outdir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print("=== VERDICT ===")
    print(report.verdict)
    print(f"metrics.json -> {os.path.join(outdir, 'metrics.json')}")

    _plots(outdir, gt, states, reach, stop, slow)
    return 0


def _plots(outdir, gt, states, reach, stop, slow):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    stop_ring = reach + stop
    slow_ring = reach + slow

    # -- top-down: true path vs tracked path + zone rings --------------- #
    fig, axp = plt.subplots(figsize=(6, 6))
    th = np.linspace(0, 2 * np.pi, 200)
    for r, c, lab in [(stop_ring, "#d1495b", f"STOP {stop_ring:.1f} m"),
                      (slow_ring, "#edae49", f"SLOW {slow_ring:.1f} m"),
                      (reach, "#8d99ae", f"reach {reach:.1f} m")]:
        axp.plot(r * np.cos(th), r * np.sin(th), color=c, lw=1.5, label=lab)
    if gt:
        gx = [r["root"][0] for r in gt]
        gy = [r["root"][1] for r in gt]
        axp.plot(gx, gy, "-", color="#2b8a3e", lw=2, label="true path")
    tx, ty = [], []
    for s in states:
        if s.get("distance_status") != "known":
            continue
        p = ac._nearest_track_pos(s)
        if p is not None:
            tx.append(p[0]); ty.append(p[1])
    if tx:
        axp.scatter(tx, ty, s=10, color="#1c7ed6", alpha=0.6, label="tracked")
    axp.plot(0, 0, "ks", ms=10, label="robot base")
    axp.set_aspect("equal")
    axp.set_xlabel("x (m, base_link)"); axp.set_ylabel("y (m, base_link)")
    axp.set_title("Top-down: true vs tracked person, with zone rings")
    axp.legend(loc="upper right", fontsize=8)
    axp.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "topdown.png"), dpi=110)
    plt.close(fig)

    # -- distance vs time with zone bands ------------------------------- #
    if not gt or not states:
        return
    t0 = gt[0]["t"]
    fig, axd = plt.subplots(figsize=(9, 4.5))
    axd.plot([r["t"] - t0 for r in gt], [r["dist_base"] for r in gt],
             "-", color="#2b8a3e", lw=2, label="true dist to base")
    st = sorted(states, key=lambda r: r["t"])
    pub_t = [s["t"] - t0 for s in st if s.get("nearest_distance_base") is not None]
    pub_d = [s["nearest_distance_base"] for s in st
             if s.get("nearest_distance_base") is not None]
    if pub_t:
        axd.plot(pub_t, pub_d, ".", color="#1c7ed6", ms=5,
                 label="published nearest-to-base")
    axd.axhline(stop_ring, color="#d1495b", ls="--", lw=1, label=f"STOP {stop_ring:.1f} m")
    axd.axhline(slow_ring, color="#edae49", ls="--", lw=1, label=f"SLOW {slow_ring:.1f} m")
    # zone band as background colour
    zc = {"CLEAR": "#e6f4ea", "SLOW": "#fff4e0", "STOP": "#fde2e4"}
    for a, b in zip(st, st[1:]):
        axd.axvspan(a["t"] - t0, b["t"] - t0, color=zc.get(a["zone"], "white"),
                    alpha=0.7, lw=0)
    axd.set_xlabel("time (s)"); axd.set_ylabel("distance to base (m)")
    axd.set_title("Distance to base over time (background = published zone)")
    axd.legend(loc="upper right", fontsize=8)
    axd.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "distance_time.png"), dpi=110)
    plt.close(fig)
    print(f"plots -> {outdir}/topdown.png, {outdir}/distance_time.png")


if __name__ == "__main__":
    sys.exit(main())
