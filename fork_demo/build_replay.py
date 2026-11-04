#!/usr/bin/env python3
"""Build the STEP-mode replay of a freedrive teach (task ur10-fork-pick-real-v1).

From <run>/teach_log.jsonl + <run>/labelled_waypoints.json, build one SEGMENT per
captain-gated step. Each segment replays the RECORDED joint path between two
labelled dwells (subsampled + endpoint-pinned) -- NOT a joint-straight move --
because the captain's taught path is collision-free by construction. Segments are
written as execute_plan.py-compatible plan yamls (joint_path/goal_joints/
duration_s/label) so the PROVEN executor dispatches them (5% cap, tracking +
program guards); step_replay.py runs them one at a time with the force_abort
watchdog.

Also derives <run>/replay_joint_limits.yaml -- the joint-window guard for the
replay, taken from the teach's OWN min/max per joint (+ margin). The stale
cumotion/measured_joint_limits.yaml does NOT cover this motion (the teach goes
well outside it: shoulder_lift to -235 deg, shoulder_pan to -65 deg); the arm
demonstrably reached these poses in freedrive, so the teach envelope is the
correct, safe guard for replaying within the demonstrated set.

Offline only; no ROS. Writes replay_plan.json (index), per-segment plan_*.yaml,
replay_joint_limits.yaml, and replay_path.png.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
VEL_LIMIT = {"shoulder_pan_joint": 2.09, "shoulder_lift_joint": 2.09, "elbow_joint": 3.14,
             "wrist_1_joint": 3.14, "wrist_2_joint": 3.14, "wrist_3_joint": 3.14}
SPEED = 0.05           # execute_plan.py cap; <= 10% free and <= 5% contact both satisfied
PEAK_TARGET = 0.045    # keep reconstructed peak under execute_plan's 5% * 1.05 guard

# Ordered segments: (name, start_label, end_label, contact, note).
# contact legs (insert/lift/lower/pick/extract) pause-before + extra care.
SEGMENTS = [
    ("to_pre_grip",        "home_start",        "pre_grip",          False, "free: approach in front of the tray"),
    ("insert_forks",       "pre_grip",          "holding",           True,  "CONTACT: forks into the tray"),
    ("lift",               "holding",           "lifted",            True,  "CONTACT: lift the tray off the posts"),
    ("carry_to_table",     "lifted",            "place",             False, "free: carry across to the table (via above-table descent)"),
    ("lower_on_table",     "__carry_via__",     "restock_pause",     True,  "CONTACT: lower/place the tray on the table (then HOLD for restock)"),
    ("lift_from_table",    "restock_pause",     "lifted_from_table", True,  "CONTACT: pick the filled tray off the table (after restock)"),
    ("carry_to_shelf",     "lifted_from_table", "carried",           False, "free: carry back toward the shelf"),
    ("approach_slot",      "carried",           "in_front_of_slot",  False, "free: in front of the shelf slot"),
    ("insert_slot",        "in_front_of_slot",  "inserted",          True,  "CONTACT: forks into the shelf slot"),
    ("lower_release",      "inserted",          "placed_on_shelf",   True,  "CONTACT: lower/release onto the shelf"),
    ("withdraw",           "placed_on_shelf",   "withdrawn",         True,  "CONTACT: withdraw the forks from the shelf"),
    ("to_home",            "withdrawn",          "home_end",         False, "free: return to home"),
]


def load_log_joints(path):
    """Return [(t, [6 joints]), ...] from teach_log.jsonl (joints present)."""
    out = []
    with open(path) as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            j = r.get("joints")
            if j and len(j) == 6:
                out.append((r["t"], j))
    return out


def subsample_window(samples, t0, t1, max_pts=30):
    seg = [s for s in samples if t0 <= s[0] <= t1]
    if len(seg) <= max_pts:
        return seg
    step = (len(seg) - 1) / (max_pts - 1)
    return [seg[round(k * step)] for k in range(max_pts)]


def seg_duration(path):
    """Duration s.t. execute_plan's reconstructed peak velocity ~ PEAK_TARGET."""
    if len(path) < 2:
        return 2.0
    # peak per-joint delta / vmax over the path, then dt so (delta/dt)/vmax = PEAK_TARGET
    worst = 0.0
    for a, b in zip(path[:-1], path[1:]):
        for i, j in enumerate(JOINT_ORDER):
            worst = max(worst, abs(b[i] - a[i]) / VEL_LIMIT[j])
    dt_min = worst / PEAK_TARGET if worst else 0.1
    return max(round(dt_min * (len(path) - 1), 1), 2.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--margin-deg", type=float, default=5.0,
                    help="margin added to the teach's own joint min/max for the guard")
    ap.add_argument("--max-pts", type=int, default=30)
    args = ap.parse_args(argv)

    lab = json.load(open(os.path.join(args.run_dir, "labelled_waypoints.json")))
    wps = {w["label"]: w for w in lab["waypoints"] if not w.get("segment")}
    samples = load_log_joints(os.path.join(args.run_dir, "teach_log.jsonl"))
    if not samples:
        print("no joint samples in teach_log", file=sys.stderr)
        return 1

    # teach joint envelope (min/max per joint over the whole log) + margin
    jmin = [min(s[1][i] for s in samples) for i in range(6)]
    jmax = [max(s[1][i] for s in samples) for i in range(6)]
    m = math.radians(args.margin_deg)
    ref = {JOINT_ORDER[i]: (jmin[i] + jmax[i]) / 2 for i in range(6)}
    win = {JOINT_ORDER[i]: math.degrees((jmax[i] - jmin[i]) / 2) + args.margin_deg
           for i in range(6)}
    import yaml
    with open(os.path.join(args.run_dir, "replay_joint_limits.yaml"), "w") as f:
        yaml.safe_dump({"reference_pose_rad": {k: float(v) for k, v in ref.items()},
                        "window_deg": {k: float(v) for k, v in win.items()},
                        "source": "teach-002 freedrive envelope + margin "
                                  f"{args.margin_deg} deg (NOT the stale measured sweep)"}, f)

    # carry_via: the recorded sample above the table (closest in XY to place, high z)
    # -- computed from the lifted->place transit. We approximate with the sample in
    # that window at max z that is already near the place XY.
    def dwell_window(label):
        w = wps[label]
        d_ts = w["t_rel"]  # relative; we need absolute -> recompute from samples t0
        return w
    # absolute times: labelled_waypoints stored t_rel; recover t0 from first sample
    t0abs = samples[0][0]

    def abs_window(label):
        w = wps[label]
        start = t0abs + w["t_rel"]
        return start, start + w["duration_s"]

    # carry_via pose: between lifted(end) and place(start), the highest-z sample.
    lifted_end = abs_window("lifted")[1]
    place_start = abs_window("place")[0]
    transit = [s for s in samples if lifted_end <= s[0] <= place_start]
    carry_via_joints = None
    carry_via_t = None
    if transit:
        # need fork z; recompute would need tf. Use the sample at ~60% through the
        # transit as the "carried above table" via (post-lift, pre-descent).
        carry_via = transit[int(len(transit) * 0.6)]
        carry_via_joints = carry_via[1]
        carry_via_t = carry_via[0]

    segments_out = []
    plan_paths = []
    for k, (name, s_lab, e_lab, contact, note) in enumerate(SEGMENTS):
        # resolve start/end joints + time window
        if s_lab == "__carry_via__":
            if carry_via_joints is None:
                continue
            s_t, s_j = carry_via_t, carry_via_joints
        else:
            sw = abs_window(s_lab); s_t = sw[1]; s_j = wps[s_lab]["joints_mean"]
        e_t0, e_t1 = abs_window(e_lab)
        e_j = wps[e_lab]["joints_mean"]
        path = [p[1] for p in subsample_window(samples, s_t, e_t0, args.max_pts)]
        # pin exact endpoints
        if not path or path[0] != s_j:
            path = [s_j] + path
        if path[-1] != e_j:
            path = path + [e_j]
        dur = seg_duration(path)
        label = f"replay_{k:02d}_{name}"
        plan = {"label": label, "joint_path": [[float(v) for v in p] for p in path],
                "goal_joints": [float(v) for v in e_j], "duration_s": float(dur)}
        pp = os.path.join(args.run_dir, f"plan_{k:02d}_{name}.yaml")
        yaml.safe_dump(plan, open(pp, "w"))
        plan_paths.append(pp)
        segments_out.append({
            "index": k, "name": name, "start": s_lab, "end": e_lab,
            "contact": contact, "pause_before": True,
            "speed_frac": 0.05 if contact else 0.10, "n_points": len(path),
            "duration_s": dur, "plan_yaml": os.path.basename(pp), "note": note,
        })

    out = {"run_dir": args.run_dir, "n_segments": len(segments_out),
           "joint_limits": "replay_joint_limits.yaml",
           "exec": "step_replay.py --segment <index> (execute_plan.py --exact + force_abort)",
           "segments": segments_out}
    json.dump(out, open(os.path.join(args.run_dir, "replay_plan.json"), "w"), indent=2)

    print(f"REPLAY PLAN for {args.run_dir}  ({len(segments_out)} segments)")
    print(f"{'#':>2} {'segment':<17}{'speed':>6}  {'pts':>3} {'dur':>5}  pause  note")
    for s in segments_out:
        print(f"{s['index']:>2} {s['name']:<17}{int(s['speed_frac']*100):>5}% "
              f"{s['n_points']:>4} {s['duration_s']:>5.1f}  "
              f"{'CONTACT' if s['contact'] else '  free ':>7}  {s['note']}")
    print(f"\njoint guard: replay_joint_limits.yaml (teach envelope + {args.margin_deg} deg)")
    print(f"wrote replay_plan.json + {len(plan_paths)} plan_*.yaml")
    _plot(args.run_dir, lab, segments_out)
    return 0


def _plot(run_dir, lab, segments):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        return
    wps = {w["label"]: w for w in lab["waypoints"] if not w.get("segment")}
    order = ["home_start", "pre_grip", "holding", "lifted", "place",
             "lifted_from_table", "carried", "in_front_of_slot", "inserted",
             "placed_on_shelf", "withdrawn", "home_end"]
    pts = [wps[o]["fork_mean"] for o in order if o in wps]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    fig = plt.figure(figsize=(9, 7)); ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, "-o", color="tab:green")
    for o in order:
        if o in wps:
            ax.text(*wps[o]["fork_mean"], f" {o}", fontsize=7)
    ax.scatter([0], [0], [0], c="k", marker="s", s=40, label="base_link")
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title("teach-002 replay waypoints (fork-tip, base_link)"); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(run_dir, "replay_path.png"), dpi=110)
    print("wrote replay_path.png")


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
