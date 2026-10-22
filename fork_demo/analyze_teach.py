#!/usr/bin/env python3
"""Correlate a freedrive teach's detected dwells with the captain's relayed labels
and emit the ordered labelled waypoint list for the replay (task ur10-fork-pick-real-v1).

Reads <run>/teach_dwells.json (dwells: t_start,t_end,duration_s,fork_mean,joints_mean)
and <run>/labels.jsonl (captain labels with announced_epoch). Matches each
dwell-type label to the dwell whose [t_start,t_end] contains its announcement
(nearest-in-time fallback), assigns home(start)=first dwell, home(end)=last dwell,
and withdrawn=the last non-home dwell by geometry. Writes
<run>/labelled_waypoints.json and prints a table + a measured-joint-window check.

Pure offline; no ROS. Joint-window check uses cumotion/measured_joint_limits.yaml.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# The leg-2 intermediate dwells (between leg2_start and placed_on_shelf) map in
# this order by geometry; see the captain's relay.
LEG2_INTERMEDIATE = ["lifted_from_table", "carried", "in_front_of_slot", "inserted"]


def load_windows(limits_path):
    import yaml
    data = yaml.safe_load(open(limits_path))
    ref, win = data["reference_pose_rad"], data["window_deg"]
    return {j: (ref[j] - math.radians(win[j]), ref[j] + math.radians(win[j])) for j in ref}


def check_windows(joints, windows):
    out = []
    for i, j in enumerate(JOINT_ORDER):
        if j in windows and joints is not None:
            lo, hi = windows[j]
            if not (lo <= joints[i] <= hi):
                out.append(f"{j}={math.degrees(joints[i]):.1f}deg outside "
                           f"[{math.degrees(lo):.1f},{math.degrees(hi):.1f}]")
    return out


def dwell_for_epoch(dwells, ep):
    """dwell whose [t_start,t_end] contains ep, else nearest by midpoint."""
    for i, d in enumerate(dwells):
        if d["t_start"] <= ep <= d["t_end"]:
            return i
    return min(range(len(dwells)),
               key=lambda i: abs((dwells[i]["t_start"] + dwells[i]["t_end"]) / 2 - ep))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--limits", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "cumotion", "measured_joint_limits.yaml"))
    args = ap.parse_args(argv)

    dwells = json.load(open(os.path.join(args.run_dir, "teach_dwells.json")))["dwells"]
    labels = [json.loads(l) for l in open(os.path.join(args.run_dir, "labels.jsonl"))]
    windows = load_windows(args.limits)
    t0 = dwells[0]["t_start"]

    # announced dwell-type labels -> dwell index
    announced = {}
    for r in labels:
        # a label with an announced time is a dwell unless tagged segment/note/marker
        if r.get("announced_epoch") and r.get("type") not in ("segment", "note", "marker"):
            announced[r["label"]] = dwell_for_epoch(dwells, r["announced_epoch"])

    # ordered key waypoints
    seq = []
    seq.append(("home_start", 0))
    for lab in ("pre_grip", "holding", "lifted"):
        if lab in announced:
            seq.append((lab, announced[lab]))
    seq.append(("carry_transit", None))          # transit lifted->place
    if "place" in announced:
        seq.append(("place", announced["place"]))
    # restock pause: the longest dwell at the table side (y<0) after place
    table_dwells = [i for i, d in enumerate(dwells)
                    if d["fork_mean"][1] < 0 and (announced.get("place", 1e9) <= i)]
    if table_dwells:
        restock = max(table_dwells, key=lambda i: dwells[i]["duration_s"])
        seq.append(("restock_pause", restock))
    # leg 2 intermediate: shelf-side dwells between leg2_start and placed_on_shelf
    pos = announced.get("placed_on_shelf")
    leg2_start_ep = next((r["announced_epoch"] for r in labels
                          if r.get("label") == "leg2_start"), None)
    if pos is not None and leg2_start_ep is not None:
        inter = [i for i, d in enumerate(dwells)
                 if dwells[i]["t_start"] >= leg2_start_ep and i < pos
                 and d["fork_mean"][1] > 0]
        # sample up to 4 spread across the interval, label in order by geometry
        if inter:
            picks = [inter[round(k * (len(inter) - 1) / max(len(LEG2_INTERMEDIATE) - 1, 1))]
                     for k in range(min(len(LEG2_INTERMEDIATE), len(inter)))]
            for name, idx in zip(LEG2_INTERMEDIATE, picks):
                seq.append((name, idx))
        seq.append(("placed_on_shelf", pos))
    # withdrawn: last shelf-side dwell before home_end
    home_end = len(dwells) - 1
    shelf_before_home = [i for i in range(len(dwells))
                         if i < home_end and dwells[i]["fork_mean"][1] > 0]
    if shelf_before_home:
        seq.append(("withdrawn", max(shelf_before_home)))
    seq.append(("home_end", home_end))

    # build output + print
    print(f"TEACH ANALYSIS  {args.run_dir}")
    print(f"dwells={len(dwells)}  total_duration={dwells[-1]['t_end']-t0:.1f}s")
    print(f"{'label':<18}{'dwell':>5}{'t_rel':>8}{'dur':>7}   fork_mean (base_link)      joint-window")
    waypoints = []
    for lab, idx in seq:
        if idx is None:
            print(f"{lab:<18}{'--':>5}{'':>8}{'':>7}   (transit segment)")
            waypoints.append({"label": lab, "segment": True})
            continue
        d = dwells[idx]
        joints = d.get("joints_mean")
        viol = check_windows(joints, windows) if joints else ["no joints"]
        flag = "OK" if not viol else "OUT: " + "; ".join(viol)
        print(f"{lab:<18}{idx:>5}{d['t_start']-t0:>8.1f}{d['duration_s']:>7.1f}   "
              f"{[round(v,3) for v in d['fork_mean']]}   {flag}")
        waypoints.append({"label": lab, "dwell": idx, "t_rel": round(d["t_start"] - t0, 1),
                          "duration_s": d["duration_s"], "fork_mean": d["fork_mean"],
                          "joints_mean": joints, "window_violations": viol})
    out = {"run_dir": args.run_dir, "total_duration_s": round(dwells[-1]["t_end"] - t0, 1),
           "n_dwells": len(dwells), "waypoints": waypoints}
    json.dump(out, open(os.path.join(args.run_dir, "labelled_waypoints.json"), "w"), indent=2)
    anoms = [w for w in waypoints if w.get("window_violations")]
    print(f"\nwaypoints with joint-window violations: {len(anoms)}")
    print(f"wrote {os.path.join(args.run_dir, 'labelled_waypoints.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
