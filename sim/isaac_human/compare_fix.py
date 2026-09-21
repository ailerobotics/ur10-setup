#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Old-vs-new before/after for the nearest-point safety fix, from ONE recorded run.

A single live Isaac run records, per published state, BOTH the nearest-point
distance (the new zone driver) and the body-centre distance (what the zone used
before the fix), plus each track's position + nearest point and the per-camera
health block. That is everything the deterministic safety state machine needs, so
we replay the EXACT live inputs through ``SafetyEvaluator`` twice — once using
each track's nearest point (NEW) and once ignoring it (OLD, == pre-fix centre-only
behaviour) — and compare both against the simulator's ground truth.

No ROS, no Isaac, no GPU: pure replay of the recorded JSONL. The NEW reconstruction
must reproduce the live zone stream (a fidelity check), which proves the OLD
reconstruction differs from NEW only by the distance rule.

Usage:
    python3 compare_fix.py --run <run-dir> [--json out.json]
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_HS = os.path.join(os.path.dirname(os.path.dirname(_HERE)), "cams", "human_safety")
for _p in (_HERE, _HS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import analysis_core as ac  # noqa: E402
from cell_layout import camera as layout_camera  # noqa: E402
from human_safety.config import load_config  # noqa: E402
from human_safety.messages import CameraHealth, Track  # noqa: E402
from human_safety.safety import SafetyEvaluator  # noqa: E402


def _health_from_rec(c: dict) -> CameraHealth:
    fa = c.get("frame_age")
    return CameraHealth(
        camera=c.get("camera", "?"),
        present_in_graph=bool(c.get("present_in_graph", c.get("status") != "absent")),
        frame_age=(float("inf") if fa is None else float(fa)),
        detector_alive=bool(c.get("detector_alive", True)),
        positioned=bool(c.get("positioned", False)),
        fresh=bool(c.get("fresh", c.get("status") == "fresh")),
        note=c.get("note", ""),
    )


def _tracks_from_rec(rec: dict, use_nearest: bool):
    out = []
    for t in rec.get("tracks", []):
        pos = t.get("position")
        if pos is None:
            continue
        npnt = t.get("nearest_point")
        nearest = (np.asarray(npnt, dtype=float)
                   if (use_nearest and npnt is not None) else None)
        out.append(Track(
            id=int(t.get("id", 0)), position=np.asarray(pos, dtype=float),
            velocity=np.zeros(3), covariance=np.eye(6),
            stamp=float(rec.get("t") or 0.0),
            confirmed=bool(t.get("confirmed", True)), nearest_point=nearest))
    return out


def reconstruct(state_records, params, use_nearest: bool,
                ignore_latency_fault: bool = False):
    """Replay recorded live states through the safety machine. Returns analysis
    'state' dicts (t, evidence_stamp, zone, distance_status, nearest_distance*,
    nearest_track_id, tracks).

    ``ignore_latency_fault``: pass ``latency=None`` to the evaluator so the
    img->state latency fault (latency > reaction_time_budget -> STOP) is disabled
    and the state is driven by distance + hysteresis + dwell + the per-camera
    health floor ONLY. Everything else is identical. This isolates whether a STOP
    is genuinely distance-driven or was latched by the latency fault (the confound
    the first harness report carried), so a "no missed STOP" claim can be shown to
    be about distance, not about a fail-safe STOP that happened to already be on."""
    ev = SafetyEvaluator(params)
    out = []
    for rec in state_records:
        now = float(rec.get("t") or rec.get("recv") or 0.0)
        tracks = _tracks_from_rec(rec, use_nearest)
        health = [_health_from_rec(c) for c in rec.get("cameras", [])]
        ev_stamp = rec.get("evidence_stamp")
        latency = None if ignore_latency_fault else (
            (now - ev_stamp) if ev_stamp is not None else None)
        any_presence = rec.get("distance_status") == "unknown"
        s = ev.evaluate(tracks, health, now, any_presence=any_presence,
                        source_stamp=ev_stamp, latency=latency)
        known = s.distance_status.value == "known"
        out.append({
            "t": now,
            "evidence_stamp": ev_stamp,
            "zone": s.zone.value,
            "distance_status": s.distance_status.value,
            "nearest_distance": (None if not known or s.nearest_distance == float("inf")
                                 else float(s.nearest_distance)),
            "nearest_distance_base": (None if not known or s.nearest_distance_base == float("inf")
                                      else float(s.nearest_distance_base)),
            "nearest_track_id": s.nearest_track_id,
            "tracks": [{"id": t.id, "position": [float(x) for x in t.position]}
                       for t in s.tracks],
        })
    return out


def true_nearest_ring_crossings(truth_records, ring):
    """Inward crossings (dist -> below ``ring``) of the TRUE nearest body point to
    base. List of crossing times. Uses ``dist_base_nearest`` when present."""
    recs = [r for r in truth_records if r.get("present")]
    recs.sort(key=lambda r: r["t"])
    out = []
    for a, b in zip(recs, recs[1:]):
        da = a.get("dist_base_nearest", a["dist_base"])
        db = b.get("dist_base_nearest", b["dist_base"])
        if da >= ring > db:
            f = (da - ring) / (da - db) if da != db else 0.0
            out.append(a["t"] + f * (b["t"] - a["t"]))
    return out


def stop_engagement(states, crossing_t, truth: ac.Truth):
    """(dt, true_nearest_at_stop) for the first STOP at or after ``crossing_t``:
    how long after the person's true nearest body point crossed the ring the state
    became STOP, and the true nearest distance at that moment. dt None if never."""
    for s in sorted(states, key=lambda r: r["t"]):
        if s["t"] >= crossing_t and s["zone"] == "STOP":
            tr = truth.at(s["t"])
            near = (tr.get("dist_base_nearest", tr["dist_base"])
                    if tr else None)
            return s["t"] - crossing_t, near
    return None, None


def zone_changes(states):
    return len(ac.zone_transitions(states))


def stop_time_s(states):
    """Total wall-clock time the zone was STOP (sum of gaps between consecutive
    STOP samples, so it is robust to a variable publish rate)."""
    ss = sorted(states, key=lambda r: r["t"])
    total = 0.0
    for a, b in zip(ss, ss[1:]):
        if a["zone"] == "STOP":
            total += b["t"] - a["t"]
    return total


def signed_dist_errors(states, truth: ac.Truth, ref="dist_base_nearest"):
    """Signed (published - true) nearest-distance-to-base error list, aligned on
    the evidence stamp. Positive = published reads FARTHER than truth (unsafe)."""
    out = []
    for s in states:
        if s.get("distance_status") != "known":
            continue
        t_align = s.get("evidence_stamp") or s["t"]
        tr = truth.at(t_align)
        if tr is None or not tr["present"]:
            continue
        nd = s.get("nearest_distance_base")
        if nd is None:
            continue
        true_d = tr.get(ref, tr["dist_base"])
        out.append(nd - true_d)
    return out


def _summ_signed(vals):
    if not vals:
        return {"n": 0, "median": float("nan"), "p95": float("nan"),
                "worst": float("nan")}
    a = sorted(vals)
    return {
        "n": len(a),
        "median": ac._percentile(a, 50),
        "p95": ac._percentile([abs(v) for v in a], 95),
        "worst": max(vals),   # most positive = worst unsafe overstatement
        "min": min(vals),
    }


def build_comparison(run_dir, params, reach, stop, budget):
    rec_path = os.path.join(run_dir, "record.jsonl")
    gt_path = os.path.join(run_dir, "ground_truth.jsonl")
    state_records = []
    with open(rec_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r.get("kind") == "state":
                state_records.append(r)
    truth_records = []
    with open(gt_path) as f:
        for line in f:
            line = line.strip()
            if line:
                truth_records.append(json.loads(line))
    truth = ac.Truth(truth_records)

    def in_fov(role, centre):
        try:
            return layout_camera(role).in_fov(centre)
        except Exception:  # noqa: BLE001
            return True

    # The PROTECTIVE ring (2.0 m) — every judgement measures misses against this,
    # never against the zone's STOP TRIGGER ring (which now includes stop_margin).
    ring = reach + stop
    crossings = true_nearest_ring_crossings(truth_records, ring)

    # Two ring configurations, differing ONLY in stop_margin: the OLD ring (margin
    # 0, STOP triggers at the 2.0 m protective ring) and the NEW ring (the config's
    # margin, STOP triggers ~0.25 m earlier at 2.25 m). Both use the nearest-point
    # rule; both are judged against the SAME 2.0 m protective ring below.
    params_old_ring = dataclasses.replace(params, stop_margin=0.0)
    params_new_ring = params   # carries the config's stop_margin (0.25 m)

    # The nearest-point before/after (centre vs nearest rule) is isolated at the
    # OLD ring (margin 0) — the ring that fix was validated against — so this
    # comparison and the ring comparison are independent axes.
    old = reconstruct(state_records, params_old_ring, use_nearest=False)
    new = reconstruct(state_records, params_old_ring, use_nearest=True)
    # DISTANCE-ONLY: latency fault disabled, so STOP is driven by distance +
    # hysteresis + dwell only. This is what tells us a "no missed STOP" is really
    # about distance, not a fail-safe latency STOP that was already latched.
    old_do = reconstruct(state_records, params_old_ring, use_nearest=False,
                         ignore_latency_fault=True)
    new_do = reconstruct(state_records, params_old_ring, use_nearest=True,
                         ignore_latency_fault=True)
    # RING before/after (this task): OLD ring vs NEW ring, both nearest rule,
    # latency fault disabled (the honest distance-driven picture).
    old_ring = new_do   # nearest rule, margin 0, distance-only == the old ring
    new_ring = reconstruct(state_records, params_new_ring, use_nearest=True,
                           ignore_latency_fault=True)

    # Fidelity: reconstructed NEW zones vs the live published zones.
    live_zones = [r.get("zone") for r in state_records]
    new_zones = [s["zone"] for s in new]
    fidelity = sum(1 for a, b in zip(live_zones, new_zones) if a == b)
    fidelity_frac = fidelity / len(new_zones) if new_zones else float("nan")

    def side(states):
        rep = ac.build_report(states, [], truth_records, in_fov, reach, stop, budget)
        eng = [stop_engagement(states, c, truth) for c in crossings]
        return {
            "zone_changes": zone_changes(states),
            "stop_time_s": stop_time_s(states),
            "missed_stops": rep.missed_stops,
            "n_missed": len(rep.missed_stops),
            "worst_missed_inside_m": (max(ring - m["min_true_dist"]
                                          for m in rep.missed_stops)
                                      if rep.missed_stops else 0.0),
            "worst_missed_dur_s": (max(m["duration"] for m in rep.missed_stops)
                                   if rep.missed_stops else 0.0),
            "stop_engagement": [
                {"dt_after_crossing_s": dt, "true_nearest_at_stop_m": near}
                for dt, near in eng],
            "signed_err_vs_nearest": _summ_signed(
                signed_dist_errors(states, truth, "dist_base_nearest")),
            "signed_err_vs_root": _summ_signed(
                signed_dist_errors(states, truth, "dist_base")),
            "verdict": rep.verdict,
            "verdict_ok": rep.verdict_ok,
        }

    return {
        "run_dir": run_dir,
        "n_states": len(state_records),
        "fidelity_new_vs_live_frac": fidelity_frac,
        "stop_boundary_m": ring,
        "true_nearest_ring_crossings_t": crossings,
        # latency fault ON (as published live) — masked by fail-safe STOP
        "old": side(old),
        "new": side(new),
        # latency fault OFF — distance-driven only (the honest safety picture)
        "old_distance_only": side(old_do),
        "new_distance_only": side(new_do),
        # RING before/after: OLD ring (2.0 m trigger) vs NEW ring (2.25 m trigger),
        # both nearest rule + distance-only, both judged vs the 2.0 m PROTECTIVE ring.
        "old_ring": side(old_ring),
        "new_ring": side(new_ring),
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="run dir (record.jsonl + ground_truth.jsonl)")
    ap.add_argument("--config", default=os.path.join(_HERE, "params", "human_safety.sim.yaml"))
    ap.add_argument("--json", default=None, help="write the comparison JSON here")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    p = cfg.safety
    comp = build_comparison(args.run, p, p.reach_radius, p.stop_radius,
                            p.reaction_time_budget)

    def fmt(s):
        se = s["signed_err_vs_nearest"]
        eng = s["stop_engagement"]
        eng_s = "; ".join(
            (f"STOP {e['dt_after_crossing_s']:.2f}s after crossing at true-nearest "
             f"{e['true_nearest_at_stop_m']:.2f}m")
            if e["dt_after_crossing_s"] is not None else "never STOP"
            for e in eng) or "(no true ring crossing)"
        return (f"  missed_stops={s['n_missed']} "
                f"(worst {s['worst_missed_dur_s']:.2f}s, "
                f"{s['worst_missed_inside_m']:.2f}m inside)  "
                f"zone_changes={s['zone_changes']}  stop_time={s['stop_time_s']:.1f}s\n"
                f"  {eng_s}\n"
                f"  signed err vs TRUE NEAREST body point: median={se['median']:+.2f} "
                f"worst={se.get('worst', float('nan')):+.2f} m (n={se['n']})")

    print(f"run: {comp['run_dir']}  states={comp['n_states']}  "
          f"NEW-reconstruction matches live zones: "
          f"{comp['fidelity_new_vs_live_frac']*100:.1f}%")
    print("=== latency fault ON (as published live; STOP may be a masked fail-safe) ===")
    print("OLD (centre-based, pre-fix):");  print(fmt(comp["old"]))
    print("NEW (nearest-point, this fix):"); print(fmt(comp["new"]))
    print("=== DISTANCE-ONLY (latency fault disabled; distance+hysteresis+dwell only) ===")
    print("OLD (centre-based, pre-fix):");  print(fmt(comp["old_distance_only"]))
    print("NEW (nearest-point, this fix):"); print(fmt(comp["new_distance_only"]))
    print(f"=== STOP-RING MARGIN (nearest rule, distance-only; judged vs "
          f"{comp['stop_boundary_m']:.2f} m PROTECTIVE ring) ===")
    print("OLD ring (stop_margin 0, trigger 2.00 m):"); print(fmt(comp["old_ring"]))
    print("NEW ring (stop_margin on, trigger 2.25 m):"); print(fmt(comp["new_ring"]))

    if args.json:
        with open(args.json, "w") as f:
            json.dump(comp, f, indent=2)
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
