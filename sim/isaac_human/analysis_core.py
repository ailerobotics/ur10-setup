# SPDX-License-Identifier: Apache-2.0
"""Pure analysis of a sim run: ground truth vs the published safety state.

No ROS, no Isaac, no torch, no matplotlib — just arithmetic over plain-dict
records — so the whole judgement is unit-tested offline on synthetic logs
(perfect tracking, a constant offset, a missed STOP, a late transition).

Record shapes (all times are wall-clock seconds, all positions base_link metres):

  truth : {"t", "present":bool, "root":[x,y,z], "centre":[x,y,z], "dist_base"}
  state : {"t", "zone":"CLEAR|SLOW|STOP", "distance_status",
           "nearest_distance", "nearest_distance_base", "nearest_track_id",
           "tracks":[{"id","position":[x,y,z]}]}
  det   : {"t", "camera", "n_persons":int, "positioned":bool}

The STOP boundary is ``reach_radius + stop_radius`` metres from the base origin
(2.0 m by default) — the same envelope the safety layer uses. The missed-STOP
budget is the reaction-time budget (0.7 s): a person truly inside the STOP zone
while the state is not STOP for LONGER than that is a real, reportable miss.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# --------------------------------------------------------------------------- #
def _percentile(vals: List[float], p: float) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (p / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return s[int(k)]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def summarize(vals: List[float]) -> Dict[str, float]:
    """median / p95 / worst(max) / n for an error list."""
    return {
        "n": len(vals),
        "median": _percentile(vals, 50) if vals else float("nan"),
        "p95": _percentile(vals, 95) if vals else float("nan"),
        "worst": max(vals) if vals else float("nan"),
    }


# --------------------------------------------------------------------------- #
class Truth:
    """Time-indexed ground truth with linear interpolation between samples."""

    def __init__(self, records: List[dict]):
        self.recs = sorted(records, key=lambda r: r["t"])
        self.ts = [r["t"] for r in self.recs]

    def __len__(self):
        return len(self.recs)

    def at(self, t: float) -> Optional[dict]:
        """Interpolated {present, root, centre, dist_base} at time t, or None if
        outside the recorded window. ``present`` is True only if BOTH bracketing
        samples are present (a conservative edge at appearance/disappearance)."""
        if not self.recs or t < self.ts[0] or t > self.ts[-1]:
            return None
        i = bisect.bisect_left(self.ts, t)
        if i < len(self.ts) and self.ts[i] == t:
            r = self.recs[i]
            out = {"present": r["present"], "root": r["root"],
                   "centre": r["centre"], "dist_base": r["dist_base"]}
            if "dist_base_nearest" in r:
                out["dist_base_nearest"] = r["dist_base_nearest"]
            return out
        lo = self.recs[i - 1]
        hi = self.recs[i]
        span = hi["t"] - lo["t"]
        a = 0.0 if span <= 0 else (t - lo["t"]) / span
        present = bool(lo["present"] and hi["present"])

        def lerp_vec(u, v):
            return [u[k] + a * (v[k] - u[k]) for k in range(len(u))]
        out = {
            "present": present,
            "root": lerp_vec(lo["root"], hi["root"]),
            "centre": lerp_vec(lo["centre"], hi["centre"]),
            "dist_base": lo["dist_base"] + a * (hi["dist_base"] - lo["dist_base"]),
        }
        # Optional stricter truth: nearest true body point to the base z-axis.
        if "dist_base_nearest" in lo and "dist_base_nearest" in hi:
            out["dist_base_nearest"] = (
                lo["dist_base_nearest"]
                + a * (hi["dist_base_nearest"] - lo["dist_base_nearest"]))
        return out


def _dist2d(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dist3d(a, b) -> float:
    return math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3)))


# --------------------------------------------------------------------------- #
def detection_rate_per_camera(dets: List[dict], truth: Truth, in_fov_fn,
                              ) -> Dict[str, dict]:
    """For each camera, over detection frames where the person is truly in that
    camera's FOV, the fraction that reported >=1 person.

    ``in_fov_fn(role, centre_xyz) -> bool`` decides visibility (the harness wires
    it to ``cell_layout.camera(role).in_fov``). Frames where the person is absent
    or out of that camera's view are not counted as opportunities."""
    out: Dict[str, dict] = {}
    for d in dets:
        role = d["camera"]
        tr = truth.at(d["t"])
        if tr is None or not tr["present"]:
            continue
        if not in_fov_fn(role, tr["centre"]):
            continue
        o = out.setdefault(role, {"opportunities": 0, "hits": 0})
        o["opportunities"] += 1
        if d["n_persons"] >= 1:
            o["hits"] += 1
    for role, o in out.items():
        o["rate"] = (o["hits"] / o["opportunities"]
                     if o["opportunities"] else float("nan"))
    return out


def detection_scores(dets: List[dict], truth: Truth, in_fov_fn) -> Dict[str, float]:
    """Summary of the detector's confidence over in-view frames that reported a
    person: the best score per such frame, then median / min / n. Same visibility
    gate as ``detection_rate_per_camera``, so the score phrase in the report is
    measured, not assumed. Frames with no reported person or no score contribute
    nothing (``n`` counts only scored hits)."""
    best: List[float] = []
    for d in dets:
        tr = truth.at(d["t"])
        if tr is None or not tr["present"]:
            continue
        if not in_fov_fn(d["camera"], tr["centre"]):
            continue
        scs = [s for s in (d.get("scores") or []) if s is not None]
        if scs:
            best.append(max(scs))
    return {
        "n": len(best),
        "median": _percentile(best, 50) if best else float("nan"),
        "min": min(best) if best else float("nan"),
    }


def _nearest_track_pos(state: dict) -> Optional[list]:
    tid = state.get("nearest_track_id")
    for t in state.get("tracks", []):
        if t.get("id") == tid:
            return t.get("position")
    # Fall back to the single track if id bookkeeping is absent (synthetic logs).
    tracks = state.get("tracks", [])
    if len(tracks) == 1:
        return tracks[0].get("position")
    return None


def position_errors(states: List[dict], truth: Truth) -> Dict[str, dict]:
    """3-D + horizontal position error of the nearest track vs the true body
    centre, and nearest-distance-to-base error vs the true distance. Only states
    with a KNOWN distance and a matching present ground-truth sample count."""
    pos3d, pos2d, dist_err = [], [], []
    for s in states:
        if s.get("distance_status") != "known":
            continue
        # Align to the image that PRODUCED this state (evidence_stamp), not the
        # publish time, so the pipeline latency does not masquerade as position
        # error (a moving person would otherwise look mislocated by v*latency).
        t_align = s.get("evidence_stamp") or s["t"]
        tr = truth.at(t_align)
        if tr is None or not tr["present"]:
            continue
        p = _nearest_track_pos(s)
        if p is not None:
            pos3d.append(_dist3d(p, tr["centre"]))
            pos2d.append(_dist2d(p, tr["centre"]))
        nd = s.get("nearest_distance_base")
        if nd is not None:
            dist_err.append(abs(nd - tr["dist_base"]))
    return {
        "position_3d_error_m": summarize(pos3d),
        "position_horizontal_error_m": summarize(pos2d),
        "nearest_distance_base_error_m": summarize(dist_err),
    }


# --------------------------------------------------------------------------- #
def latency_stats(states: List[dict], budget_s: float) -> Dict:
    """Image->state latency (state publish time minus the evidence_stamp that sized
    it) over every localised state, plus the fraction over the reaction budget.
    This is the quantity the detector's fault-5 rule checks; on a shared GPU it is
    what pushes the state to STOP (fail-safe) even while the person is far."""
    vals = [s["t"] - s["evidence_stamp"] for s in states
            if s.get("evidence_stamp") is not None and s.get("t") is not None]
    over = sum(1 for v in vals if v > budget_s)
    return {
        "summary": summarize(vals),
        "budget_s": budget_s,
        "n_over_budget": over,
        "frac_over_budget": (over / len(vals)) if vals else float("nan"),
    }


def zone_breakdown(states: List[dict]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for s in states:
        out[s["zone"]] = out.get(s["zone"], 0) + 1
    return out


def zone_transitions(states: List[dict]) -> List[dict]:
    """The ordered zone changes: [{t, from, to}]."""
    out = []
    prev = None
    for s in sorted(states, key=lambda r: r["t"]):
        z = s["zone"]
        if prev is not None and z != prev:
            out.append({"t": s["t"], "from": prev, "to": z})
        prev = z
    return out


def boundary_crossings(truth: Truth, boundary_m: float) -> List[dict]:
    """True inward/outward crossings of a base-distance ring: [{t, dir}] where
    dir is 'inward' (entering, dist -> below boundary) or 'outward'."""
    out = []
    recs = [r for r in truth.recs if r["present"]]
    for a, b in zip(recs, recs[1:]):
        da, db = a["dist_base"], b["dist_base"]
        if da >= boundary_m > db:
            frac = (da - boundary_m) / (da - db) if da != db else 0.0
            out.append({"t": a["t"] + frac * (b["t"] - a["t"]), "dir": "inward"})
        elif da <= boundary_m < db:
            frac = (boundary_m - da) / (db - da) if da != db else 0.0
            out.append({"t": a["t"] + frac * (b["t"] - a["t"]), "dir": "outward"})
    return out


def stop_boundary_m(reach_radius: float, stop_radius: float) -> float:
    return reach_radius + stop_radius


def stop_transition_latencies(states: List[dict], truth: Truth,
                              reach_radius: float, stop_radius: float
                              ) -> List[dict]:
    """Pair each true STOP-ring crossing with the state's response.

    For an INWARD crossing: the first time the zone becomes STOP at or after the
    crossing (entry latency). For an OUTWARD crossing: the first time the zone
    leaves STOP at or after the crossing (exit latency — expected to be >= the
    2.0 s dwell by design, reported, not judged as a fault). ``true_dist_at_change``
    is the true base distance when the state actually changed."""
    boundary = stop_boundary_m(reach_radius, stop_radius)
    crossings = boundary_crossings(truth, boundary)
    ss = sorted(states, key=lambda r: r["t"])
    out = []
    for c in crossings:
        want_stop = c["dir"] == "inward"
        change_t = None
        for s in ss:
            if s["t"] < c["t"]:
                continue
            is_stop = s["zone"] == "STOP"
            if want_stop and is_stop:
                change_t = s["t"]
                break
            if not want_stop and not is_stop:
                change_t = s["t"]
                break
        rec = {"dir": c["dir"], "boundary_m": boundary, "crossing_t": c["t"]}
        if change_t is None:
            rec["latency_s"] = None
            rec["true_dist_at_change"] = None
        else:
            rec["latency_s"] = change_t - c["t"]
            tr = truth.at(change_t)
            rec["true_dist_at_change"] = tr["dist_base"] if tr else None
        out.append(rec)
    return out


def missed_stops(states: List[dict], truth: Truth, reach_radius: float,
                 stop_radius: float, budget_s: float) -> List[dict]:
    """Intervals where the person is TRULY inside the STOP ring but the state is
    not STOP, lasting longer than ``budget_s``. These are the safety-critical
    failures — reported first and loudly. Each: {start, end, duration, min_true_dist}.

    Evaluated on the state stream (each state sample carries the true distance at
    its own time). A run of consecutive not-STOP samples while truly inside the
    ring is one interval; it counts only if its duration exceeds the budget."""
    boundary = stop_boundary_m(reach_radius, stop_radius)
    ss = sorted(states, key=lambda r: r["t"])
    intervals = []
    cur = None
    for s in ss:
        tr = truth.at(s["t"])
        if tr is None or not tr["present"]:
            true_close = None
        else:
            # "inside the ring" means ANY true body part inside -> use the nearest
            # true body point when the ground truth provides it, else the root.
            true_close = tr.get("dist_base_nearest", tr["dist_base"])
        inside = true_close is not None and true_close < boundary
        violating = inside and s["zone"] != "STOP"
        if violating:
            d = true_close
            if cur is None:
                cur = {"start": s["t"], "end": s["t"], "min_true_dist": d}
            else:
                cur["end"] = s["t"]
                cur["min_true_dist"] = min(cur["min_true_dist"], d)
        else:
            if cur is not None:
                cur["duration"] = cur["end"] - cur["start"]
                intervals.append(cur)
                cur = None
    if cur is not None:
        cur["duration"] = cur["end"] - cur["start"]
        intervals.append(cur)
    return [iv for iv in intervals if iv["duration"] > budget_s]


# --------------------------------------------------------------------------- #
@dataclass
class RunReport:
    detection_rates: Dict[str, dict] = field(default_factory=dict)
    detection_score: dict = field(default_factory=dict)
    errors: Dict[str, dict] = field(default_factory=dict)
    transitions: List[dict] = field(default_factory=list)
    stop_latencies: List[dict] = field(default_factory=list)
    missed_stops: List[dict] = field(default_factory=list)
    latency: dict = field(default_factory=dict)
    zone_breakdown: dict = field(default_factory=dict)
    stop_boundary_m: float = 0.0
    verdict: str = ""
    verdict_ok: bool = False


def build_report(states: List[dict], dets: List[dict], truth_records: List[dict],
                 in_fov_fn, reach_radius: float, stop_radius: float,
                 reaction_time_budget: float) -> RunReport:
    truth = Truth(truth_records)
    r = RunReport()
    r.stop_boundary_m = stop_boundary_m(reach_radius, stop_radius)
    r.detection_rates = detection_rate_per_camera(dets, truth, in_fov_fn)
    r.detection_score = detection_scores(dets, truth, in_fov_fn)
    r.errors = position_errors(states, truth)
    r.transitions = zone_transitions(states)
    r.stop_latencies = stop_transition_latencies(
        states, truth, reach_radius, stop_radius)
    r.missed_stops = missed_stops(
        states, truth, reach_radius, stop_radius, reaction_time_budget)
    r.latency = latency_stats(states, reaction_time_budget)
    r.zone_breakdown = zone_breakdown(states)
    # Verdict: a genuine miss (person inside the STOP ring, state not STOP, beyond
    # the reaction budget) is a FAIL. Otherwise PASS the safety-critical bar; the
    # accuracy numbers are reported separately and read on their own.
    if r.missed_stops:
        worst = min(iv["min_true_dist"] for iv in r.missed_stops)
        longest = max(iv["duration"] for iv in r.missed_stops)
        inside_by = r.stop_boundary_m - worst
        r.verdict = (f"FAIL: {len(r.missed_stops)} missed-STOP interval(s); the "
                     f"person's nearest true body point reached {worst:.2f} m from "
                     f"base ({inside_by:.2f} m inside the {r.stop_boundary_m:.2f} m "
                     f"STOP ring) while the state was not STOP, for up to "
                     f"{longest:.2f} s (> the reaction budget).")
        r.verdict_ok = False
    else:
        any_stop = any(s["zone"] == "STOP" for s in states)
        if not any_stop:
            r.verdict = ("INCONCLUSIVE: no STOP was ever published — the run did "
                         "not exercise the STOP zone (check detection).")
            r.verdict_ok = False
        else:
            r.verdict = ("PASS (safety-critical): no missed STOP — whenever the "
                         "person was truly inside the STOP ring beyond the "
                         "reaction budget, the state was STOP.")
            r.verdict_ok = True
    return r
