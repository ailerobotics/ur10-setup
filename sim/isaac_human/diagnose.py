#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Diagnose the published-vs-true distance bias for a sim run.

Answers the four questions the reviewer raised, from the recorded log:
  * signed error (published minus true) vs true distance, against THREE truth
    definitions (placed root, true body centroid, true nearest body point to the
    base z-axis) — is it a definition mismatch or a real localisation bias, and
    which way (toward/away from the robot)?
  * per-camera split: shelf-only vs wrist-only body-centre distance error, and
    the detector's own body-centre vs its nearest-point (does the depth median
    sit behind the person, in the background?).
  * the lead safety finding: at what TRUE distance would a distance-driven STOP
    actually fire, i.e. where does the PUBLISHED nearest distance cross the 2.0 m
    ring, and how far inside the ring is the person truly at that moment?

Pure post-hoc over the JSONL logs; writes diagnosis.json + signed_error.png.
"""
import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


def _load(path):
    out = []
    if os.path.isfile(path):
        for line in open(path):
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return out


class GT:
    """Interpolate the full ground-truth record (root / centroid / nearest)."""
    def __init__(self, recs):
        self.r = sorted(recs, key=lambda x: x["t"])
        self.ts = [x["t"] for x in self.r]

    def at(self, t):
        import bisect
        if not self.r or t < self.ts[0] or t > self.ts[-1]:
            return None
        i = bisect.bisect_left(self.ts, t)
        if i < len(self.ts) and self.ts[i] == t:
            return self.r[i]
        lo, hi = self.r[i - 1], self.r[i]
        span = hi["t"] - lo["t"]
        a = 0.0 if span <= 0 else (t - lo["t"]) / span
        out = {"t": t, "present": bool(lo.get("present") and hi.get("present"))}
        for k in ("root", "centre", "nearest_body", "bbox_min", "bbox_max"):
            if k in lo and k in hi:
                out[k] = [lo[k][j] + a * (hi[k][j] - lo[k][j])
                          for j in range(len(lo[k]))]
        for k in ("dist_base", "dist_base_centroid", "dist_base_nearest"):
            if k in lo and k in hi:
                out[k] = lo[k] + a * (hi[k] - lo[k])
        return out


def _hyp(p):
    return math.hypot(p[0], p[1])


def _med(v):
    if not v:
        return float("nan")
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _summary(v):
    s = sorted(v)
    if not s:
        return {"n": 0}
    return {"n": len(s), "median": _med(s), "mean": sum(s) / len(s),
            "min": s[0], "max": s[-1]}


def _nearest_track_pos(state):
    tid = state.get("nearest_track_id")
    for t in state.get("tracks", []):
        if t.get("id") == tid:
            return t.get("position")
    tr = state.get("tracks", [])
    return tr[0].get("position") if len(tr) == 1 else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--stop-ring", type=float, default=2.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    outdir = args.out or args.run

    gt = GT(_load(os.path.join(args.run, "ground_truth.jsonl")))
    rec = _load(os.path.join(args.run, "record.jsonl"))
    states = [r for r in rec if r.get("kind") == "state"]
    dets = [r for r in rec if r.get("kind") == "det"]

    D = {}

    # -- 1. signed error published-minus-true, three truth definitions ------- #
    per_def = {"root": [], "centroid": [], "nearest_body": []}
    for s in states:
        if s.get("distance_status") != "known":
            continue
        nd = s.get("nearest_distance_base")
        if nd is None:
            continue
        ta = gt.at(s.get("evidence_stamp") or s["t"])
        if ta is None or not ta.get("present"):
            continue
        if "dist_base" in ta:
            per_def["root"].append(nd - ta["dist_base"])
        if "dist_base_centroid" in ta:
            per_def["centroid"].append(nd - ta["dist_base_centroid"])
        if "dist_base_nearest" in ta:
            per_def["nearest_body"].append(nd - ta["dist_base_nearest"])
    D["signed_error_published_minus_true"] = {
        k: _summary(v) for k, v in per_def.items()}

    # -- 2. per-camera body-centre + centre-vs-nearest ----------------------- #
    per_cam = {}
    for d in dets:
        cam = d["camera"]
        ta = gt.at(d.get("t") or d.get("recv"))
        if ta is None or not ta.get("present"):
            continue
        for pos, npt in zip(d.get("positions", []) or [],
                            d.get("nearest_points", []) or []):
            pc = per_cam.setdefault(cam, {"centre_err": [], "nearest_err": [],
                                          "centre_minus_nearest": []})
            true_c = ta.get("dist_base_centroid", ta.get("dist_base"))
            true_n = ta.get("dist_base_nearest", true_c)
            if pos is not None:
                pc["centre_err"].append(_hyp(pos) - true_c)
                if npt is not None:
                    pc["centre_minus_nearest"].append(_hyp(pos) - _hyp(npt))
            if npt is not None:
                pc["nearest_err"].append(_hyp(npt) - true_n)
    D["per_camera"] = {
        cam: {k: _summary(v) for k, v in d.items()}
        for cam, d in per_cam.items()}

    # -- 3. lead finding: where a distance-driven STOP would fire ------------- #
    ss = sorted([s for s in states
                 if s.get("nearest_distance_base") is not None], key=lambda x: x["t"])
    lead = None
    for a, b in zip(ss, ss[1:]):
        da, db = a["nearest_distance_base"], b["nearest_distance_base"]
        if da >= args.stop_ring > db:      # published crosses the ring inward
            ta = gt.at(b["t"])
            true_here = ta.get("dist_base") if ta else None
            true_near = ta.get("dist_base_nearest") if ta else None
            lead = {
                "published_crosses_ring_at_t": b["t"],
                "stop_ring_m": args.stop_ring,
                "true_root_dist_at_that_moment": true_here,
                "true_nearest_body_dist_at_that_moment": true_near,
                "person_inside_ring_by_m_root":
                    (args.stop_ring - true_here) if true_here is not None else None,
                "person_inside_ring_by_m_nearest":
                    (args.stop_ring - true_near) if true_near is not None else None,
            }
            break
    D["distance_driven_stop_would_fire"] = lead

    # -- bbox extent sanity: how wide is the person (T-pose vs natural)? ------ #
    widths = []
    for r in gt.r:
        if "bbox_min" in r and "bbox_max" in r:
            widths.append((r["bbox_max"][0] - r["bbox_min"][0],
                           r["bbox_max"][1] - r["bbox_min"][1]))
    if widths:
        D["person_bbox_extent_m"] = {
            "x_width_median": _med([w[0] for w in widths]),
            "y_depth_median": _med([w[1] for w in widths]),
            "note": "x_width ~1.6 m indicates an arms-out T-pose; the wide box "
                    "lets background floor into the depth region.",
        }

    with open(os.path.join(outdir, "diagnosis.json"), "w") as f:
        json.dump(D, f, indent=2)
    print(json.dumps(D, indent=2))

    _plot_signed_error(outdir, gt, states)
    return 0


def _plot_signed_error(outdir, gt, states):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:  # noqa: BLE001
        return
    xs_root, ys_root, ys_near = [], [], []
    for s in states:
        if s.get("distance_status") != "known":
            continue
        nd = s.get("nearest_distance_base")
        if nd is None:
            continue
        ta = gt.at(s.get("evidence_stamp") or s["t"])
        if ta is None or not ta.get("present") or "dist_base" not in ta:
            continue
        xs_root.append(ta["dist_base"])
        ys_root.append(nd - ta["dist_base"])
        if "dist_base_nearest" in ta:
            ys_near.append(nd - ta["dist_base_nearest"])
    if not xs_root:
        return
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.scatter(xs_root, ys_root, s=10, color="#1c7ed6", alpha=0.6,
               label="published − true(root)")
    if len(ys_near) == len(xs_root):
        ax.scatter(xs_root, ys_near, s=10, color="#d1495b", alpha=0.5,
                   label="published − true(nearest body)")
    ax.axhline(0, color="k", lw=1)
    ax.set_xlabel("true distance to base (m)")
    ax.set_ylabel("signed distance error (m), + = reported FARTHER (unsafe)")
    ax.set_title("Published nearest-distance error vs true distance")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "signed_error.png"), dpi=110)
    plt.close(fig)
    print(f"signed_error.png -> {outdir}")


if __name__ == "__main__":
    sys.exit(main())
