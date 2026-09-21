#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Post-run health check for a --record session, from the recorded state stream.

Reads a run's ``record.jsonl``, prints the zone distribution and the fraction of
localised states over the reaction budget, and exits NON-ZERO when the clip would
be STOP-pinned by latency faults (too many states over budget, or too few
distinct zones to show a CLEAR->SLOW->STOP progression).

The record path (run_session.sh --record) runs this AFTER the session so a
latency-pinned clip is flagged loudly; the non-zero exit is the CHECK failing, not
the session. It changes no threshold or budget — the budget is read from the same
sim config the detector used.

  python3 sim/isaac_human/record_health.py --run <RUNDIR> \
      --config sim/isaac_human/params/human_safety.sim.yaml
"""
import argparse
import json
import os
import sys

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import analysis_core as ac  # noqa: E402


def _load_states(path):
    states = []
    if not os.path.isfile(path):
        return states
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("kind") == "state":
                states.append(r)
    return states


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="RUNDIR with record.jsonl")
    ap.add_argument("--config", required=True, help="sim human_safety yaml")
    ap.add_argument("--max-frac-over-budget", type=float, default=0.20,
                    help="fail if more than this fraction of states are over the "
                         "reaction budget (default 0.20)")
    ap.add_argument("--min-distinct-zones", type=int, default=2,
                    help="fail if fewer than this many distinct zones appear "
                         "(default 2)")
    ap.add_argument("--out", default=None,
                    help="write the health JSON here (default RUNDIR/record_health.json)")
    args = ap.parse_args(argv)

    states = _load_states(os.path.join(args.run, "record.jsonl"))
    cfg = yaml.safe_load(open(args.config)) or {}
    budget = float(cfg.get("safety", {}).get("reaction_time_budget", 0.7))

    health = ac.record_health(
        states, budget,
        max_frac_over_budget=args.max_frac_over_budget,
        min_distinct_zones=args.min_distinct_zones)

    out_path = args.out or os.path.join(args.run, "record_health.json")
    try:
        with open(out_path, "w") as f:
            json.dump(health, f, indent=2)
    except OSError:
        out_path = None

    lat = health["latency"]
    print("=== RECORD HEALTH ===")
    print(f"states           : {health['n_states']}")
    print(f"zone breakdown   : {health['zone_breakdown']} "
          f"({health['distinct_zones']} distinct)")
    frac = lat["frac_over_budget"]
    frac_s = "n/a" if frac != frac else f"{frac * 100:.1f}%"
    print(f"latency budget   : {lat['budget_s']:.2f} s")
    print(f"over budget      : {lat['n_over_budget']} states ({frac_s})")
    print(f"latency median/p95/worst: {lat['summary']['median']:.3f} / "
          f"{lat['summary']['p95']:.3f} / {lat['summary']['worst']:.3f} s")
    if out_path:
        print(f"record_health.json -> {out_path}")

    if health["ok"]:
        print("VERDICT: OK -- clip shows varied zones at healthy latency.")
        return 0
    print("VERDICT: UNHEALTHY -- the recorded clip is likely STOP-pinned:")
    for r in health["reasons"]:
        print(f"  * {r}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
