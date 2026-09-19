#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Analyse the GRASP end-to-end run (scenario `dynamic_pick_grasp`) and assert the
near/grasp/retreat pick path behaves (report §3). The load-bearing behaviour — the
one this scenario exists to exercise end to end — is the finding that the per-phase
re-plan policy holds on the FRESH-leg path, not only on preemption:

  **while the tool point is in GRASP the target keeps MOVING, yet the controller
  issues NO plan toward it (GRASP is FROZEN) — a leg completing in grasp never
  re-targets a moved object.**

It then confirms the grasp completes -> begin_retreat -> a retreat leg to the FIXED
pose, and re-checks the sole-authority invariant (every goal under a fresh RUN).

NEAR-band traversal: on real trajectory timing (URSim) the arm crosses the near
band gradually, so NEAR is REQUIRED there. On the MOCK arm a leg completes
near-instantly (it teleports the state to the setpoint), so the tool point jumps
across the near band into GRASP without dwelling in it — a documented rig
limitation (exactly like preemption; see assert_dynamic_pick.py). NEAR is therefore
INFORMATIONAL on `mock` and REQUIRED on `ursim`.

Usage:
  assert_grasp.py --pick pick.jsonl [--retreat 0.45,-0.25,0.50] [--mode mock|ursim]
"""
import argparse
import json
import sys


def load(path):
    rows = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
    except FileNotFoundError:
        pass
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick", required=True)
    ap.add_argument("--retreat", default="0.45,-0.25,0.50")
    ap.add_argument("--mode", default="mock")
    a = ap.parse_args()
    retreat = [round(float(v), 3) for v in a.retreat.split(",")]

    pick = load(a.pick)
    ticks = [r for r in pick if r.get("kind") == "tick"]
    goals = [r for r in pick if r.get("kind") == "goal_sent"]
    plan_ticks = [t for t in ticks if t.get("intent") == "PLAN"]
    phases = sorted({t.get("phase") for t in ticks if t.get("phase")})

    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    check("controller produced ticks", len(ticks) > 0, f"{len(ticks)} ticks")
    check("controller sent goals (arm followed the target)", len(goals) > 0, f"{len(goals)} goals")

    # ---- sole authority still holds ------------------------------------- #
    bad_goals = [g for g in goals if g.get("gate_action") != "RUN"]
    check("SOLE AUTHORITY: every goal forwarded ONLY under a fresh gate RUN",
          len(bad_goals) == 0,
          "all RUN" if not bad_goals else f"{len(bad_goals)} not RUN")

    # ---- the tool point reached the grasp band -------------------------- #
    check("phase FAR exercised", "FAR" in phases, f"phases={phases}")
    check("phase GRASP exercised (grasp band entered)", "GRASP" in phases, f"phases={phases}")
    grasp_ticks = [t for t in ticks if t.get("phase") == "GRASP"]

    # ---- THE finding-B proof: GRASP is FROZEN on the fresh-leg path too -- #
    grasp_plans = [t for t in plan_ticks if t.get("phase") == "GRASP"]
    check("GRASP froze re-planning (no PLAN issued while in GRASP)", len(grasp_plans) == 0,
          f"{len(grasp_plans)} PLAN ticks in GRASP")
    grasp_moved = [t for t in grasp_ticks if t.get("committed_moved")]
    check("a target that MOVES while in GRASP produces NO plan (frozen fresh-leg path)",
          len(grasp_moved) > 0 and all(t.get("intent") != "PLAN" for t in grasp_moved),
          f"{len(grasp_moved)} GRASP ticks with a moved target, all held")

    # ---- grasp completion -> begin_retreat -> fixed retreat leg --------- #
    check("grasp completion signalled begin_retreat",
          any(r.get("kind") == "grasp_complete_begin_retreat" for r in pick))
    check("phase RETREAT exercised", "RETREAT" in phases, f"phases={phases}")
    retreat_goals = [g for g in goals if g.get("goal_xyz") == retreat]
    check("retreat leg planned to the FIXED pre-planned pose", len(retreat_goals) > 0,
          f"retreat={retreat}")

    # ---- NEAR: required only where trajectory timing is real ------------ #
    if a.mode == "ursim":
        check("phase NEAR exercised (near band traversed under real timing)",
              "NEAR" in phases, f"phases={phases}")
    else:
        checks.append(("phase NEAR (informational on mock: teleport jumps the band)",
                       True, f"NEAR {'seen' if 'NEAR' in phases else 'skipped'}"))

    print(f"\n=== dynamic-pick GRASP end-to-end assertions ({a.mode}) ===")
    print(f"phases exercised: {phases}")
    print(f"goals sent: {len(goals)}  (retreat-pose goals: {len(retreat_goals)})")
    print()
    failed = 0
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
        if not ok:
            failed += 1
    print(f"\n{len(checks) - failed}/{len(checks)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
