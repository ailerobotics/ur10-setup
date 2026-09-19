#!/usr/bin/env python3
"""Analyse the dashboard pause/resume investigation (PAUSE_RESUME_FINDING.md) and
assert the finding still holds: on this cell's HEADLESS CB3 driver, dashboard
`pause`/`play` do NOT deliver a freeze-and-continue, so the gate's STOP correctly
stays the goal-cancel backstop (stop-then-replan).

It is also a REGRESSION GUARD. If a future driver / PolyScope / config change ever
makes headless `pause` actually execute (a true freeze becomes available), this
asserter FAILS with RE-EVALUATE so the owner revisits the stop-path decision rather
than silently keeping stop-then-replan when a same-trajectory continue exists.

Usage:
  python3 assert_pause_investigation.py --file <pause_investigation.txt>
  python3 assert_pause_investigation.py --result <result.json>   # for tests

Exit code 0 = finding holds. Non-zero = the experiment was invalid, OR pause now
executes (RE-EVALUATE), OR the negative behaviour was not reproduced.
"""
import argparse
import json
import sys


def load_result(args) -> dict:
    if args.result:
        with open(args.result) as f:
            return json.load(f)
    with open(args.file) as f:
        for line in f:
            line = line.strip()
            if line.startswith("RESULT "):
                return json.loads(line[len("RESULT "):])
    raise SystemExit(f"no RESULT line found in {args.file}")


def evaluate(r: dict):
    """Return (checks, reevaluate). checks = list of (name, ok, detail)."""
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    if "error" in r:
        check("probe ran to completion", False, r.get("error", "probe error"))
        return checks, False

    # --- experiment validity: it must actually have tested a moving arm -------- #
    check("dashboard pause/play services existed",
          bool(r.get("dashboard_before_motion")),
          str(r.get("dashboard_before_motion")))
    check("trajectory goal was accepted", r.get("goal_accepted") is True)
    check("arm was clearly moving before the pause call",
          r.get("clearly_moving_before_pause") is True,
          f"{r.get('speed_before_pause')} rad/s")

    pause_ok = bool(r.get("pause_call", {}).get("ok"))
    play_ok = bool(r.get("play_call", {}).get("ok"))

    # --- the finding: headless pause is rejected and delivers no freeze -------- #
    reevaluate = pause_ok or play_ok
    check("dashboard PAUSE was rejected (headless: no loaded program)",
          not pause_ok, r.get("pause_call", {}).get("msg", ""))
    check("dashboard PLAY was rejected (headless: no loaded program)",
          not play_ok, r.get("play_call", {}).get("msg", ""))
    check("arm did NOT freeze on the pause call",
          r.get("froze_after_pause") is False,
          f"pan drift during 'pause' = {r.get('pan_drift_during_pause')} rad")
    check("pause did NOT abort the goal or drop the connection",
          r.get("goal_aborted_by_pause") is False
          and r.get("joint_states_flowing_during_pause") is True)
    check("trajectory ran to completion unaffected (goal succeeded)",
          r.get("final_goal_succeeded") is True,
          f"result_code={r.get('final_goal_result_code')}")
    check("no true freeze-and-continue via the dashboard",
          r.get("true_freeze_and_continue") is False)

    return checks, reevaluate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", help="pause_investigation.txt (with a RESULT line)")
    ap.add_argument("--result", help="a bare RESULT json file (for tests)")
    a = ap.parse_args()
    if not a.file and not a.result:
        ap.error("give --file or --result")

    r = load_result(a)
    checks, reevaluate = evaluate(r)

    print("\n=== dashboard pause/resume investigation (headless CB3) ===")
    failed = 0
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
        if not ok:
            failed += 1

    if reevaluate:
        print("\n*** RE-EVALUATE: dashboard pause/play EXECUTED on this run. A true "
              "freeze-and-continue may now be available on this controller — revisit "
              "the stop-path decision in PAUSE_RESUME_FINDING.md before shipping. ***")
        return 2

    if failed:
        print(f"\n{len(checks) - failed}/{len(checks)} checks passed — finding NOT reproduced")
        return 1
    print(f"\n{len(checks)}/{len(checks)} checks passed — finding holds: no dashboard "
          "freeze-and-continue on headless CB3; STOP stays the goal-cancel backstop "
          "(stop-then-replan).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
