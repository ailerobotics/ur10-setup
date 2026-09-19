#!/usr/bin/env python3
"""Analyse the dashboard pause/resume investigation (PAUSE_RESUME_FINDING.md,
PENDANT_PAUSE_FINDING.md) and assert the finding for the given mode.

Two modes (the pendant mode was added by the pendant-pause task):

  * ``--mode headless`` (default): assert that on this cell's HEADLESS CB3 driver,
    dashboard `pause`/`play` do NOT deliver a freeze-and-continue, so the gate's
    STOP correctly stays the goal-cancel backstop (stop-then-replan). Also a
    REGRESSION GUARD: if a future driver/PolyScope/config change ever makes headless
    `pause` execute (a true freeze becomes available), it FAILS with RE-EVALUATE so
    the owner revisits the stop-path decision rather than silently keeping
    stop-then-replan when a same-trajectory continue exists.

  * ``--mode pendant``: with an External Control program LOADED and PLAYING on the
    pendant, assert the MEASURED positive finding by ``action``:
      - pause_play      -> a TRUE freeze-and-continue: pause executes, the arm freezes
                           on its path, the applied scaling falls to ~0, the SAME
                           FollowJointTrajectory goal survives and, on play, runs to
                           its ORIGINAL goal with NO replan (final code 0);
      - pause_only      -> pause executes, the arm freezes and STAYS stopped for the
                           whole hold (durability); it never runs away;
      - foreign_play    -> play does resume motion (the conflict the gate must
                           re-pause): the run ends MOVING — this documents the hazard;
      - stop_instead    -> dashboard `stop` is terminal: the goal does NOT succeed
                           (aborts / connection drops) — a resume needs a fresh plan;
      - pause_no_program-> with nothing running, pause is rejected and the arm keeps
                           moving — pause is not a stop when no program plays.
    A pendant regression guard fires (RE-EVALUATE) if pause_play STOPS delivering the
    freeze-and-continue it delivered when the strategy was commissioned.

Usage:
  python3 assert_pause_investigation.py --file <pause_investigation.txt> [--mode M]
  python3 assert_pause_investigation.py --result <result.json> [--mode M]  # for tests

Exit code 0 = finding holds for the mode. Non-zero = the experiment was invalid, OR
the expected behaviour was not reproduced, OR (headless) pause now executes / (pendant)
pause_play stopped freezing-and-continuing (RE-EVALUATE).
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


STOP_SCALING = 0.05          # applied scaling at/below this counts as an effected stop


def evaluate_pendant(r: dict):
    """Return (checks, reevaluate) for MODE=pendant. `action` in the RESULT selects
    the expected behaviour. reevaluate=True means the pause_play freeze-and-continue
    that the strategy relies on has REGRESSED and the owner must revisit."""
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    if "error" in r:
        check("probe ran to completion", False, r.get("error", "probe error"))
        return checks, False

    action = r.get("action", "pause_play")

    # --- experiment validity: a loaded, playing program and a moving arm --------- #
    dbm = r.get("dashboard_before_motion") or {}
    check("External Control program was LOADED before motion",
          bool(dbm.get("loaded_program")), str(dbm.get("loaded_program")))
    check("scaled_joint_trajectory_controller was ACTIVE",
          r.get("jtc_state_before") == "active" or r.get("jtc_state_before_pause") == "active",
          f"{r.get('jtc_state_before')} / {r.get('jtc_state_before_pause')}")
    check("trajectory goal was accepted", r.get("goal_accepted") is True)
    check("arm was clearly moving before the pause call",
          r.get("clearly_moving_before_pause") is True,
          f"{r.get('speed_before_pause')} rad/s")

    pause_ok = bool(r.get("pause_call", {}).get("ok"))
    froze = r.get("froze_after_pause") is True
    scaling_zeroed = (r.get("scaling_during_pause_min") is not None
                      and r.get("scaling_during_pause_min") <= STOP_SCALING)
    reevaluate = False

    if action == "pause_play":
        check("dashboard PAUSE executed (loaded program)", pause_ok,
              r.get("pause_call", {}).get("msg", ""))
        check("arm FROZE on its path within a few seconds", froze,
              f"t_freeze={r.get('time_to_freeze_s')}s, "
              f"drift={r.get('pan_drift_during_pause')} rad")
        check("applied speed scaling fell to ~0 while paused (a CONFIRMABLE stop)",
              scaling_zeroed, f"min scaling during pause = {r.get('scaling_during_pause_min')}")
        check("the SAME FJT goal SURVIVED the pause (not aborted)",
              r.get("goal_aborted_by_pause") is False,
              f"code_at_pause={r.get('goal_code_at_pause')}")
        check("connection stayed up (/joint_states flowing while paused)",
              r.get("joint_states_flowing_during_pause") is True)
        check("dashboard PLAY executed", bool(r.get("play_call", {}).get("ok")),
              r.get("play_call", {}).get("msg", ""))
        check("motion RESUMED on play (arm moving again / pan advanced past pause point)",
              r.get("motion_resumed_after_play") is True,
              f"speed_after_play={r.get('speed_after_play')} rad/s, "
              f"pan_advanced={r.get('pan_advanced_since_pause')} rad")
        check("the SAME ORIGINAL goal succeeded with NO replan (code 0)",
              r.get("final_goal_succeeded") is True,
              f"result_code={r.get('final_goal_result_code')}")
        check("true freeze-and-continue", r.get("true_freeze_and_continue") is True)
        # REGRESSION GUARD: pause_play must keep freezing-and-continuing.
        reevaluate = not (r.get("true_freeze_and_continue") is True)
        if reevaluate:
            # A regression here isn't "invalid experiment"; surface it distinctly.
            pass

    elif action == "pause_only":
        check("dashboard PAUSE executed", pause_ok, r.get("pause_call", {}).get("msg", ""))
        check("arm FROZE on its path", froze,
              f"t_freeze={r.get('time_to_freeze_s')}s")
        check("arm STAYED stopped for the whole hold (no drift/run-away)",
              (r.get("pan_drift_during_pause") or 0.0) < 0.02,
              f"drift over {r.get('pause_hold')}s hold = {r.get('pan_drift_during_pause')} rad")
        check("applied scaling stayed ~0 for the whole hold",
              scaling_zeroed, f"min scaling = {r.get('scaling_during_pause_min')}")

    elif action == "foreign_play":
        # Someone plays the program while STOP is still wanted. The hazard is that
        # motion RESUMES. We assert the hazard is real (so the gate MUST re-pause).
        check("dashboard PAUSE executed", pause_ok)
        check("arm FROZE on pause", froze)
        check("a foreign PLAY DID resume motion (=> the gate must RE-PAUSE)",
              r.get("motion_resumed_after_play") is True,
              f"speed_after_play={r.get('speed_after_play')} rad/s, "
              f"pan_advanced={r.get('pan_advanced_since_pause')} rad")

    elif action == "stop_instead":
        # dashboard stop is terminal: it must NOT deliver a same-goal continue.
        check("arm stopped moving after `stop`",
              froze or r.get("goal_aborted_by_pause") is True)
        check("the goal did NOT succeed as a same-goal continue (stop is terminal)",
              r.get("final_goal_succeeded") is not True,
              f"result_code={r.get('final_goal_result_code')}")

    elif action == "pause_no_program":
        # With nothing running, pause is rejected and the arm keeps moving: pause is
        # not a stop when no program plays (fail-open unless the backstop catches it).
        check("dashboard PAUSE was rejected (no running program to pause)",
              not pause_ok, r.get("pause_call", {}).get("msg", ""))
        check("arm did NOT freeze on the rejected pause",
              r.get("froze_after_pause") is False,
              f"drift={r.get('pan_drift_during_pause')} rad")

    return checks, reevaluate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", help="pause_investigation.txt (with a RESULT line)")
    ap.add_argument("--result", help="a bare RESULT json file (for tests)")
    ap.add_argument("--mode", choices=["headless", "pendant"], default="headless")
    a = ap.parse_args()
    if not a.file and not a.result:
        ap.error("give --file or --result")

    r = load_result(a)
    # RESULT carries its own mode; the CLI --mode is an override / default for old logs.
    mode = r.get("mode", a.mode) if isinstance(r, dict) else a.mode
    pendant = (mode == "pendant")
    checks, reevaluate = evaluate_pendant(r) if pendant else evaluate(r)

    title = ("pendant CB3, loaded External Control program" if pendant
             else "headless CB3")
    print(f"\n=== dashboard pause/resume investigation ({title}) ===")
    failed = 0
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
        if not ok:
            failed += 1

    if reevaluate:
        if pendant:
            print("\n*** RE-EVALUATE: the pendant pause_play freeze-and-continue that the "
                  "PENDANT-PAUSE STOP strategy relies on did NOT reproduce. Revisit the "
                  "strategy in PENDANT_PAUSE_FINDING.md before shipping. ***")
        else:
            print("\n*** RE-EVALUATE: dashboard pause/play EXECUTED on this run. A true "
                  "freeze-and-continue may now be available on this controller — revisit "
                  "the stop-path decision in PAUSE_RESUME_FINDING.md before shipping. ***")
        return 2

    if failed:
        print(f"\n{len(checks) - failed}/{len(checks)} checks passed — finding NOT reproduced")
        return 1
    if pendant:
        print(f"\n{len(checks)}/{len(checks)} checks passed — pendant finding holds "
              f"(action={r.get('action')}).")
    else:
        print(f"\n{len(checks)}/{len(checks)} checks passed — finding holds: no dashboard "
              "freeze-and-continue on headless CB3; STOP stays the goal-cancel backstop "
              "(stop-then-replan).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
