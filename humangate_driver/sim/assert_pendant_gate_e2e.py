#!/usr/bin/env python3
"""Assert the PENDANT-PAUSE gate end-to-end proof (run_pendant_gate_e2e.sh): the real
gate_driver_node, with stop_strategy=pendant_pause, made the arm FREEZE on its path
when a person was close (dashboard PAUSE) and CONTINUE the SAME trajectory to
completion when the person left and the CLEAR dwell elapsed (dashboard PLAY) — with no
replan and without the goal-cancel backstop firing.

Reads the gate's decision log, the mover's progress log, and the watcher log. Exit 0
= the pendant-pause gate behaviour held; nonzero = it did not.
"""
import argparse
import json
import sys


def load_jsonl(path):
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
    ap.add_argument("--decisions", required=True)
    ap.add_argument("--mover", required=True)
    ap.add_argument("--watcher", default=None)
    a = ap.parse_args()

    dec = load_jsonl(a.decisions)
    mov = load_jsonl(a.mover)
    wat = load_jsonl(a.watcher) if a.watcher else []

    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    # --- the gate used the pendant-pause strategy ---------------------------- #
    strategies = {d.get("stop_strategy") for d in dec}
    check("gate ran the pendant_pause strategy", strategies == {"pendant_pause"},
          str(strategies))

    first_run_i = next((i for i, d in enumerate(dec) if d.get("action") == "RUN"), None)
    check("gate reached RUN (motion enabled)", first_run_i is not None)

    # --- a PAUSE happened AFTER motion started (not just the startup STOP) ----- #
    # The meaningful pause is a paused-STOP that comes AFTER the first RUN — i.e. the
    # gate pausing a moving trajectory, not the startup hold before anything ran.
    paused_after_run = [i for i, d in enumerate(dec)
                        if first_run_i is not None and i > first_run_i
                        and d.get("action") == "STOP" and d.get("paused") is True]
    check("the gate PAUSED the program AFTER motion started (paused=true post-RUN)",
          len(paused_after_run) > 0, f"{len(paused_after_run)} paused-STOP decisions after RUN")

    # --- RUN -> (paused STOP after RUN) -> RUN (freeze then resume) ----------- #
    resumed_run = None
    if paused_after_run:
        resumed_run = next((i for i, d in enumerate(dec)
                            if i > paused_after_run[0] and d.get("action") == "RUN"), None)
    check("RUN -> paused STOP -> RUN (freeze then resume via the gate)",
          first_run_i is not None and paused_after_run and resumed_run is not None,
          f"run@{first_run_i}, paused@{paused_after_run[0] if paused_after_run else None}, "
          f"resumed@{resumed_run}")

    # --- the mover's SAME (monotonic) goal froze MID-motion, then completed ---- #
    # A monotonic ramp only flat-lines at an INTERMEDIATE progress value when it is
    # actually paused (its ends are 0 and 1; there is no turnaround), so a >=3 s window
    # with <0.01 change at 0.1 < tps < 0.9 is an unambiguous freeze.
    prog = [(m["elapsed"], m["tps"]) for m in mov if "tps" in m and "elapsed" in m]
    froze_mid = False
    freeze_tps = None
    for i in range(len(prog)):
        if not (0.1 < prog[i][1] < 0.9):
            continue
        j = i
        while j < len(prog) and prog[j][0] - prog[i][0] < 3.0:
            j += 1
        if j < len(prog) and abs(prog[j][1] - prog[i][1]) < 0.01:
            froze_mid = True
            freeze_tps = prog[i][1]
            break
    check("trajectory progress FROZE mid-motion for >=3 s (the pause)",
          froze_mid, f"flat at tps={freeze_tps}")

    max_tps = max((t for _, t in prog), default=0.0)
    check("the SAME trajectory then CONTINUED to the goal (progress reached ~1.0)",
          max_tps > 0.95, f"max tps={max_tps:.4f}")

    status = next((m.get("status") for m in mov if m.get("done")), None)
    check("the ORIGINAL mover goal SUCCEEDED with no replan (code=0)",
          status == "finished(code=0)", str(status))

    # --- the backstop did NOT cancel the ACTIVE goal (it would have aborted it) --
    # The goal succeeding with code 0 already proves no cancel reached it. Separately,
    # no watcher cancel should be an INEFFECTIVE-STOP / BACKSTOP cancel (those would
    # fire during a pause); a heartbeat-loss cancel AFTER the scenario ends (the
    # publisher stops) is benign and expected, so it is allowed.
    bad_cancels = [w for w in wat if "heartbeat" not in (w.get("reason") or "")]
    check("no backstop cancel of the active pause (only benign end-of-run heartbeat cancels, if any)",
          len(bad_cancels) == 0,
          f"{len(wat)} total watcher cancels, {len(bad_cancels)} non-heartbeat")

    print("\n=== pendant-pause GATE end-to-end ===")
    failed = 0
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
        if not ok:
            failed += 1
    if failed:
        print(f"\n{len(checks) - failed}/{len(checks)} checks passed — pendant-pause gate "
              "behaviour NOT proven")
        return 1
    print(f"\n{len(checks)}/{len(checks)} checks passed — the gate froze the arm on its "
          "path (PAUSE) and continued the SAME trajectory on CLEAR+dwell (PLAY), no replan, "
          "no backstop.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
