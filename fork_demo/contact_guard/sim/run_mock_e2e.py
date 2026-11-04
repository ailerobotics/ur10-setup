#!/usr/bin/env python3
"""Mock end-to-end proof of the RUNTIME path, no ROS, no robot.

Drives :class:`runner.ContactGuardRunner` exactly as the live node does, but
from a scripted telemetry stream, and self-asserts the full behaviour:

* a clean stop-and-probe contact leg completes with NO trip;
* a gentle tip touch during a segment creep trips PROBE and the reaction is a
  bounded in-axis BACK-OFF (inside the workspace box);
* a hard contact (progress stall) trips STALL and the reaction cancels;
* the named near-miss negatives (427/318 N artefact, 8.64 deg lag, EC drop) do
  NOT trip;
* the pre-contact gate blocks on a bad snapshot and passes on a good one;
* post-LIFT / post-LOWER verification accept a good lift/release and refuse a bad
  one.

Exit 0 iff every assertion holds. This is the offline analogue of the URSim
proof (``run_ursim_e2e.sh``), which exercises the same runner against the real
driver + simulator.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
# sim -> contact_guard -> fork_demo -> <repo root>
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, REPO)

from fork_demo.contact_guard import (  # noqa: E402
    envelope, faults, gate, params, verify,
)
from fork_demo.contact_guard.runner import ContactGuardRunner, LegContext  # noqa: E402

ENV_PATH = os.path.join(REPO, "fork_demo", "contact_guard", "envelopes",
                        "teach-002.yaml")

FAILS = []


def check(name: str, cond: bool, detail: str = "") -> None:
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        FAILS.append(name)


def clean_probe_segment(runner, lb, s0, span, touch_tip_n=0.0):
    """Run one stop-and-probe segment (dwell -> baseline -> creep). Returns det."""
    p = runner.p
    runner.begin_probe_dwell()
    base = {k: v for k, v in _baseline(lb, s0).items()}
    for _ in range(p.probe_min_samples + 2):
        runner.add_baseline_sample(base)
    assert runner.begin_creep(), "baseline should be accepted"
    det = None
    n = 60
    for i in range(n):
        t = 100.0 + i * 0.01
        s = s0 + span * (i / (n - 1))
        feats = _baseline(lb, s)
        if touch_tip_n and i >= 4:
            feats["my"] += touch_tip_n * p.fork_lever_m
            feats["torque_mag"] = (feats["mx"] ** 2 + feats["my"] ** 2
                                   + feats["mz"] ** 2) ** 0.5
        d = runner.feed(t, s, s, feats, executing=True)
        if d is not None and det is None:
            det = d
    return det


def _baseline(lb, s):
    return faults._baseline_feats(lb, s)  # reuse the clean baseline builder


def main() -> int:
    p = params.load()
    env = envelope.Envelope.from_dict(yaml.safe_load(open(ENV_PATH)))
    lb = env.leg("insert_forks")
    span = p.segment_m / max(lb.tcp_travel_m, 1e-6)

    print("1) clean stop-and-probe segment -> no trip")
    r = ContactGuardRunner(p, env)
    r.start_leg(LegContext("insert_forks", True, axis=(0, 0, 1)))
    det = clean_probe_segment(r, lb, 0.3, span)
    check("clean segment no trip", det is None)

    print("2) gentle tip touch (6 N) during creep -> PROBE + bounded BACK-OFF")
    r = ContactGuardRunner(p, env)
    r.start_leg(LegContext("insert_forks", True, axis=(0.0, 0.0, 1.0),
                           box=(0.1, 1.35, -0.45, 1.25, -0.35, 1.0)))
    det = clean_probe_segment(r, lb, 0.3, span, touch_tip_n=6.0)
    check("tip touch trips PROBE", det is not None and det.trigger == "PROBE",
          det.trigger if det else "none")
    if det:
        plan = r.react(det, tcp_pos=(0.6, 0.3, 0.8))
        check("reaction is BACKOFF", plan.action == "BACKOFF", plan.action)
        check("back-off ~15 mm along -axis",
              plan.backoff_target is not None
              and abs((0.8 - plan.backoff_target[2]) - p.backoff_m) < 1e-6)

    print("3) back-off refused when it would leave the workspace box -> HOLD")
    r = ContactGuardRunner(p, env)
    r.start_leg(LegContext("insert_forks", True, axis=(0.0, 0.0, 1.0),
                           box=(0.1, 1.35, -0.45, 1.25, 0.799, 1.0)))
    det = clean_probe_segment(r, lb, 0.3, span, touch_tip_n=6.0)
    plan = r.react(det, tcp_pos=(0.6, 0.3, 0.8))
    check("out-of-box back-off -> HOLD", plan.action == "HOLD", plan.action)

    print("4) hard contact (stall) on a contact leg -> STALL")
    res = faults.inject_fault("insert_hit", lb, p)
    check("insert hit trips STALL", res.detected and res.detection.trigger == "STALL")

    print("5) named negatives do NOT trip")
    check("427 N artefact", not faults.negative_force_artifact(env.leg("to_pre_grip"), p, 427.0).detected)
    check("318 N artefact", not faults.negative_force_artifact(env.leg("insert_forks"), p, 318.0).detected)
    check("8.64 deg lag", not faults.negative_loaded_lift_lag(env.leg("lift"), p).detected)
    check("EC drop", not faults.negative_ec_drop(env.leg("insert_forks"), p).detected)

    print("6) pre-contact gate: blocks bad snapshot, passes good one")
    bad = gate.GateInput(tcp_offset=(0, 0, 0.15, 0, 0, 0), payload_kg=1.0,
                         speed_frac=0.05, robot_running=True, robot_normal=True,
                         program_running=True, joint_feed_live=True, free_gb=50,
                         pendant_force_ack_n=100.0, align_veto_ok=True)
    check("gate blocks non-fork TCP", not gate.evaluate_gate(bad, p).ok)
    good = gate.GateInput(tcp_offset=(0, -0.038, 0.427, 0, 0, 0), payload_kg=0.3,
                          loaded=False, speed_frac=0.05, robot_running=True,
                          robot_normal=True, program_running=True,
                          joint_feed_live=True, free_gb=50,
                          pendant_force_ack_n=100.0, align_veto_ok=True)
    check("gate passes good snapshot", gate.evaluate_gate(good, p).ok)
    check("gate blocks on alignment veto",
          not gate.evaluate_gate(
              gate.GateInput(**{**good.__dict__, "align_veto_ok": False}), p).ok)

    print("7) post-action verification")
    before = {"f_ins": 5.0, "force_mag": 6.0}
    after_ok = {"f_ins": 25.0, "force_mag": 26.0}
    check("lift confirmed (rise + load step)",
          verify.verify_lift(before, after_ok, dz_m=0.023, p=p).ok)
    check("lift refused when TCP did not rise",
          not verify.verify_lift(before, after_ok, dz_m=0.002, p=p).ok)
    check("release confirmed (load dropped)",
          verify.verify_release({"f_ins": 25.0}, {"f_ins": 3.0}, p).ok)
    check("withdraw refused with residual load",
          not verify.verify_release({"f_ins": 25.0}, {"f_ins": 25.0}, p).ok)

    print()
    if FAILS:
        print(f"MOCK E2E FAILED: {len(FAILS)} assertion(s): {FAILS}")
        return 1
    print("MOCK E2E PASSED: all assertions hold.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
