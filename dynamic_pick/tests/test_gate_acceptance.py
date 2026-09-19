#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the gate acceptance shim — the SOLE motion authority
(report §2). The load-bearing proof: FORWARD happens ONLY on a fresh RUN; every
other action, and any stale/missing decision, HOLDs."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from gate_acceptance import (  # noqa: E402
    BACKSTOP, FORWARD, GateAcceptanceShim, GateView, HOLD, RUN, SLOW, STOP)


def test_forward_only_on_fresh_run():
    shim = GateAcceptanceShim(max_decision_age_s=0.3)
    r = shim.admit(GateView(RUN, 10.0), now=10.1)
    assert r.verdict == FORWARD and r.gate_fresh


def test_every_non_run_action_holds():
    shim = GateAcceptanceShim(max_decision_age_s=0.3)
    for act in (SLOW, STOP, BACKSTOP, "WHATEVER", None):
        r = shim.admit(GateView(act, 10.0), now=10.1)
        assert r.verdict == HOLD, f"{act} should HOLD"


def test_stale_run_holds():
    # a RUN decision older than the freshness window is NOT clearance.
    shim = GateAcceptanceShim(max_decision_age_s=0.3)
    r = shim.admit(GateView(RUN, 10.0), now=10.5)   # 500 ms old
    assert r.verdict == HOLD and not r.gate_fresh


def test_never_seen_holds():
    shim = GateAcceptanceShim()
    r = shim.admit(GateView(None, None), now=5.0)
    assert r.verdict == HOLD and not r.gate_fresh


def test_exhaustive_no_motion_unless_run():
    """The task invariant: across the full cross-product of gate actions and
    freshness, a FORWARD is emitted iff (action == RUN and fresh)."""
    shim = GateAcceptanceShim(max_decision_age_s=0.3)
    now = 100.0
    actions = [RUN, SLOW, STOP, BACKSTOP, "UNKNOWN", None]
    ages = [0.0, 0.1, 0.29, 0.31, 1.0, None]     # None == never received
    for act in actions:
        for age in ages:
            t = None if age is None else now - age
            r = shim.admit(GateView(act if t is not None else None, t), now)
            fresh = (t is not None) and (age <= 0.3)
            expect_forward = (act == RUN and t is not None and fresh)
            got_forward = (r.verdict == FORWARD)
            assert got_forward == expect_forward, (
                f"action={act} age={age}: expected forward={expect_forward}, got {r.verdict}")


def test_freshness_boundary():
    shim = GateAcceptanceShim(max_decision_age_s=0.3)
    assert shim.admit(GateView(RUN, 0.0), now=0.30).verdict == FORWARD    # exactly at bound
    assert shim.admit(GateView(RUN, 0.0), now=0.31).verdict == HOLD       # just past


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn(); print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1; print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1; print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
