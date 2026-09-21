# SPDX-License-Identifier: Apache-2.0
"""Regression guard for the nearest-point safety fix, replaying a committed Isaac
after-run detection log through the OLD (body-centre) vs NEW (nearest-point) zone
rule.

The fixture `data/after_run_replay/{record,ground_truth}.jsonl` is a compact,
scripted stand-in for one Isaac after-run of the fix (report
`data/human-safety-nearest-point-fix-v1/report.md`), written in the SAME schema
the live recorder emits, so it drives the REAL consumer
`compare_fix.build_comparison` unchanged. Each state carries both the body centre
(`position`) and the nearest body point (`nearest_point`); replaying the SAME
records with `use_nearest=False` reproduces the pre-fix centre behaviour and with
`use_nearest=True` the fix, so the two differ ONLY by the distance rule (this is
the production `SafetyEvaluator.evaluate` path — revert the nearest-point logic in
`cams/human_safety/human_safety/safety.py` and the new-rule leg regresses to the
centre leg, failing the residual bound below).

Guards (distance-only view — latency fault disabled, so the state is sized by
distance + hysteresis + dwell only, the honest safety picture of the report):

  1. the fix does not add zone chatter: new-rule zone changes <= old-rule;
  2. the fix is never less protective: new-rule STOP time >= old-rule STOP time;
  3. the fix's residual missed-STOP interval is no worse than the 0.93 s the
     report documents (and no worse than the old rule's).

No threshold is changed here (the sim config's `safety:` block is a verbatim copy
of production, asserted byte-for-byte by test_sim_config.py).

NOTE: no real-rig human walk-through recording exists yet — the raw Isaac
bags/frames were intentionally not committed and a live rig capture has not been
taken. This fixture is a scripted reconstruction of the after-run scenario, not a
real-rig or raw-sim dump. When a real walk-through recording is captured, drop it
in beside this one (same two-file schema) and extend this test to replay it the
same way. Regenerate this fixture with
`python3 data/make_after_run_replay.py`.
"""
import os

import pytest

import compare_fix as cf
from human_safety.config import load_config

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(_HERE)
FIXTURE = os.path.join(_HERE, "data", "after_run_replay")
SIM_CONFIG = os.path.join(_HARNESS, "params", "human_safety.sim.yaml")

# The report's documented distance-only residual on the after-run: the new
# (nearest-point) rule still engages STOP ~0.93 s late (improved, not PASS). The
# regression must not exceed it.
RESIDUAL_BUDGET_S = 0.93


@pytest.fixture(scope="module")
def comparison():
    p = load_config(SIM_CONFIG).safety
    return cf.build_comparison(FIXTURE, p, p.reach_radius, p.stop_radius,
                               p.reaction_time_budget)


def test_fixture_is_compact(comparison):
    # Committed fixture must stay a small artifact (<= 300 KB) so it travels with
    # the code without bloating the repo.
    total = sum(os.path.getsize(os.path.join(FIXTURE, n))
                for n in ("record.jsonl", "ground_truth.jsonl"))
    assert total <= 300 * 1024, f"fixture is {total} bytes (> 300 KB)"


def test_new_reconstruction_reproduces_live_zones(comparison):
    # Sanity: the NEW reconstruction reproduces the recorded (live) zone stream,
    # which is what proves the OLD reconstruction differs only by the rule.
    assert comparison["fidelity_new_vs_live_frac"] > 0.95


def test_fix_adds_no_zone_chatter(comparison):
    old = comparison["old_distance_only"]
    new = comparison["new_distance_only"]
    # Guard 1: the nearest rule must not increase the number of zone changes.
    assert new["zone_changes"] <= old["zone_changes"]


def test_fix_is_never_less_protective(comparison):
    old = comparison["old_distance_only"]
    new = comparison["new_distance_only"]
    # Guard 2: the nearest rule must hold STOP at least as long as the centre rule
    # (it engages STOP earlier, so it can only add STOP time).
    assert new["stop_time_s"] >= old["stop_time_s"] - 1e-9
    # Both rules must actually exercise the STOP zone or the comparison is vacuous.
    assert new["stop_time_s"] > 0.0
    assert old["stop_time_s"] > 0.0


def test_residual_missed_stop_within_budget(comparison):
    old = comparison["old_distance_only"]
    new = comparison["new_distance_only"]
    # Guard 3: the fix's residual missed-STOP interval is no worse than the
    # documented 0.93 s, and no worse than the pre-fix centre rule. The old
    # (centre) rule must still show a genuine miss, so this is a real before/after.
    assert old["n_missed"] >= 1
    assert old["worst_missed_dur_s"] > new["worst_missed_dur_s"]
    assert new["worst_missed_dur_s"] <= RESIDUAL_BUDGET_S + 1e-9
