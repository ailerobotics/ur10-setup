"""Bag-replay proof: the detector over the whole clean teach-002 record trips
zero times. Skips when the mcap reader or the bag is unavailable (CI without the
mcap venv); the envelope-baseline replay below is the bag-free proxy that always
runs.
"""
import os

import pytest

from fork_demo.contact_guard import bagsource, dryrun, faults
from fork_demo.contact_guard.detector import ContactGuard

BAG = os.path.join(bagsource.TEACH_DIR_DEFAULT, "bag", "bag_0.mcap")


def _have_mcap():
    try:
        import mcap_ros2.reader  # noqa: F401
        return True
    except Exception:
        return False


@pytest.mark.skipif(not (_have_mcap() and os.path.exists(BAG)),
                    reason="mcap reader or teach-002 bag not available")
def test_teach002_zero_false_trips(env, p):
    rep = dryrun.dry_run_bag(bagsource.TEACH_DIR_DEFAULT, BAG, p, env)
    assert rep.n_trips == 0, rep.text()
    assert rep.min_margin > 0.0            # the clean record sits inside the band


def test_clean_baseline_replay_never_trips(env, p):
    """Bag-free proxy: a perfectly-tracked replay of every leg from its own
    envelope baselines must not trip (force-alone-never-trips guarantee)."""
    for name, lb in env.legs.items():
        g = ContactGuard(p, lb, name, lb.contact)
        det = None
        for s in faults.clean_replay(lb):
            det = g.update(s.t, s.s_cmd, s.s_act, s.feats, executing=True) or det
        assert det is None, f"{name} false-tripped on its own clean baseline"
