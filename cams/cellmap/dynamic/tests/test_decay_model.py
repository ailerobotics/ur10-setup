#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the occupancy decay model — NO ROS, NO GPU.

Asserts the two properties the captain asked of the dynamic layer, in the model
that mirrors the YAML: a fresh obstacle is NOT missed (it appears within a few
observations) and a departed obstacle does NOT linger forever (it clears in a
bounded time) but also does not flicker out instantly.

Run:  python3 test_decay_model.py   |   pytest test_decay_model.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from decay_model import (  # noqa: E402
    DecayParams, frames_to_appear, half_life_s, seconds_to_appear, time_to_clear_s)


def test_fresh_obstacle_not_missed():
    p = DecayParams()
    # a handful of observations is enough to enter the ESDF
    assert 1 <= frames_to_appear(p) <= 5
    assert seconds_to_appear(p) <= 0.6


def test_departed_obstacle_clears_bounded():
    p = DecayParams()
    clear = time_to_clear_s(p)
    assert clear > 0.0, "must not clear before any decay"
    assert clear < 10.0, "stale obstacle must not linger indefinitely"


def test_faster_decay_clears_sooner():
    slow = DecayParams(occupied_decay_prob=0.10)   # weak pull toward free
    fast = DecayParams(occupied_decay_prob=0.45)   # strong pull toward free
    assert time_to_clear_s(fast) < time_to_clear_s(slow)


def test_half_life_positive():
    assert half_life_s(DecayParams()) > 0.0


def test_validation_rejects_bad_params():
    for bad in (dict(integrate_rate_hz=0.0), dict(occupied_obs_prob=1.0),
                dict(free_obs_prob=0.0), dict(p_present=-0.1)):
        try:
            DecayParams(**bad).validate()
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for {bad}")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
