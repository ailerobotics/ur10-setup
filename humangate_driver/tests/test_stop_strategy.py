#!/usr/bin/env python3
"""Offline unit tests for stop_strategy.py — the selectable STOP strategy (cancel vs
pendant_pause), its resume timeout, and the re-pause-on-foreign-play rule. No ROS, no
robot, no docker: fakes record the slider / dashboard / cancel calls and time is
passed in.

Run: python3 tests/test_stop_strategy.py   (or: pytest -q tests)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from stop_strategy import (  # noqa: E402
    BACKSTOP, RUN, SLOW, STOP, CancelHoldStrategy, PendantPauseParams,
    PendantPauseStrategy, make_stop_strategy)


class FakeSlider:
    def __init__(self):
        self.writes = []

    def set(self, f):
        self.writes.append(round(float(f), 4))

    @property
    def last(self):
        return self.writes[-1] if self.writes else None


class FakeCanceller:
    def __init__(self):
        self.cancels = 0

    def cancel_all(self):
        self.cancels += 1


class FakeDashboard:
    def __init__(self):
        self.pauses = 0
        self.plays = 0
        self.stops = 0

    def pause(self):
        self.pauses += 1

    def play(self):
        self.plays += 1

    def stop(self):
        self.stops += 1


def _cancel_strat():
    s, c = FakeSlider(), FakeCanceller()
    return CancelHoldStrategy(s, c), s, c


def _pendant(resume_max=120.0):
    s, c, d = FakeSlider(), FakeCanceller(), FakeDashboard()
    strat = PendantPauseStrategy(s, c, d, params=PendantPauseParams(resume_max))
    return strat, s, c, d


# ---- CancelHoldStrategy: reproduces the shipped behaviour ------------------ #

def test_cancel_run_slow_stop_are_the_slider():
    strat, s, c = _cancel_strat()
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(SLOW, 0.1, "slow", 0.1)
    strat.apply(STOP, 0.0, "stop", 0.2)
    assert s.writes == [1.0, 0.1, 0.0]
    assert c.cancels == 0                      # no cancel until BACKSTOP


def test_cancel_backstop_cancels_and_zeros():
    strat, s, c = _cancel_strat()
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(BACKSTOP, None, "backstop", 1.0)
    assert c.cancels == 1
    assert s.last == 0.0


def test_cancel_backstop_is_throttled():
    strat, s, c = _cancel_strat()
    for t in (0.0, 0.05, 0.1, 0.9):            # all within 1 s reassert window
        strat.apply(BACKSTOP, None, "b", t)
    assert c.cancels == 1
    strat.apply(BACKSTOP, None, "b", 1.2)      # past the window -> re-cancel
    assert c.cancels == 2


# ---- PendantPauseStrategy -------------------------------------------------- #

def test_pendant_stop_pauses_the_program():
    strat, s, c, d = _pendant()
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(STOP, 0.0, "person close", 1.0)
    assert d.pauses == 1 and strat.paused is True
    assert s.last == 0.0                       # belt-and-suspenders slider zero
    assert c.cancels == 0                      # a pause is NOT a cancel


def test_pendant_stop_then_run_resumes_same_trajectory():
    strat, s, c, d = _pendant(resume_max=120.0)
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(STOP, 0.0, "stop", 1.0)
    strat.apply(RUN, 1.0, "clear+dwell", 30.0)  # within the resume window
    assert d.plays == 1                         # play -> continue the SAME trajectory
    assert c.cancels == 0                       # no replan
    assert strat.paused is False
    assert s.last == 1.0                        # slider restored to operating speed


def test_pendant_resume_refused_when_paused_too_long():
    strat, s, c, d = _pendant(resume_max=10.0)
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(STOP, 0.0, "stop", 1.0)
    strat.apply(RUN, 1.0, "clear+dwell", 20.0)  # 19 s > 10 s window
    assert d.plays == 0                         # NOT resumed as the same trajectory
    assert c.cancels == 1                       # cancelled -> a fresh plan is needed
    assert strat.resume_needs_replan is True
    assert strat.paused is False


def test_pendant_slow_after_stop_stays_frozen():
    """A retreat to the SLOW ring is not a CLEAR+dwell: the arm must stay paused,
    never resume, and the slider must not be raised."""
    strat, s, c, d = _pendant()
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(STOP, 0.0, "stop", 1.0)
    d.plays = 0
    strat.apply(SLOW, 0.1, "person retreated to slow ring", 2.0)
    assert d.plays == 0                         # did NOT resume
    assert strat.paused is True                 # still frozen
    # slider was left at 0 (the STOP value); SLOW did not raise it
    assert s.last == 0.0


def test_pendant_foreign_play_during_stop_is_repaused():
    strat, s, c, d = _pendant()
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(STOP, 0.0, "stop", 1.0, program_running=False)
    assert d.pauses == 1
    # someone plays the program on the pendant: program_running goes False -> True
    strat.apply(STOP, 0.0, "stop", 2.0, program_running=True)
    assert d.pauses == 2                        # re-paused
    assert strat.conflict is True
    assert "foreign PLAY" in strat.conflict_reason
    # it stays True while running but does not spam a new pause every tick
    strat.apply(STOP, 0.0, "stop", 2.05, program_running=True)
    assert d.pauses == 2


def test_pendant_foreign_play_during_slow_hold_is_repaused():
    strat, s, c, d = _pendant()
    strat.apply(RUN, 1.0, "run", 0.0)
    strat.apply(STOP, 0.0, "stop", 1.0, program_running=False)
    strat.apply(SLOW, 0.1, "slow-hold", 2.0, program_running=True)
    assert d.pauses == 2 and strat.conflict is True


def test_pendant_run_clears_conflict():
    strat, s, c, d = _pendant()
    strat.apply(STOP, 0.0, "stop", 1.0, program_running=False)
    strat.apply(STOP, 0.0, "stop", 2.0, program_running=True)
    assert strat.conflict is True
    strat.apply(RUN, 1.0, "clear+dwell", 3.0, program_running=True)
    assert strat.conflict is False


def test_pendant_backstop_supersedes_pause():
    strat, s, c, d = _pendant()
    strat.apply(STOP, 0.0, "stop", 1.0)
    assert strat.paused is True
    strat.apply(BACKSTOP, None, "self-fault", 2.0)
    assert c.cancels == 1 and strat.paused is False
    assert s.last == 0.0


# ---- factory --------------------------------------------------------------- #

def test_factory_rejects_unknown_and_dashboardless_pendant():
    s, c, d = FakeSlider(), FakeCanceller(), FakeDashboard()
    make_stop_strategy("cancel", s, c)                       # ok
    make_stop_strategy("pendant_pause", s, c, dashboard=d)   # ok
    for bad in (lambda: make_stop_strategy("nope", s, c),
                lambda: make_stop_strategy("pendant_pause", s, c, dashboard=None)):
        try:
            bad(); raise AssertionError("expected ValueError")
        except ValueError:
            pass


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    fail = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS {fn.__name__}")
        except AssertionError as e:
            fail += 1
            print(f"  FAIL {fn.__name__}: {e}")
    print(f"\n{len(fns) - fail}/{len(fns)} tests passed")
    sys.exit(1 if fail else 0)
