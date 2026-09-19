#!/usr/bin/env python3
"""
Offline tests for the sole-writer monitor (report §5 #16). NO ROS, NO robot.

The crux: a foreign write (scaling MOVES to an uncommanded value) must trip a
conflict, while an inert/ineffective slider (scaling does NOT move from its
pre-command value — the mock-hardware case) must NOT, because that is the
backstop's job, not a sole-writer conflict.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from sole_writer import SoleWriterMonitor  # noqa: E402


def test_in_sync_no_conflict():
    m = SoleWriterMonitor()
    m.note_command(1.0, 0.0)
    for t in range(1, 20):
        assert m.observe(1.0, t * 0.1) is None
    assert not m.conflict


def test_ramp_to_commanded_value_tolerated():
    m = SoleWriterMonitor(settle_s=0.5)
    # slider at 1.0, gate commands SLOW 0.1; scaling ramps 1.0 -> 0.1 over 0.3 s
    m.note_command(1.0, 0.0); m.observe(1.0, 0.0)
    m.note_command(0.1, 1.0)
    assert m.observe(0.6, 1.05) is None      # mid-ramp, within settle
    assert m.observe(0.3, 1.1) is None
    assert m.observe(0.1, 1.3) is None       # arrived
    assert not m.conflict


def test_inert_slider_is_not_a_conflict():
    # mock hardware: scaling stuck at 1.0 no matter what we command.
    m = SoleWriterMonitor(settle_s=0.3)
    m.note_command(1.0, 0.0); m.observe(1.0, 0.0)
    m.note_command(0.0, 1.0)                  # command STOP
    for t in range(1, 30):
        r = m.observe(1.0, 1.0 + t * 0.1)     # scaling never moves from 1.0
        assert r is None
    assert not m.conflict                     # -> backstop path handles it


def test_foreign_write_trips_conflict():
    m = SoleWriterMonitor(settle_s=0.3, tol=0.05)
    m.note_command(0.1, 0.0); m.observe(0.1, 0.0)   # gate holds SLOW 0.1, in sync
    # someone else shoves the slider to 0.8 and it stays there
    assert m.observe(0.8, 1.0) is None              # within settle window
    r = m.observe(0.8, 1.4)                          # sustained past settle -> foreign
    assert r is not None and m.conflict
    assert "foreign speed-slider write" in r


def test_conflict_latches():
    m = SoleWriterMonitor(settle_s=0.1)
    m.note_command(0.1, 0.0); m.observe(0.1, 0.0)
    m.observe(0.8, 1.0); m.observe(0.8, 1.2)         # trip
    assert m.conflict
    # even if the slider returns to the commanded value, the conflict stays
    assert m.observe(0.1, 2.0) is None
    assert m.conflict
    m.reset()
    assert not m.conflict


def test_no_conflict_before_first_command():
    m = SoleWriterMonitor(settle_s=0.1)
    for t in range(10):
        assert m.observe(0.5, t * 0.1) is None       # gate has not taken control
    assert not m.conflict


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
