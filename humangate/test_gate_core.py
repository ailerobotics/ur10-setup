#!/usr/bin/env python3
"""
Offline unit tests for the gate decision core — NO ROS, NO robot.

Covers every fail-toward-stop rule (report §5, §7 / task brief item 3), the
primary->backstop fall-through (item 4), and a property-style check that no input
sequence yields RUN without a preceding healthy CLEAR dwell.

Run standalone:  python3 test_gate_core.py
Or under pytest: pytest test_gate_core.py
"""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from camera_state import (  # noqa: E402
    HEALTH_TRACK_LOST_CLOSE, ZONE_CLEAR, ZONE_SLOW, ZONE_STOP, CameraState)
from gate_config import GateConfig  # noqa: E402
import gate_core as gc  # noqa: E402


def cfg(**kw):
    base = dict(clear_dwell_s=2.0, heartbeat_timeout_s=0.3, state_max_age_s=0.2,
                backstop_deadline_s=0.5)
    base.update(kw)
    return GateConfig(**base).validate()


def clear(stamp, seq=1, health_ok=True, flags=()):
    return CameraState(stamp=stamp, zone=ZONE_CLEAR, health_ok=health_ok,
                       health_flags=frozenset(flags), heartbeat_seq=seq)


def zone(z, stamp, dist=None, seq=1, health_ok=True, flags=()):
    return CameraState(stamp=stamp, zone=z, nearest_distance_m=dist,
                       health_ok=health_ok, health_flags=frozenset(flags),
                       heartbeat_seq=seq)


# --- rule: startup with no state -> STOP -----------------------------------

def test_startup_no_state_is_stop():
    g = gc.HumanGate(cfg())
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.STOP
    assert "startup" in d.reason
    assert d.speed_fraction == 0.0


# --- rule: stale evidence stamp (>200 ms) -> STOP --------------------------

def test_stale_stamp_is_stop():
    g = gc.HumanGate(cfg())
    g.on_state(clear(stamp=0.0, seq=1), now=0.10)
    # arrival fresh (0.02 s ago) but evidence stamp is 0.30 s old by the wall clock
    d = g.decide(now=0.12, now_wall=0.30)
    assert d.action == gc.STOP
    assert "stale" in d.reason


def test_fresh_stamp_within_budget_is_not_stale():
    g = gc.HumanGate(cfg())
    g.on_state(clear(stamp=0.05, seq=1), now=0.05)
    d = g.decide(now=0.06, now_wall=0.10)  # 0.05 s old, under 0.2
    assert "stale" not in d.reason


# --- rule: heartbeat timeout (no msg in 300 ms) -> STOP --------------------

def test_heartbeat_timeout_is_stop():
    g = gc.HumanGate(cfg())
    g.on_state(clear(stamp=0.0, seq=1), now=0.0)
    d = g.decide(now=0.40, now_wall=0.40)  # 0.4 s since last arrival
    assert d.action == gc.STOP
    assert "heartbeat timeout" in d.reason


# --- rule: heartbeat_seq stalls -> STOP ------------------------------------

def test_heartbeat_seq_stall_is_stop():
    g = gc.HumanGate(cfg())
    t = 0.0
    for _ in range(6):                # messages keep arriving, seq frozen at 5
        g.on_state(clear(stamp=t, seq=5), now=t)
        t += 0.1
    d = g.decide(now=0.5, now_wall=0.5)  # arrival is fresh, but seq unchanged 0.5 s
    assert d.action == gc.STOP
    assert "stalled" in d.reason


# --- rule: unknown distance is never treated as "far" ----------------------

def test_unknown_distance_present_person_never_runs():
    g = gc.HumanGate(cfg())
    t = 0.0
    ran = False
    for i in range(60):              # 6 s of SLOW with unknown distance
        g.on_state(zone(ZONE_SLOW, stamp=t, dist=None, seq=i + 1), now=t)
        d = g.decide(now=t, now_wall=t)
        ran = ran or d.action == gc.RUN
        t += 0.1
    assert not ran
    assert d.action == gc.SLOW


# --- rule: health fault -> never trust CLEAR; SLOW stays SLOW, else STOP ----
# The camera side forces a non-clear zone on any fault it detects, so a fault
# reported together with CLEAR is internally inconsistent and cannot be trusted;
# the gate fails toward stop on anything it cannot confirm.

def test_health_fault_with_clear_zone_is_stop():
    g = gc.HumanGate(cfg())
    g.on_state(clear(stamp=0.0, seq=1, health_ok=False), now=0.0)
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.STOP
    assert "health" in d.reason


def test_health_flag_with_clear_zone_is_stop():
    g = gc.HumanGate(cfg())
    g.on_state(clear(stamp=0.0, seq=1, health_ok=True, flags=("STALE_CAM",)), now=0.0)
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.STOP


def test_health_fault_with_slow_zone_stays_slow():
    # camera judged itself degraded but is still tracking (SLOW) -> stay SLOW.
    g = gc.HumanGate(cfg())
    g.on_state(zone(ZONE_SLOW, stamp=0.0, dist=2.4, seq=1, health_ok=False), now=0.0)
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.SLOW
    assert "health" in d.reason


def test_health_fault_with_stop_zone_is_stop():
    g = gc.HumanGate(cfg())
    g.on_state(zone(ZONE_STOP, stamp=0.0, dist=1.0, seq=1, health_ok=False), now=0.0)
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.STOP


# --- rule: no RUN until the gate observes the full continuous CLEAR dwell ---

def _run_clear_stream(g, dwell_probe_s, dt=0.1, start_seq=1):
    """Feed continuous healthy CLEAR and return the (t, decision) list."""
    out = []
    t = 0.0
    seq = start_seq
    while t <= dwell_probe_s + 1e-9:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        out.append((t, g.decide(now=t, now_wall=t)))
        t = round(t + dt, 9)
        seq += 1
    return out


def test_clear_holds_stop_until_dwell_then_runs():
    g = gc.HumanGate(cfg(clear_dwell_s=2.0))
    trace = _run_clear_stream(g, dwell_probe_s=3.0)
    # before the dwell completes: STOP; at/after 2.0 s: RUN
    for t, d in trace:
        if t < 2.0 - 1e-9:
            assert d.action == gc.STOP, f"t={t}: expected STOP, got {d.action}"
        elif t >= 2.0:
            assert d.action == gc.RUN, f"t={t}: expected RUN, got {d.action}"
    assert trace[-1][1].speed_fraction == g.cfg.operating_speed_fraction


def test_run_resets_to_stop_when_person_reappears():
    g = gc.HumanGate(cfg(clear_dwell_s=1.0))
    _run_clear_stream(g, dwell_probe_s=1.5)          # arm to RUN
    g.on_state(zone(ZONE_STOP, stamp=2.0, dist=1.0, seq=99), now=2.0)
    d = g.decide(now=2.0, now_wall=2.0)
    assert d.action == gc.STOP


def test_dwell_continuity_resets_on_nonclear_between_ticks():
    # regression: continuity must be observed on EVERY received message, not only
    # at decide ticks. When the camera publishes faster than the gate decides, a
    # single non-clear frame arriving and being overwritten by a CLEAR before the
    # next tick must still reset the dwell — otherwise RUN is granted early.
    g = gc.HumanGate(cfg(clear_dwell_s=1.0))
    t, seq = 0.0, 1
    while t < 0.95:                       # accumulate ~0.9 s of clear dwell
        g.on_state(clear(stamp=t, seq=seq), now=t, now_wall=t)
        d = g.decide(now=t, now_wall=t)
        t = round(t + 0.1, 9)
        seq += 1
    assert d.action == gc.STOP            # not yet at the 1.0 s dwell
    # two messages arrive before the next decide tick: a non-clear frame, then a
    # CLEAR that overwrites it. The gate never decides on the SLOW frame.
    g.on_state(zone(ZONE_SLOW, stamp=1.0, dist=2.0, seq=seq), now=1.0, now_wall=1.0)
    seq += 1
    g.on_state(clear(stamp=1.02, seq=seq), now=1.02, now_wall=1.02)
    # next tick lands past the original dwell target; had the SLOW frame not reset
    # the timer, dwell would read >1.0 s and this would RUN.
    d = g.decide(now=1.05, now_wall=1.05)
    assert d.action == gc.STOP, "an intermediate non-clear message must reset the dwell"
    assert d.clear_dwell_s < 1.0


# --- rule: resume refused when person LOST close (not tracked leaving) ------

def test_lost_close_refuses_resume_through_full_dwell():
    g = gc.HumanGate(cfg(clear_dwell_s=1.0))
    # SLOW(close) -> STOP(close) -> CLEAR directly (track vanished while close)
    g.on_state(zone(ZONE_SLOW, stamp=0.0, dist=1.5, seq=1), now=0.0)
    g.decide(0.0, 0.0)
    g.on_state(zone(ZONE_STOP, stamp=0.1, dist=0.8, seq=2), now=0.1)
    g.decide(0.1, 0.1)
    # now CLEAR for a long time; must never RUN
    t = 0.2
    seq = 3
    ran = False
    while t < 5.0:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t)
        ran = ran or d.action == gc.RUN
        assert d.provenance == gc.PROV_LOST_CLOSE
        t = round(t + 0.1, 9)
        seq += 1
    assert not ran
    assert d.action == gc.STOP


def test_track_lost_close_flag_refuses_resume():
    g = gc.HumanGate(cfg(clear_dwell_s=1.0))
    # STOP frame carries the explicit TRACK_LOST_CLOSE flag, then CLEAR
    g.on_state(zone(ZONE_STOP, stamp=0.0, dist=0.9, seq=1,
                    flags=(HEALTH_TRACK_LOST_CLOSE,)), now=0.0)
    g.decide(0.0, 0.0)
    t, seq, ran = 0.1, 2, False
    while t < 3.0:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t)
        ran = ran or d.action == gc.RUN
        t = round(t + 0.1, 9)
        seq += 1
    assert not ran and d.provenance == gc.PROV_LOST_CLOSE


def test_tracked_leaving_allows_resume():
    g = gc.HumanGate(cfg(clear_dwell_s=1.0))
    # approach then a TRACKED departure: STOP -> SLOW -> CLEAR
    g.on_state(zone(ZONE_STOP, stamp=0.0, dist=0.9, seq=1), now=0.0)
    g.decide(0.0, 0.0)
    g.on_state(zone(ZONE_SLOW, stamp=0.1, dist=2.4, seq=2), now=0.1)
    g.decide(0.1, 0.1)
    t, seq, ran = 0.2, 3, False
    while t < 2.0:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t)
        ran = ran or d.action == gc.RUN
        t = round(t + 0.1, 9)
        seq += 1
    assert ran, "a tracked departure + dwell should allow RUN"
    assert d.provenance == gc.PROV_TRACKED_LEAVING


def test_lost_close_recovers_after_new_tracked_pass():
    g = gc.HumanGate(cfg(clear_dwell_s=0.5))
    # lost close
    g.on_state(zone(ZONE_STOP, stamp=0.0, dist=0.8, seq=1), now=0.0)
    g.decide(0.0, 0.0)
    g.on_state(clear(stamp=0.1, seq=2), now=0.1)
    d = g.decide(0.1, 0.1)
    assert d.provenance == gc.PROV_LOST_CLOSE and d.action == gc.STOP
    # a new person walks cleanly in and out: SLOW -> CLEAR (tracked leaving)
    g.on_state(zone(ZONE_SLOW, stamp=0.2, dist=2.3, seq=3), now=0.2)
    g.decide(0.2, 0.2)
    t, seq, ran = 0.3, 4, False
    while t < 1.5:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t)
        ran = ran or d.action == gc.RUN
        t = round(t + 0.1, 9)
        seq += 1
    assert ran and d.provenance == gc.PROV_TRACKED_LEAVING


def test_lost_close_flag_recovers_after_new_tracked_pass():
    # regression: a lost-close FLAG episode must not latch LOST_CLOSE forever.
    g = gc.HumanGate(cfg(clear_dwell_s=0.5))
    # a SLOW frame carries the explicit lost-close flag, then CLEAR: even though
    # the last non-clear zone was SLOW, the flag makes this LOST_CLOSE.
    g.on_state(zone(ZONE_SLOW, stamp=0.0, dist=1.5, seq=1,
                    flags=(HEALTH_TRACK_LOST_CLOSE,)), now=0.0)
    g.decide(0.0, 0.0)
    g.on_state(clear(stamp=0.1, seq=2), now=0.1)
    d = g.decide(0.1, 0.1)
    assert d.provenance == gc.PROV_LOST_CLOSE and d.action == gc.STOP
    # a new person walks cleanly in and out: SLOW -> CLEAR (tracked leaving); the
    # flag must NOT persist into this fresh episode, so resume is allowed.
    g.on_state(zone(ZONE_SLOW, stamp=0.2, dist=2.3, seq=3), now=0.2)
    g.decide(0.2, 0.2)
    t, seq, ran = 0.3, 4, False
    while t < 1.5:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t)
        ran = ran or d.action == gc.RUN
        t = round(t + 0.1, 9)
        seq += 1
    assert ran and d.provenance == gc.PROV_TRACKED_LEAVING


# --- rule: clock skew (stamp in the future) -> STOP ------------------------

def test_clock_skew_future_stamp_is_stop():
    g = gc.HumanGate(cfg())
    g.on_state(clear(stamp=1.0, seq=1), now=0.0)   # stamp 1 s ahead of wall
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.STOP and "skew" in d.reason


# --- fall-through: STOP not confirmed within deadline -> BACKSTOP ----------

def test_stop_escalates_to_backstop_when_arm_not_confirmed_stopped():
    g = gc.HumanGate(cfg(backstop_deadline_s=0.5))
    t = 0.0
    seq = 1
    saw_backstop = False
    while t < 1.2:
        g.on_state(zone(ZONE_STOP, stamp=t, dist=1.0, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t, driver_stopped=False)
        if t <= 0.5:
            assert d.action == gc.STOP
        if d.action == gc.BACKSTOP:
            saw_backstop = True
        t = round(t + 0.1, 9)
        seq += 1
    assert saw_backstop
    assert d.speed_fraction is None


def test_confirmed_stop_does_not_escalate():
    g = gc.HumanGate(cfg(backstop_deadline_s=0.5))
    t, seq = 0.0, 1
    while t < 1.5:
        g.on_state(zone(ZONE_STOP, stamp=t, dist=1.0, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t, driver_stopped=True)
        assert d.action == gc.STOP
        t = round(t + 0.1, 9)
        seq += 1


def test_unconfirmed_feedback_escalates_after_deadline():
    g = gc.HumanGate(cfg(backstop_deadline_s=0.5))
    t, seq, saw = 0.0, 1, False
    while t < 1.2:
        g.on_state(zone(ZONE_STOP, stamp=t, dist=1.0, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t, driver_stopped=None)
        saw = saw or d.action == gc.BACKSTOP
        t = round(t + 0.1, 9)
        seq += 1
    assert saw


# --- fall-through: gate self-fault -> BACKSTOP -----------------------------

def test_self_fault_forces_backstop():
    g = gc.HumanGate(cfg())
    g.on_state(zone(ZONE_SLOW, stamp=0.0, dist=2.5, seq=1), now=0.0)
    g.inject_self_fault("decide loop overran")
    d = g.decide(now=0.0, now_wall=0.0)
    assert d.action == gc.BACKSTOP and "self-fault" in d.reason


def test_backstop_de_escalates_only_through_full_clear_dwell():
    g = gc.HumanGate(cfg(clear_dwell_s=1.0))
    g.on_state(zone(ZONE_SLOW, stamp=0.0, dist=2.5, seq=1), now=0.0)
    g.inject_self_fault("boom")
    d = g.decide(0.0, 0.0)
    assert d.action == gc.BACKSTOP
    # feed healthy CLEAR; must stay BACKSTOP until the dwell completes, then RUN
    t, seq, ran = 0.1, 2, False
    while t < 2.0:
        g.on_state(clear(stamp=t, seq=seq), now=t)
        d = g.decide(now=t, now_wall=t)
        if d.action == gc.BACKSTOP:
            assert t < 1.0 + 1e-9, f"still BACKSTOP at t={t} after dwell should be done"
        if d.action == gc.RUN:
            ran = True
            # the fresh-plan note appears on the FIRST RUN after a backstop only
            assert "fresh plan" in d.reason
            break
        t = round(t + 0.1, 9)
        seq += 1
    assert ran


# --- speed fractions map to config -----------------------------------------

def test_speed_fractions_match_config():
    c = cfg(operating_speed_fraction=1.0, slow_speed_fraction=0.1,
            stop_speed_fraction=0.0, clear_dwell_s=0.5)
    g = gc.HumanGate(c)
    g.on_state(zone(ZONE_SLOW, stamp=0.0, dist=2.5, seq=1), now=0.0)
    assert g.decide(0.0, 0.0).speed_fraction == 0.1
    g.on_state(zone(ZONE_STOP, stamp=0.1, dist=1.0, seq=2), now=0.1)
    assert g.decide(0.1, 0.1).speed_fraction == 0.0


# --- property: RUN never appears without a preceding healthy CLEAR dwell ----

def test_property_no_run_without_preceding_healthy_dwell():
    random.seed(1234)
    c = cfg(clear_dwell_s=1.0, backstop_deadline_s=0.5)
    zones = [ZONE_CLEAR, ZONE_SLOW, ZONE_STOP]
    for trial in range(200):
        g = gc.HumanGate(c)
        t, seq = 0.0, 1
        for _ in range(60):
            z = random.choice(zones)
            healthy = random.random() > 0.15
            dist = None if random.random() > 0.5 else round(random.uniform(0.3, 3.0), 2)
            # random gaps to exercise heartbeat timeout
            gap = random.choice([0.1, 0.1, 0.1, 0.4])
            t = round(t + gap, 9)
            g.on_state(CameraState(stamp=t, zone=z, nearest_distance_m=dist,
                                   health_ok=healthy, heartbeat_seq=seq), now=t)
            d = g.decide(now=t, now_wall=t,
                         driver_stopped=random.choice([True, False, None]))
            if d.action == gc.RUN:
                # the public-field contract of a RUN: a full trusted CLEAR dwell
                assert d.healthy, f"trial {trial}: RUN on an unhealthy tick"
                assert d.demand == ZONE_CLEAR, f"trial {trial}: RUN without CLEAR demand"
                assert d.provenance in (gc.PROV_CLEAN_START, gc.PROV_TRACKED_LEAVING), \
                    f"trial {trial}: RUN with untrusted provenance {d.provenance}"
                assert d.clear_dwell_s >= c.clear_dwell_s - 1e-6, \
                    f"trial {trial}: RUN with dwell {d.clear_dwell_s} < {c.clear_dwell_s}"
            seq += 1


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
