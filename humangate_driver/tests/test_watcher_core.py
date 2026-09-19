#!/usr/bin/env python3
"""
Offline tests for the independent backstop-watcher core (report §2, §5 #11/#12/#15).
NO ROS, NO robot.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from watcher_core import BackstopWatcher, WatcherParams  # noqa: E402

P = WatcherParams(state_heartbeat_timeout_s=0.5, gate_heartbeat_timeout_s=0.5,
                  stop_effect_deadline_s=0.7, stop_scaling_threshold=0.05,
                  startup_grace_s=1.0)


def _healthy(now):
    return dict(last_state_arrival=now, last_gate_arrival=now,
                state_zone="CLEAR", gate_action="RUN", gate_demand="CLEAR",
                observed_scaling=1.0)


def test_dwell_hold_does_not_cancel():
    # gate holds STOP while dwelling toward a resume: action STOP but demand
    # CLEAR and camera zone CLEAR. Cancelling here would kill the trajectory the
    # slider is freezing to resume -> must NOT cancel even with inert scaling.
    w = BackstopWatcher(P, start_time=0.0)
    for i in range(1, 30):
        t = 2.0 + i * 0.1
        r = w.update(t, last_state_arrival=t, last_gate_arrival=t,
                     state_zone="CLEAR", gate_action="STOP", gate_demand="CLEAR",
                     observed_scaling=1.0)
        assert r is None
    assert not w.tripped


def test_gate_demand_stop_not_effected_cancels():
    # fault-forced STOP: camera zone CLEAR but the gate DEMAND is STOP and the
    # slider is inert -> cancel.
    w = BackstopWatcher(P, start_time=0.0)
    for i in range(1, 12):
        t = 2.0 + i * 0.1
        w.update(t, last_state_arrival=t, last_gate_arrival=t,
                 state_zone="CLEAR", gate_action="STOP", gate_demand="STOP",
                 observed_scaling=1.0)
    assert w.tripped and "slider ineffective" in w.reason


def test_healthy_never_cancels():
    w = BackstopWatcher(P, start_time=0.0)
    for i in range(1, 100):
        t = i * 0.1
        assert w.update(t, **_healthy(t)) is None
    assert not w.tripped


def test_camera_heartbeat_loss_cancels():
    w = BackstopWatcher(P, start_time=0.0)
    w.update(2.0, **_healthy(2.0))
    # state stops arriving; gate still alive
    r = w.update(2.7, last_state_arrival=2.0, last_gate_arrival=2.7,
                 state_zone="CLEAR", gate_action="RUN", observed_scaling=1.0)
    assert r is not None and "camera-state heartbeat lost" in r and w.tripped


def test_gate_heartbeat_loss_cancels():
    w = BackstopWatcher(P, start_time=0.0)
    w.update(2.0, **_healthy(2.0))
    r = w.update(2.7, last_state_arrival=2.7, last_gate_arrival=2.0,
                 state_zone="CLEAR", gate_action="RUN", observed_scaling=1.0)
    assert r is not None and "gate decision heartbeat lost" in r


def test_gate_backstop_action_cancels():
    w = BackstopWatcher(P, start_time=0.0)
    r = w.update(2.0, last_state_arrival=2.0, last_gate_arrival=2.0,
                 state_zone="STOP", gate_action="BACKSTOP", observed_scaling=1.0)
    assert r is not None and "BACKSTOP" in r


def test_stop_not_effected_cancels():
    w = BackstopWatcher(P, start_time=0.0)
    # STOP demanded, scaling stays high (slider ineffective, e.g. mock).
    # Loop past the 0.7 s effect deadline.
    for i in range(1, 12):
        t = 2.0 + i * 0.1
        r = w.update(t, last_state_arrival=t, last_gate_arrival=t,
                     state_zone="STOP", gate_action="STOP", observed_scaling=1.0)
        if r:
            break
    assert w.tripped and "slider ineffective" in w.reason


def test_stop_effected_does_not_cancel():
    w = BackstopWatcher(P, start_time=0.0)
    # STOP demanded and scaling promptly drops to ~0 -> no cancel
    for i in range(1, 30):
        t = 2.0 + i * 0.1
        r = w.update(t, last_state_arrival=t, last_gate_arrival=t,
                     state_zone="STOP", gate_action="STOP", observed_scaling=0.0)
        assert r is None
    assert not w.tripped


def test_startup_grace_no_premature_cancel():
    w = BackstopWatcher(P, start_time=0.0)
    # nothing received yet, but within grace -> do not cancel
    assert w.update(0.5, last_state_arrival=None, last_gate_arrival=None,
                    state_zone=None, gate_action=None, observed_scaling=None) is None
    # past grace with still nothing -> cancel (harmless: no goal yet)...
    r = w.update(1.5, last_state_arrival=None, last_gate_arrival=None,
                 state_zone=None, gate_action=None, observed_scaling=None)
    assert r is not None
    # ...but it does NOT latch: once state + gate flow, it recovers.
    assert not w.tripped
    assert w.update(2.0, **_healthy(2.0)) is None
    assert not w.tripped


def test_latches():
    w = BackstopWatcher(P, start_time=0.0)
    w.update(2.0, **_healthy(2.0))
    w.update(2.7, last_state_arrival=2.0, last_gate_arrival=2.7,
             state_zone="CLEAR", gate_action="RUN", observed_scaling=1.0)
    assert w.tripped
    # returns to healthy, but stays tripped until reset
    assert w.update(3.0, **_healthy(3.0)) == w.reason
    w.reset()
    assert not w.tripped


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
