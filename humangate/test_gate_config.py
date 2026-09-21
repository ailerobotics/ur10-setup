#!/usr/bin/env python3
"""
Offline tests for the gate threshold config — NO ROS, NO robot.

Asserts the committed gate_params.yaml matches report §3/§7 provisional defaults,
that the loader validates coherence, and that a missing/bad file falls back to
the safe built-in defaults rather than crashing (the gate must still come up
STOP-safe without its param file).

Run standalone:  python3 test_gate_config.py
Or under pytest: pytest test_gate_config.py
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_config as gcfg  # noqa: E402


def test_committed_defaults_match_report():
    c = gcfg.load()
    # report §7.2/§7.3 interface budgets
    assert c.state_max_age_s == 0.200
    assert c.heartbeat_timeout_s == 0.300
    # report §3 dwell and zone geometry
    assert c.clear_dwell_s == 2.0
    # Mirrors the camera STOP TRIGGER ring: protective 2.0 m + estimator margin
    # 0.25 m = 2.25 m (data/human-safety-stop-margin-v1).
    assert c.stop_radius_m == 2.25
    assert c.slow_clear_hysteresis_m == 0.3
    # report §2.1 lever fractions
    assert c.stop_speed_fraction == 0.0
    assert 0.0 < c.slow_speed_fraction <= 0.2
    assert c.operating_speed_fraction == 1.0


def test_validate_rejects_incoherent_fractions():
    for bad in (dict(slow_speed_fraction=0.0),          # slow not > stop
                dict(operating_speed_fraction=0.05),    # operating < slow
                dict(stop_speed_fraction=-0.1)):
        try:
            gcfg.GateConfig(**bad).validate()
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for {bad}")


def test_validate_rejects_heartbeat_shorter_than_stamp_budget():
    try:
        gcfg.GateConfig(state_max_age_s=0.5, heartbeat_timeout_s=0.2).validate()
    except ValueError:
        return
    raise AssertionError("expected ValueError when heartbeat < stamp budget")


def test_missing_file_falls_back_to_safe_defaults():
    missing = os.path.join(tempfile.gettempdir(), "no_such_gate_params_xyz.yaml")
    assert not os.path.exists(missing)
    c = gcfg.load_or_default(missing)
    assert c.stop_speed_fraction == 0.0 and c.clear_dwell_s > 0


def test_unknown_keys_ignored():
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write("clear_dwell_s: 3.0\nsome_future_key: 42\n")
        path = f.name
    try:
        c = gcfg.load(path)
        assert c.clear_dwell_s == 3.0
    finally:
        os.unlink(path)


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
