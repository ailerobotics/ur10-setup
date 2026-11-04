"""Injected-fault + named-negative tests over the committed teach-002 envelope."""
import pytest

from fork_demo.contact_guard import faults


def test_all_six_faults_detected(env, p):
    for name in faults.FAULTS:
        lb = env.leg(faults.FAULT_LEG[name])
        r = faults.inject_fault(name, lb, p)
        assert r.detected, f"{name} not detected"
        assert r.detection.trigger in ("STALL", "EARLY", "PROBE", "LOAD_STEP")


def test_impeding_faults_trip_fast_and_low_force(env, p):
    # the five motion-impeding faults trip within a short time and well below the
    # 200 N old watchdog and the 100 N pendant floor
    for name in ("insert_hit", "lift_under_post", "extract_snag",
                 "setdown_misaligned", "carry_collision"):
        r = faults.inject_fault(name, env.leg(faults.FAULT_LEG[name]), p)
        assert r.detected
        assert r.ms_after_onset <= 700.0, (name, r.ms_after_onset)
        assert r.force_mag_at_trip < 200.0
        assert r.margin_to_200n > 100.0


def test_tray_slide_detected_as_load_step(env, p):
    r = faults.inject_fault("tray_slide", env.leg("carry_to_table"), p)
    assert r.detected and r.detection.trigger == "LOAD_STEP"


def test_probe_touch_resolves_small_tip_force(env, p):
    for leg in ("insert_forks", "lift", "lower_on_table"):
        r = faults.inject_probe_touch(env.leg(leg), p, tip_force_n=6.0)
        assert r.detected and r.detection.trigger == "PROBE"
        assert abs(r.detection.tip_force_n - 6.0) < 1.5
        assert r.margin_to_pendant_n > 80.0        # far below the 100 N floor


def test_named_negatives_do_not_trip(env, p):
    assert not faults.negative_force_artifact(env.leg("to_pre_grip"), p, 427.0).detected
    assert not faults.negative_force_artifact(env.leg("insert_forks"), p, 318.0).detected
    assert not faults.negative_loaded_lift_lag(env.leg("lift"), p).detected
    assert not faults.negative_ec_drop(env.leg("insert_forks"), p).detected


def test_fault_report_summary(env, p):
    rep = faults.fault_report(env, p)
    assert rep["all_detected"] is True
    assert rep["any_negative_tripped"] is False
