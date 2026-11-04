import pytest

from fork_demo.contact_guard import params


def test_load_defaults_valid():
    p = params.load()
    p.validate()
    assert p.segment_m == 0.015
    assert p.fork_lever_m == 0.38


def test_load_or_default_on_missing(tmp_path):
    p = params.load_or_default(str(tmp_path / "nope.yaml"))
    assert p == params.ContactGuardParams()


def test_tune_down_rejected():
    d = params.ContactGuardParams()
    # widening a band floor loosens detection -> rejected
    with pytest.raises(ValueError):
        params.with_overrides(d, floor_force_n=d.floor_force_n + 10)
    # a looser probe torque floor -> rejected
    with pytest.raises(ValueError):
        params.with_overrides(d, probe_torque_nm=d.probe_torque_nm + 1)
    # a looser stall ratio -> rejected
    with pytest.raises(ValueError):
        params.with_overrides(d, stall_ratio=d.stall_ratio + 0.1)


def test_tune_up_allowed():
    d = params.ContactGuardParams()
    tighter = params.with_overrides(d, floor_force_n=d.floor_force_n - 5,
                                    probe_torque_nm=1.0)
    assert tighter.floor_force_n < d.floor_force_n


def test_backoff_speed_not_above_contact_cap():
    d = params.ContactGuardParams()
    with pytest.raises(ValueError):
        params.with_overrides(d, backoff_speed_frac=0.5)
