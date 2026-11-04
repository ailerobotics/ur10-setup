import numpy as np

from fork_demo.contact_guard import envelope, params
from fork_demo.contact_guard.signals import LegTimeSeries


def _ts(n=200):
    t = np.linspace(0, 2, n)
    prog = np.linspace(0, 1, n)
    rng = np.random.default_rng(0)
    ts = LegTimeSeries(leg="demo", t=t, progress=prog)
    ts.feats = {
        "force_mag": 10 + 2 * np.sin(prog * 6) + rng.normal(0, 0.5, n),
        "f_ins": 5 + rng.normal(0, 0.3, n),
        "f_lat": 3 + rng.normal(0, 0.3, n),
        "torque_mag": 2 + rng.normal(0, 0.1, n),
        "t_pry": 1 + rng.normal(0, 0.1, n),
        "eff_max": 8 + rng.normal(0, 0.4, n),
        "tcp_vel": 0.1 + rng.normal(0, 0.01, n),
    }
    return ts


def test_band_contains_training_data():
    p = params.load()
    ts = _ts()
    lb = envelope.build_leg_band(ts, contact=True, p=p, tcp_travel_m=0.2)
    # every training sample must sit inside its own band (excursion 0)
    for signame, vals in ts.feats.items():
        for i in range(0, len(vals), 7):
            exc = envelope.band_excursion(lb, signame, ts.progress[i], vals[i])
            assert exc == 0.0, (signame, i, exc)


def test_serialize_roundtrip():
    p = params.load()
    lb = envelope.build_leg_band(_ts(), contact=True, p=p, tcp_travel_m=0.2)
    env = envelope.Envelope(legs={"demo": lb})
    env2 = envelope.Envelope.from_dict(env.to_dict())
    b1 = lb.at("force_mag", 0.5)
    b2 = env2.leg("demo").at("force_mag", 0.5)
    assert np.allclose(b1, b2, atol=1e-3)


def test_excursion_direction():
    p = params.load()
    lb = envelope.build_leg_band(_ts(), contact=True, p=p, tcp_travel_m=0.2)
    _, lo, hi = lb.at("force_mag", 0.5)
    assert envelope.band_excursion(lb, "force_mag", 0.5, hi + 100) > 1.0
    assert envelope.band_excursion(lb, "force_mag", 0.5, (lo + hi) / 2) == 0.0


def test_committed_envelope_has_legs(env):
    assert env.leg("insert_forks") is not None
    assert env.leg("insert_forks").contact is True
    assert env.leg("to_pre_grip").contact is False
