import numpy as np

from fork_demo.contact_guard import envelope, params
from fork_demo.contact_guard.detector import ContactGuard
from fork_demo.contact_guard.signals import LegTimeSeries


def _band():
    p = params.load()
    n = 100
    ts = LegTimeSeries(leg="L", t=np.linspace(0, 1, n), progress=np.linspace(0, 1, n))
    ts.feats = {s: np.full(n, base) for s, base in (
        ("force_mag", 10.0), ("f_ins", 5.0), ("f_lat", 3.0),
        ("torque_mag", 2.0), ("t_pry", 1.0), ("eff_max", 8.0), ("tcp_vel", 0.1))}
    return envelope.build_leg_band(ts, True, p, tcp_travel_m=0.2), p


def _feat(force_mag=10, f_ins=5, track=0.02, **kw):
    f = {"force_mag": force_mag, "f_ins": f_ins, "f_lat": 3.0, "torque_mag": 2.0,
         "t_pry": 1.0, "eff_max": 8.0, "tcp_vel": 0.1, "track_err": track}
    f.update(kw)
    return f


def test_clean_no_trip():
    lb, p = _band()
    g = ContactGuard(p, lb, "L", True)
    det = None
    for i in range(50):
        s = i / 49
        det = g.update(i * 0.01, s, s, _feat()) or det
    assert det is None


def test_force_alone_never_trips():
    lb, p = _band()
    g = ContactGuard(p, lb, "L", True)
    det = None
    for i in range(80):
        s = i / 79
        # huge force but perfect tracking + progress (an estimate artefact)
        det = g.update(i * 0.01, s, s, _feat(force_mag=427, f_ins=400)) or det
    assert det is None


def test_stall_trips():
    lb, p = _band()
    g = ContactGuard(p, lb, "L", True)
    det = None
    s_act = 0.3
    for i in range(120):
        t = i * 0.01
        s_cmd = min(1.0, 0.3 + 0.01 * i)      # commanded advances
        # actual frozen, tracking grows
        track = 0.02 + 2.0 * (s_cmd - s_act)
        det = g.update(t, s_cmd, s_act, _feat(track=track)) or det
        if det:
            break
    assert det is not None and det.trigger == "STALL"


def test_stale_fails_toward_stop():
    lb, p = _band()
    g = ContactGuard(p, lb, "L", True)
    g.update(0.0, 0.0, 0.0, _feat())
    det = g.update(0.0 + p.stale_s + 0.2, 0.1, 0.1, _feat())
    assert det is not None and det.trigger == "STALE"


def test_not_executing_stands_down():
    lb, p = _band()
    g = ContactGuard(p, lb, "L", True)
    det = None
    for i in range(50):
        det = g.update(i * 0.01, 0.5, 0.3, _feat(force_mag=427), executing=False) or det
    assert det is None


def test_probe_touch_trips_and_reports_tip():
    lb, p = _band()
    g = ContactGuard(p, lb, "L", True)
    g.set_probe_baseline(_feat())              # fresh at-pose baseline
    det = None
    for i in range(30):
        s = 0.4 + 0.001 * i
        f = _feat()
        f["my"] = 6.0 * p.fork_lever_m          # 6 N tip force
        f["torque_mag"] = (f["mx"] ** 2 + f["my"] ** 2 + f.get("mz", 0) ** 2) ** 0.5 \
            if "mx" in f else f["my"]
        f.setdefault("mx", 0.0); f.setdefault("mz", 0.0)
        det = g.update(i * 0.01, s, s, f, creeping=True) or det
        if det:
            break
    assert det is not None and det.trigger == "PROBE"
    assert abs(det.tip_force_n - 6.0) < 1.0
