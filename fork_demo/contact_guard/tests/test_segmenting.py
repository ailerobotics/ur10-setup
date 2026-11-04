import numpy as np

from fork_demo.contact_guard import params
from fork_demo.contact_guard.segmenting import segment_leg


def _straight_leg(length_m=0.20, n=8):
    # a straight 200 mm insert along z, few coarse waypoints
    jp = np.linspace([0, 0, 0, 0, 0, 0], [0.4, 0, 0, 0, 0, 0], n)
    tcp = np.linspace([0.6, 0.3, 0.8], [0.6, 0.3, 0.8 + length_m], n)
    return jp, tcp


def test_contact_leg_split_into_short_dense_segments():
    p = params.load()
    jp, tcp = _straight_leg(0.20)
    plan = segment_leg("insert", True, jp, tcp, p)
    # 200 mm / 15 mm -> ceil = 14 segments
    assert plan.n_segments == int(np.ceil(0.20 / p.segment_m))
    for s in plan.segments:
        assert s.probe is True
        assert s.length_m <= p.segment_m + 1e-6
        assert s.n_points >= p.segment_min_points    # densified (132-pt lesson)


def test_transit_leg_single_segment_no_probe():
    p = params.load()
    jp, tcp = _straight_leg(0.20)
    plan = segment_leg("carry", False, jp, tcp, p)
    assert plan.n_segments == 1
    assert plan.segments[0].probe is False
    assert plan.added_cycle_time_s() == 0.0


def test_added_cycle_time_is_one_dwell_per_segment():
    p = params.load()
    jp, tcp = _straight_leg(0.20)
    plan = segment_leg("insert", True, jp, tcp, p)
    assert abs(plan.added_cycle_time_s()
               - plan.n_segments * p.probe_dwell_s) < 1e-9
