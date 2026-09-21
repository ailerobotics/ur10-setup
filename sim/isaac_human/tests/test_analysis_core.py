# SPDX-License-Identifier: Apache-2.0
"""Offline unit tests for analysis_core on synthetic logs: perfect tracking, a
constant offset, a missed STOP, and a late transition."""
import math

import analysis_core as ac

REACH = 1.30
STOP = 0.70
BOUNDARY = REACH + STOP     # 2.0 m
BUDGET = 0.7


def _straight_line_truth(t0=0.0, dt=0.1, y_start=-4.0, y_end=-1.0, n=31):
    """Person walking straight in along -Y from y_start to y_end (present)."""
    recs = []
    for i in range(n):
        a = i / (n - 1)
        y = y_start + a * (y_end - y_start)
        root = [0.1, y, 0.0]
        centre = [0.1, y, 0.95]
        recs.append({"t": t0 + i * dt, "present": True, "root": root,
                     "centre": centre, "dist_base": math.hypot(0.1, y)})
    return recs


def _state(t, zone, tracks=None, nd_base=None, tid=None, status="known"):
    return {"t": t, "zone": zone, "distance_status": status,
            "nearest_distance": None, "nearest_distance_base": nd_base,
            "nearest_track_id": tid, "tracks": tracks or []}


def _all_in_fov(role, centre):
    return True


# --------------------------------------------------------------------------- #
def test_boundary_crossing_inward():
    truth = ac.Truth(_straight_line_truth())
    cr = ac.boundary_crossings(truth, BOUNDARY)
    assert len(cr) == 1
    assert cr[0]["dir"] == "inward"
    # crossing distance is exactly the boundary
    tr = truth.at(cr[0]["t"])
    assert abs(tr["dist_base"] - BOUNDARY) < 0.05


def test_perfect_tracking_no_missed_stop():
    truth_recs = _straight_line_truth()
    truth = ac.Truth(truth_recs)
    states = []
    for r in truth_recs:
        d = r["dist_base"]
        zone = "STOP" if d < BOUNDARY else "SLOW"
        states.append(_state(r["t"], zone, tracks=[{"id": 1, "position": r["centre"]}],
                             nd_base=d, tid=1))
    rep = ac.build_report(states, [], truth_recs, _all_in_fov, REACH, STOP, BUDGET)
    assert rep.verdict_ok, rep.verdict
    assert rep.missed_stops == []
    # perfect position + distance error
    assert rep.errors["position_3d_error_m"]["worst"] < 1e-6
    assert rep.errors["nearest_distance_base_error_m"]["worst"] < 1e-6
    # one inward stop transition, entry latency ~0
    inward = [x for x in rep.stop_latencies if x["dir"] == "inward"]
    assert len(inward) == 1
    assert inward[0]["latency_s"] is not None
    assert inward[0]["latency_s"] <= 0.1 + 1e-9


def test_constant_offset_error():
    truth_recs = _straight_line_truth()
    truth = ac.Truth(truth_recs)
    off = 0.20
    states = []
    for r in truth_recs:
        d = r["dist_base"]
        pos = [r["centre"][0] + off, r["centre"][1], r["centre"][2]]
        # distance reported off by the same offset (person appears farther out)
        nd = d + off
        zone = "STOP" if d < BOUNDARY else "SLOW"
        states.append(_state(r["t"], zone, tracks=[{"id": 1, "position": pos}],
                             nd_base=nd, tid=1))
    rep = ac.build_report(states, [], truth_recs, _all_in_fov, REACH, STOP, BUDGET)
    assert abs(rep.errors["position_3d_error_m"]["median"] - off) < 1e-6
    assert abs(rep.errors["nearest_distance_base_error_m"]["median"] - off) < 1e-6
    # zone still driven by TRUE distance in this synthetic -> no miss
    assert rep.missed_stops == []


def test_missed_stop_detected_loudly():
    # Person walks inside the STOP ring but the state stays SLOW the whole time.
    truth_recs = _straight_line_truth(y_start=-2.5, y_end=-1.0, n=21)  # ends deep inside
    states = []
    for r in truth_recs:
        # BUG: never publishes STOP (e.g. detector blind) -> SLOW throughout.
        states.append(_state(r["t"], "SLOW",
                             tracks=[{"id": 1, "position": r["centre"]}],
                             nd_base=r["dist_base"], tid=1))
    rep = ac.build_report(states, [], truth_recs, _all_in_fov, REACH, STOP, BUDGET)
    assert not rep.verdict_ok
    assert rep.verdict.startswith("FAIL")
    assert len(rep.missed_stops) == 1
    iv = rep.missed_stops[0]
    assert iv["duration"] > BUDGET
    assert iv["min_true_dist"] < BOUNDARY


def test_missed_stop_uses_nearest_body_when_present():
    # Root is OUTSIDE the ring the whole time, but the nearest body point is
    # INSIDE — a distance-biased state reads the (farther) centre and stays SLOW.
    # The nearest-body truth must catch this as a missed STOP.
    recs = []
    for i in range(20):
        t = 100.0 + i * 0.1
        root_d = 2.3                      # root always outside the 2.0 m ring
        near_d = 1.5                      # but a hand/torso is inside
        recs.append({"t": t, "present": True, "root": [0.1, -2.3, 0.0],
                     "centre": [0.1, -2.3, 0.9], "dist_base": root_d,
                     "dist_base_nearest": near_d})
    states = [_state(r["t"], "SLOW", tracks=[{"id": 1, "position": [0.1, -2.6, 0.9]}],
                     nd_base=2.6, tid=1) for r in recs]
    rep = ac.build_report(states, [], recs, _all_in_fov, REACH, STOP, BUDGET)
    assert not rep.verdict_ok
    assert len(rep.missed_stops) == 1
    assert abs(rep.missed_stops[0]["min_true_dist"] - 1.5) < 1e-9


def test_late_transition_latency():
    # Person crosses the ring at t=1.0; STOP is published 0.5 s late.
    truth_recs = _straight_line_truth(t0=0.0, dt=0.1, y_start=-3.0, y_end=-1.0, n=21)
    truth = ac.Truth(truth_recs)
    cr = ac.boundary_crossings(truth, BOUNDARY)[0]
    states = []
    for r in truth_recs:
        # STOP only once we are 0.5 s past the true crossing.
        zone = "STOP" if r["t"] >= cr["t"] + 0.5 else "SLOW"
        states.append(_state(r["t"], zone,
                             tracks=[{"id": 1, "position": r["centre"]}],
                             nd_base=r["dist_base"], tid=1))
    rep = ac.build_report(states, [], truth_recs, _all_in_fov, REACH, STOP, BUDGET)
    inward = [x for x in rep.stop_latencies if x["dir"] == "inward"][0]
    assert inward["latency_s"] is not None
    assert 0.4 < inward["latency_s"] < 0.75
    # the 0.5 s lag while inside the ring exceeds the 0.7 s budget? No (0.5<0.7),
    # so it is a late-but-acceptable transition, not a missed STOP.
    assert rep.missed_stops == []


def test_detection_rate_counts_only_in_view():
    truth_recs = _straight_line_truth()
    dets = []
    for r in truth_recs:
        # shelf always sees + fires; table is "out of view" (in_fov_fn False)
        dets.append({"t": r["t"], "camera": "shelf", "n_persons": 1, "positioned": True})
        dets.append({"t": r["t"], "camera": "table", "n_persons": 0, "positioned": False})

    def fov(role, centre):
        return role == "shelf"
    rep = ac.build_report([], dets, truth_recs, fov, REACH, STOP, BUDGET)
    assert rep.detection_rates["shelf"]["rate"] == 1.0
    # table never in view -> no opportunities recorded at all
    assert "table" not in rep.detection_rates


def test_latency_stats_and_zone_breakdown():
    states = [
        {"t": 10.0, "evidence_stamp": 9.7, "zone": "SLOW"},   # 0.3 s ok
        {"t": 10.1, "evidence_stamp": 9.2, "zone": "STOP"},   # 0.9 s over
        {"t": 10.2, "evidence_stamp": None, "zone": "CLEAR"}, # no evidence -> skipped
    ]
    ls = ac.latency_stats(states, budget_s=0.7)
    assert ls["summary"]["n"] == 2
    assert ls["n_over_budget"] == 1
    assert abs(ls["frac_over_budget"] - 0.5) < 1e-9
    zb = ac.zone_breakdown(states)
    assert zb == {"SLOW": 1, "STOP": 1, "CLEAR": 1}


def test_summarize_percentiles():
    s = ac.summarize([1.0, 2.0, 3.0, 4.0])
    assert s["median"] == 2.5
    assert s["worst"] == 4.0
    assert s["n"] == 4


def _healthy_states(n=20, over=2):
    """n localised states across CLEAR/SLOW/STOP; `over` of them over budget."""
    zones = ["CLEAR", "SLOW", "STOP"]
    states = []
    for i in range(n):
        lat = 0.9 if i < over else 0.3
        states.append({"t": 100.0 + i, "evidence_stamp": 100.0 + i - lat,
                       "zone": zones[i % 3]})
    return states


def test_record_health_ok_on_varied_healthy_run():
    h = ac.record_health(_healthy_states(n=20, over=2), budget_s=0.7)
    assert h["ok"] is True
    assert h["distinct_zones"] == 3
    assert h["reasons"] == []
    assert abs(h["latency"]["frac_over_budget"] - 0.1) < 1e-9


def test_record_health_fails_when_mostly_over_budget():
    # 12 of 20 states over the 0.70 s budget (60% > 20%).
    h = ac.record_health(_healthy_states(n=20, over=12), budget_s=0.7)
    assert h["ok"] is False
    assert any("reaction budget" in r for r in h["reasons"])


def test_record_health_fails_on_single_zone():
    states = [{"t": 100.0 + i, "evidence_stamp": 100.0 + i - 0.3, "zone": "STOP"}
              for i in range(20)]
    h = ac.record_health(states, budget_s=0.7)
    assert h["ok"] is False
    assert h["distinct_zones"] == 1
    assert any("distinct zone" in r for r in h["reasons"])


def test_record_health_ignores_none_zone_for_distinct_count():
    states = ([{"t": 100.0 + i, "evidence_stamp": 100.0 + i - 0.3, "zone": None}
               for i in range(5)]
              + [{"t": 200.0 + i, "evidence_stamp": 200.0 + i - 0.3, "zone": "SLOW"}
                 for i in range(5)]
              + [{"t": 300.0 + i, "evidence_stamp": 300.0 + i - 0.3, "zone": "STOP"}
                 for i in range(5)])
    h = ac.record_health(states, budget_s=0.7)
    assert h["distinct_zones"] == 2
    assert None not in h["zone_breakdown"]
    assert h["ok"] is True


def test_record_health_thresholds_are_configurable():
    states = _healthy_states(n=20, over=2)   # 10% over budget, 3 zones
    strict = ac.record_health(states, budget_s=0.7, max_frac_over_budget=0.05)
    assert strict["ok"] is False
    lenient = ac.record_health(states, budget_s=0.7, min_distinct_zones=4)
    assert lenient["ok"] is False
