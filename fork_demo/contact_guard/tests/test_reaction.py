from fork_demo.contact_guard import params
from fork_demo.contact_guard.detector import Detection
from fork_demo.contact_guard.reaction import in_box, plan_reaction

BOX = (0.10, 1.35, -0.45, 1.25, -0.35, 1.00)


def _det(trigger="PROBE"):
    return Detection(leg="insert_forks", trigger=trigger, families=("P",),
                     s_at_trip=0.5, mm_at_trip=7.0, ms_at_trip=120.0)


def test_in_box():
    assert in_box((0.6, 0.3, 0.8), BOX)
    assert not in_box((0.0, 0.3, 0.8), BOX)


def test_backoff_inside_box():
    p = params.load()
    plan = plan_reaction(_det(), tcp_pos=(0.6, 0.3, 0.8),
                         leg_axis=(0, 0, 1), box=BOX, p=p, contact=True)
    assert plan.action == "BACKOFF"
    # back-off along -z by backoff_m
    assert abs((0.8 - plan.backoff_target[2]) - p.backoff_m) < 1e-9
    assert plan.speed_frac == p.backoff_speed_frac


def test_backoff_out_of_box_holds():
    p = params.load()
    tight = (0.10, 1.35, -0.45, 1.25, 0.799, 1.00)
    plan = plan_reaction(_det(), tcp_pos=(0.6, 0.3, 0.8),
                         leg_axis=(0, 0, 1), box=tight, p=p, contact=True)
    assert plan.action == "HOLD"


def test_transit_and_stale_hold():
    p = params.load()
    assert plan_reaction(_det(), (0.6, 0.3, 0.8), (0, 0, 1), BOX, p,
                         contact=False).action == "HOLD"
    assert plan_reaction(_det("STALE"), (0.6, 0.3, 0.8), (0, 0, 1), BOX, p,
                         contact=True).action == "HOLD"


def test_verdict_mentions_no_auto_retry():
    p = params.load()
    plan = plan_reaction(_det(), (0.6, 0.3, 0.8), (0, 0, 1), BOX, p, contact=True)
    assert "NOT retry automatically" in plan.verdict
    assert "CAPTAIN GO" in plan.verdict
