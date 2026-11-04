from fork_demo.contact_guard import params, verify


def test_lift_confirmed():
    p = params.load()
    r = verify.verify_lift({"f_ins": 5.0}, {"f_ins": 25.0}, dz_m=0.023, p=p)
    assert r.ok and r.kind == "lift"


def test_lift_refused_when_no_rise():
    p = params.load()
    r = verify.verify_lift({"f_ins": 5.0}, {"f_ins": 25.0}, dz_m=0.002, p=p)
    assert not r.ok


def test_lift_marginal_wrench_relies_on_kinematics():
    p = params.load()
    # TCP rose but the wrench step is marginal (CB3 reality) -> still OK on dz
    r = verify.verify_lift({"f_ins": 5.0}, {"f_ins": 9.0}, dz_m=0.023, p=p)
    assert r.ok and not r.wrench_conclusive


def test_lift_refused_when_wrench_drops_conclusively():
    p = params.load()
    r = verify.verify_lift({"f_ins": 40.0}, {"f_ins": 5.0}, dz_m=0.023, p=p)
    assert not r.ok


def test_release_confirmed_and_refused():
    p = params.load()
    assert verify.verify_release({"f_ins": 25.0}, {"f_ins": 3.0}, p).ok
    assert not verify.verify_release({"f_ins": 25.0}, {"f_ins": 25.0}, p).ok
