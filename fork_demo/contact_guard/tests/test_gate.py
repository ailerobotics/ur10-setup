from fork_demo.contact_guard import gate, params


def _good(**kw):
    base = dict(tcp_offset=(0, -0.038, 0.427, 0, 0, 0), payload_kg=0.3, loaded=False,
               speed_frac=0.05, robot_running=True, robot_normal=True,
               program_running=True, joint_feed_live=True, free_gb=50,
               pendant_force_ack_n=100.0, align_veto_ok=True)
    base.update(kw)
    return gate.GateInput(**base)


def test_good_snapshot_passes():
    p = params.load()
    assert gate.evaluate_gate(_good(), p).ok


def test_non_fork_tcp_blocks():
    p = params.load()
    v = gate.evaluate_gate(_good(tcp_offset=(0, 0, 0.15, 0, 0, 0)), p)
    assert not v.ok and any(c.name == "fork_tcp" for c in v.blocking)


def test_missing_values_fail_toward_stop():
    p = params.load()
    v = gate.evaluate_gate(_good(tcp_offset=None, payload_kg=None, speed_frac=None,
                                 free_gb=None, align_veto_ok=None), p)
    names = {c.name for c in v.blocking}
    assert {"fork_tcp", "payload", "speed_cap", "free_disk",
            "alignment_veto"} <= names


def test_speed_cap_and_ec_and_feed():
    p = params.load()
    assert not gate.evaluate_gate(_good(speed_frac=0.2), p).ok
    assert not gate.evaluate_gate(_good(program_running=False), p).ok
    assert not gate.evaluate_gate(_good(joint_feed_live=False), p).ok


def test_pendant_ack_recorded_and_noted():
    p = params.load()
    v = gate.evaluate_gate(_good(pendant_force_ack_n=250.0), p)
    c = next(c for c in v.checks if c.name == "pendant_force_ack")
    assert c.ok and "recommends 100" in c.detail
    assert v.pendant_force_ack_n == 250.0
    # missing ack blocks
    assert not gate.evaluate_gate(_good(pendant_force_ack_n=None), p).ok


def test_loaded_payload_band():
    p = params.load()
    assert gate.evaluate_gate(_good(loaded=True, payload_kg=4.4), p).ok
    assert not gate.evaluate_gate(_good(loaded=False, payload_kg=4.4), p).ok


def test_gate_json_schema():
    p = params.load()
    j = gate.evaluate_gate(_good(), p).to_json()
    assert j["schema"] == "contact_guard.gate_verdict.v1"
    assert j["ok"] is True
