import json

from fork_demo.contact_guard import veto


def _msg(**kw):
    base = {"schema": veto.SCHEMA, "stamp": 100.0, "leg": "*", "ok": True,
            "reason": "aligned", "source": "test"}
    base.update(kw)
    return json.dumps(base)


def test_ok_verdict_does_not_block():
    v = veto.parse_veto(_msg(), now=101.0, max_age_s=5.0)
    assert v.ok and not v.blocks


def test_not_ok_blocks():
    v = veto.parse_veto(_msg(ok=False, reason="offset 20 mm"), now=101.0)
    assert v.blocks and not v.ok


def test_stale_blocks():
    v = veto.parse_veto(_msg(stamp=100.0), now=110.0, max_age_s=5.0)
    assert v.blocks and v.stale


def test_malformed_blocks():
    assert veto.parse_veto("{not json", now=1.0).blocks
    assert veto.parse_veto(json.dumps({"schema": "wrong"}), now=1.0).blocks
    assert veto.parse_veto(_msg(ok="yes"), now=1.0).blocks     # non-bool


def test_leg_mismatch_blocks():
    v = veto.parse_veto(_msg(leg="lift"), now=1.0, leg="insert_forks")
    assert v.blocks


def test_missing_file_blocks(tmp_path):
    v = veto.read_veto_file(str(tmp_path / "none.json"), now=1.0)
    assert v.blocks and v.stale
