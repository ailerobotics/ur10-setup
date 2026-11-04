"""Tolerance / verdict logic + verdict-file contract -- pure, offline."""
import json
import os

from fork_demo.align_check import tolerance, verdict_io


def test_lateral_tolerance_formula():
    # (44 - 40)/2 = 2 mm half-clearance; /1.0 safety = 2 mm.
    assert abs(tolerance.lateral_tolerance_mm(0.044, 0.040, 1.0) - 2.0) < 1e-9
    # Safety factor shrinks the tolerance (more conservative).
    assert abs(tolerance.lateral_tolerance_mm(0.044, 0.040, 2.0) - 1.0) < 1e-9
    # No clearance -> zero tolerance.
    assert tolerance.lateral_tolerance_mm(0.040, 0.040, 1.0) == 0.0


def test_tune_up_never_down_monotonic():
    base = tolerance.lateral_tolerance_mm(0.044, 0.040, 1.5)
    tighter_sf = tolerance.lateral_tolerance_mm(0.044, 0.040, 3.0)
    narrower = tolerance.lateral_tolerance_mm(0.042, 0.040, 1.5)
    assert tighter_sf < base and narrower < base  # both only tighten


def test_shelf_verdict_go_nogo():
    kw = dict(channel_width_m=0.044, tine_width_m=0.040, safety_factor=1.5,
              yaw_tolerance_deg=3.0)
    # tol = 2/1.5 = 1.333 mm
    go = tolerance.shelf_verdict({"left": 0.5, "right": -0.4}, 1.0, **kw)
    assert go.ok and go.margin_mm > 0 and go.margin_deg > 0
    nogo_pos = tolerance.shelf_verdict({"left": 2.0, "right": 0.0}, 0.0, **kw)
    assert not nogo_pos.ok and nogo_pos.margin_mm < 0
    nogo_yaw = tolerance.shelf_verdict({"left": 0.1, "right": 0.1}, 5.0, **kw)
    assert not nogo_yaw.ok and nogo_yaw.margin_deg < 0


def test_shelf_correction_sign():
    v = tolerance.shelf_verdict({"left": 2.0, "right": 4.0}, 2.0,
                                channel_width_m=0.044, tine_width_m=0.040,
                                safety_factor=1.5, yaw_tolerance_deg=3.0)
    # mean lateral +3 mm -> correction -3 mm; yaw +2 -> correction -2.
    assert abs(v.dx_mm - 3.0) < 1e-6 and abs(v.correction_dx_mm + 3.0) < 1e-6
    assert abs(v.correction_dyaw_deg + 2.0) < 1e-6


def test_table_verdict_both_axes():
    kw = dict(channel_width_m=0.21, tine_width_m=0.20, safety_factor=1.5,
              dyaw_tolerance_deg=3.0)  # tol = (0.21-0.20)/2/1.5*1000 = 3.333 mm
    go = tolerance.table_verdict(1.0, -2.0, 1.0, **kw)
    assert go.ok
    nogo = tolerance.table_verdict(1.0, 5.0, 0.0, **kw)  # dy over tolerance
    assert not nogo.ok


def test_verdict_contract_fields(tmp_path):
    v = tolerance.shelf_verdict({"left": 0.5, "right": -0.5}, 1.0,
                                channel_width_m=0.044, tine_width_m=0.040,
                                safety_factor=1.5, yaw_tolerance_deg=3.0,
                                image_stamp="frame-1", assumptions=["a1"])
    p = os.path.join(tmp_path, "v.json")
    contract = verdict_io.write_verdict(v, p)
    for key in ("side", "ok", "dx_mm", "dy_mm", "dyaw_deg", "tolerance",
                "margin", "image_stamp", "assumptions"):
        assert key in contract, f"missing contract field {key}"
    assert "mm" in contract["tolerance"] and "deg" in contract["tolerance"]
    with open(p) as f:
        on_disk = json.load(f)
    assert on_disk == contract
    assert on_disk["correction"]["note"].startswith("PROPOSAL ONLY")
