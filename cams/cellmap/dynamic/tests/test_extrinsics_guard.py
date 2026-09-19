#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the fixed-camera extrinsics guard — NO ROS.

Proves the same refuse-until-calibrated behaviour the camera side has: a sim
camera gets a clearly-labelled PROVISIONAL pose; a real camera is refused until a
passing calibration file exists, and stays refused on serial mismatch, staleness,
or a failed recheck. This is what makes "real use refuses until a valid
calibration file exists" true by construction.

Run:  python3 test_extrinsics_guard.py   |   pytest test_extrinsics_guard.py
"""
import datetime as dt
import os
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from extrinsics_guard import load_extrinsics  # noqa: E402

NOW = dt.datetime(2026, 9, 19, 12, 0, 0)


def _write(tmp, name, obj):
    p = os.path.join(tmp, name)
    with open(p, "w") as f:
        yaml.safe_dump(obj, f)
    return p


def _sim_entry():
    return {"mode": "sim", "serial": "SYNTHETIC",
            "child": "cam_optical",
            "provisional_extrinsic": {
                "parent": "base_link", "child": "cam_optical",
                "translation": [0.0, -1.6, 1.2], "rotation": [0.0, 0.0, 0.0, 1.0],
                "note": "sim only"}}


def _real_entry(**over):
    e = {"mode": "real", "serial": "SER123", "child": "cam_optical",
         "calibration_file": "calib.yaml", "max_age_days": 30}
    e.update(over)
    return e


def _calib(serial="SER123", when="2026-09-18T10:00:00", passed=True, recheck=None):
    c = {"calibrated": passed, "serial": serial, "when": when,
         "verdict": {"pass": passed},
         "base_link_to_camera_optical_frame": {
             "parent": "base_link", "child": "cam_optical",
             "xyz": [0.1, 0.2, 0.3], "quaternion_xyzw": [0, 0, 0, 1]}}
    if recheck is not None:
        c["recheck"] = recheck
    return c


def test_sim_returns_provisional():
    with tempfile.TemporaryDirectory() as tmp:
        p = _write(tmp, "poses.yaml", {"cameras": {"sim_fixed": _sim_entry()}})
        good, rej = load_extrinsics(p, NOW)
        assert len(good) == 1 and not rej
        assert good[0].provisional is True
        assert good[0].source.startswith("PROVISIONAL-SIM")


def test_real_refused_without_calibration_file():
    with tempfile.TemporaryDirectory() as tmp:
        p = _write(tmp, "poses.yaml",
                   {"cameras": {"cam": _real_entry(calibration_file="")}})
        good, rej = load_extrinsics(p, NOW)
        assert not good and len(rej) == 1
        assert "not calibrated" in rej[0][1] or "no calibration_file" in rej[0][1]


def test_real_refused_when_file_missing():
    with tempfile.TemporaryDirectory() as tmp:
        p = _write(tmp, "poses.yaml", {"cameras": {"cam": _real_entry()}})
        good, rej = load_extrinsics(p, NOW)
        assert not good and "missing" in rej[0][1]


def test_real_accepts_valid_calibration():
    with tempfile.TemporaryDirectory() as tmp:
        _write(tmp, "calib.yaml", _calib())
        p = _write(tmp, "poses.yaml", {"cameras": {"cam": _real_entry()}})
        good, rej = load_extrinsics(p, NOW)
        assert len(good) == 1 and not rej
        assert good[0].provisional is False
        assert good[0].translation == [0.1, 0.2, 0.3]


def test_real_refused_on_serial_mismatch():
    with tempfile.TemporaryDirectory() as tmp:
        _write(tmp, "calib.yaml", _calib(serial="OTHER"))
        p = _write(tmp, "poses.yaml", {"cameras": {"cam": _real_entry()}})
        good, rej = load_extrinsics(p, NOW)
        assert not good and "serial mismatch" in rej[0][1]


def test_real_refused_when_stale():
    with tempfile.TemporaryDirectory() as tmp:
        _write(tmp, "calib.yaml", _calib(when="2026-06-01T10:00:00"))
        p = _write(tmp, "poses.yaml", {"cameras": {"cam": _real_entry()}})
        good, rej = load_extrinsics(p, NOW)
        assert not good and "old" in rej[0][1]


def test_real_refused_on_failed_recheck():
    with tempfile.TemporaryDirectory() as tmp:
        _write(tmp, "calib.yaml",
               _calib(recheck={"status": "fail", "when": "2026-09-19T09:00:00"}))
        p = _write(tmp, "poses.yaml", {"cameras": {"cam": _real_entry()}})
        good, rej = load_extrinsics(p, NOW)
        assert not good and "recheck FAILED" in rej[0][1]


def test_real_refused_when_recheck_required_but_absent():
    with tempfile.TemporaryDirectory() as tmp:
        _write(tmp, "calib.yaml", _calib())
        p = _write(tmp, "poses.yaml",
                   {"cameras": {"cam": _real_entry(require_recheck=True)}})
        good, rej = load_extrinsics(p, NOW)
        assert not good and "no recheck" in rej[0][1]


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
