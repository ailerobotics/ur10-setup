# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the fixed-camera pose loader / calibration guard
(human_safety/camera_poses.py): a valid calibration publishes a TF; a missing,
uncalibrated, stale, serial-mismatched, or failed-recheck calibration yields
presence-only. No ROS, no camera.
"""
import datetime as dt
import os

import yaml

from human_safety.camera_poses import load_static_transforms, load_with_reasons


def _calib(tmp, when="2026-09-19T12:00:00", serial="025522060953",
           calibrated=True, verdict_pass=True, recheck=None, name="calib.yaml"):
    d = {
        "calibrated": calibrated,
        "when": when,
        "serial": serial,
        "base_link_to_camera_optical_frame": {
            "parent": "base_link", "child": "table_color_optical_frame",
            "xyz": [0.1, 0.2, 0.3], "quaternion_xyzw": [0, 0, 0, 1]},
        "verdict": {"pass": verdict_pass},
    }
    if recheck is not None:
        d["recheck"] = recheck
    p = os.path.join(tmp, name)
    with open(p, "w") as f:
        yaml.safe_dump(d, f)
    return p


def _poses(tmp, entry, name="camera_poses.yaml"):
    p = os.path.join(tmp, name)
    with open(p, "w") as f:
        yaml.safe_dump({"cameras": {"table": entry}}, f)
    return p


NOW = dt.datetime(2026, 9, 20, 12, 0, 0)


def test_valid_calibration_publishes_tf(tmp_path):
    tmp = str(tmp_path)
    cf = _calib(tmp)
    poses = _poses(tmp, {"calibrated": True, "serial": "025522060953",
                         "child": "table_color_optical_frame",
                         "calibration_file": cf})
    tfs = load_static_transforms(poses, now=NOW)
    assert len(tfs) == 1
    assert tfs[0].serial == "025522060953"
    assert tfs[0].child == "table_color_optical_frame"
    assert tfs[0].translation == [0.1, 0.2, 0.3]


def test_missing_poses_file_is_presence_only(tmp_path):
    assert load_static_transforms(str(tmp_path / "nope.yaml"), now=NOW) == []


def test_missing_calibration_file_is_presence_only(tmp_path):
    poses = _poses(str(tmp_path), {"calibrated": True, "serial": "x",
                                   "child": "c", "calibration_file": "/no/such.yaml"})
    good, rej = load_with_reasons(poses, now=NOW)
    assert good == [] and "missing" in rej[0][1]


def test_uncalibrated_file_is_presence_only(tmp_path):
    tmp = str(tmp_path)
    cf = _calib(tmp, calibrated=False, verdict_pass=False)
    poses = _poses(tmp, {"calibrated": True, "serial": "025522060953",
                         "child": "table_color_optical_frame", "calibration_file": cf})
    good, rej = load_with_reasons(poses, now=NOW)
    assert good == [] and "not marked calibrated" in rej[0][1]


def test_entry_not_calibrated_is_presence_only(tmp_path):
    tmp = str(tmp_path)
    cf = _calib(tmp)
    poses = _poses(tmp, {"calibrated": False, "serial": "025522060953",
                         "child": "table_color_optical_frame", "calibration_file": cf})
    assert load_static_transforms(poses, now=NOW) == []


def test_serial_mismatch_is_presence_only(tmp_path):
    """A camera swapped into another's role must NOT inherit its pose."""
    tmp = str(tmp_path)
    cf = _calib(tmp, serial="025522060953")
    poses = _poses(tmp, {"calibrated": True, "serial": "021422061772",
                         "child": "table_color_optical_frame", "calibration_file": cf})
    good, rej = load_with_reasons(poses, now=NOW)
    assert good == [] and "serial mismatch" in rej[0][1]


def test_stale_calibration_is_presence_only(tmp_path):
    tmp = str(tmp_path)
    cf = _calib(tmp, when="2026-01-01T00:00:00")
    poses = _poses(tmp, {"calibrated": True, "serial": "025522060953",
                         "child": "table_color_optical_frame",
                         "calibration_file": cf, "max_age_days": 30})
    good, rej = load_with_reasons(poses, now=NOW)
    assert good == [] and "old" in rej[0][1]


def test_failed_recheck_is_presence_only(tmp_path):
    """The moved-camera guard: a recheck marked fail => uncalibrated."""
    tmp = str(tmp_path)
    cf = _calib(tmp, recheck={"status": "fail", "when": "2026-09-20T09:00:00",
                              "reproj_px_max": 14.2})
    poses = _poses(tmp, {"calibrated": True, "serial": "025522060953",
                         "child": "table_color_optical_frame", "calibration_file": cf})
    good, rej = load_with_reasons(poses, now=NOW)
    assert good == [] and "recheck FAILED" in rej[0][1]


def test_passing_recheck_publishes(tmp_path):
    tmp = str(tmp_path)
    cf = _calib(tmp, recheck={"status": "pass", "when": "2026-09-20T09:00:00",
                              "reproj_px_max": 1.1})
    poses = _poses(tmp, {"calibrated": True, "serial": "025522060953",
                         "child": "table_color_optical_frame",
                         "calibration_file": cf, "require_recheck": True})
    assert len(load_static_transforms(poses, now=NOW)) == 1


def test_require_recheck_without_recheck_is_presence_only(tmp_path):
    tmp = str(tmp_path)
    cf = _calib(tmp)   # no recheck block
    poses = _poses(tmp, {"calibrated": True, "serial": "025522060953",
                         "child": "table_color_optical_frame",
                         "calibration_file": cf, "require_recheck": True})
    good, rej = load_with_reasons(poses, now=NOW)
    assert good == [] and "require_recheck" in rej[0][1]


def test_inline_transform_still_supported(tmp_path):
    poses = _poses(str(tmp_path), {"calibrated": True, "serial": "manual",
                                   "parent": "base_link", "child": "c",
                                   "translation": [1, 2, 3], "rotation": [0, 0, 0, 1]})
    tfs = load_static_transforms(poses, now=NOW)
    assert len(tfs) == 1 and tfs[0].translation == [1, 2, 3]


def test_repo_camera_poses_yaml_is_presence_only_today():
    """The committed camera_poses.yaml must ship with no calibrated cameras."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    p = os.path.join(here, "params", "camera_poses.yaml")
    assert load_static_transforms(p, now=NOW) == []
