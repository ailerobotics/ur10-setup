# SPDX-License-Identifier: Apache-2.0
"""Fixed-camera extrinsics for the dynamic layer, with the SAME calibration guard
the camera side uses (``cams/human_safety/human_safety/camera_poses.py``).

The two fixed D415s are not calibrated into ``base_link`` yet (the tool is merged
in ``cams/cellmap/calibrate_static_camera.py`` but the owner has not placed the
tags). So this loader has two, mutually exclusive, modes per camera entry:

  * SIMULATION (``mode: sim``) — returns a **clearly labelled provisional**
    extrinsic (``source`` begins ``PROVISIONAL-SIM``) for replaying recorded or
    synthetic depth into nvblox. It is NEVER a metric truth and must not be used
    to place a real obstacle relative to the arm.
  * REAL (``mode: real``) — returns a transform ONLY when a valid calibration
    file written by ``calibrate_static_camera.py`` exists and passes exactly the
    same checks as the camera side: ``calibrated: true``, serial match, not older
    than ``max_age_days``, and ``recheck`` not failed. Any failure -> no transform
    (the camera is refused), so live depth from an uncalibrated camera can never
    be integrated into a map the planner trusts.

Pure: yaml + datetime only, so the launch and the offline tests share one path.
"""
from __future__ import annotations

import datetime as dt
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import yaml

MODE_SIM = "sim"
MODE_REAL = "real"


@dataclass
class CameraExtrinsic:
    role: str
    serial: str
    parent: str
    child: str
    translation: list          # [x, y, z]
    rotation: list             # [qx, qy, qz, qw]
    source: str                # "PROVISIONAL-SIM:..." or a calibration filename
    provisional: bool          # True => sim only, not a metric truth


def _age_days(iso: Optional[str], now: dt.datetime) -> Optional[float]:
    if not iso:
        return None
    try:
        t = dt.datetime.fromisoformat(iso)
    except ValueError:
        return None
    return (now - t).total_seconds() / 86400.0


def _resolve_real(entry, serial_want, poses_dir, now) -> Tuple[Optional[CameraExtrinsic], str]:
    """Real mode: require a passing calibration file (same rules as the camera
    side). Returns (extrinsic_or_None, rejection_reason)."""
    cf = entry.get("calibration_file")
    if not cf:
        return None, "real mode but no calibration_file (camera not calibrated)"
    path = cf if os.path.isabs(cf) else os.path.normpath(os.path.join(poses_dir, cf))
    if not os.path.isfile(path):
        return None, f"calibration file missing: {path} (place the tags and calibrate)"
    cal = yaml.safe_load(open(path)) or {}
    if not cal.get("calibrated", False):
        return None, "calibration file not marked calibrated (verdict failed)"
    verdict = cal.get("verdict") or {}
    if verdict and verdict.get("pass") is False:
        return None, "calibration verdict.pass is false"
    have_serial = str(cal.get("serial", ""))
    if serial_want and have_serial and serial_want != have_serial:
        return None, f"serial mismatch: entry {serial_want} != calib {have_serial}"
    max_age = entry.get("max_age_days")
    if max_age is not None:
        age = _age_days(cal.get("when"), now)
        if age is not None and age > float(max_age):
            return None, f"calibration is {age:.0f} d old (> {max_age} d)"
    recheck = cal.get("recheck")
    if recheck and str(recheck.get("status")) == "fail":
        return None, "last recheck FAILED (camera may have moved)"
    elif not recheck and entry.get("require_recheck"):
        return None, "require_recheck set but no recheck has been run"
    e = cal["base_link_to_camera_optical_frame"]
    return CameraExtrinsic(
        role=entry["_role"], serial=have_serial or serial_want,
        parent=e.get("parent", "base_link"),
        child=e.get("child", entry.get("child", "")),
        translation=list(e["xyz"]), rotation=list(e["quaternion_xyzw"]),
        source=os.path.basename(path), provisional=False), ""


def _resolve_sim(entry) -> Tuple[Optional[CameraExtrinsic], str]:
    """Sim mode: a provisional extrinsic must be present and explicitly marked."""
    prov = entry.get("provisional_extrinsic")
    if not prov:
        return None, "sim mode but no provisional_extrinsic block"
    if not (prov.get("translation") and prov.get("rotation")):
        return None, "provisional_extrinsic missing translation/rotation"
    note = prov.get("note", "provisional simulation extrinsic — NOT a metric truth")
    return CameraExtrinsic(
        role=entry["_role"], serial=str(entry.get("serial", "")),
        parent=prov.get("parent", "base_link"),
        child=prov.get("child", entry.get("child", "")),
        translation=list(prov["translation"]), rotation=list(prov["rotation"]),
        source=f"PROVISIONAL-SIM: {note}", provisional=True), ""


def _resolve(entry, poses_dir, now) -> Tuple[Optional[CameraExtrinsic], str]:
    mode = str(entry.get("mode", MODE_REAL)).lower()
    serial_want = str(entry.get("serial", ""))
    if mode == MODE_SIM:
        return _resolve_sim(entry)
    if mode == MODE_REAL:
        return _resolve_real(entry, serial_want, poses_dir, now)
    return None, f"unknown mode {mode!r} (want 'sim' or 'real')"


def load_extrinsics(poses_path: str, now: Optional[dt.datetime] = None
                    ) -> Tuple[List[CameraExtrinsic], List[Tuple[str, str]]]:
    """Return (valid_extrinsics, [(role, rejection_reason)]). A rejected camera is
    simply absent — exactly how an uncalibrated camera behaves today."""
    now = now or dt.datetime.now()
    if not os.path.isfile(poses_path):
        return [], []
    data = yaml.safe_load(open(poses_path)) or {}
    poses_dir = os.path.dirname(os.path.abspath(poses_path))
    good: List[CameraExtrinsic] = []
    rejected: List[Tuple[str, str]] = []
    for role, entry in (data.get("cameras") or {}).items():
        if not entry:
            continue
        entry = dict(entry)
        entry["_role"] = role
        tf, reason = _resolve(entry, poses_dir, now)
        if tf is not None:
            good.append(tf)
        else:
            rejected.append((role, reason))
    return good, rejected
