# SPDX-License-Identifier: Apache-2.0
"""Load fixed-camera extrinsics for the camera side, honouring the calibration
guard so an uncalibrated / stale / moved camera stays PRESENCE ONLY.

``camera_poses.yaml`` lists the fixed cameras. Each entry points at a calibration
file written by ``cams/cellmap/calibrate_static_camera.py`` (keyed BY SERIAL, the
identity anchor — never by the provisional shelf/table role). This loader returns
a static transform for an entry ONLY when the calibration is valid:

  * the entry is ``calibrated: true`` and its ``serial`` matches the calibration
    file's ``serial`` (a swapped camera won't silently inherit another's pose);
  * the calibration file itself passed (``calibrated: true`` / ``verdict.pass``);
  * it is not older than ``max_age_days`` (if set);
  * its ``recheck`` (from ``recheck_static_camera.py``) is not ``fail``, and — if
    ``require_recheck: true`` — a fresh passing recheck exists.

Any failure drops the camera to presence-only (no TF published), EXACTLY as an
uncalibrated camera behaves today. Pure: yaml + datetime only, so the launch and
the offline tests share one code path.
"""
from __future__ import annotations

import datetime as dt
import os
from dataclasses import dataclass
from typing import List, Optional

import yaml


@dataclass
class StaticCameraTransform:
    role: str
    serial: str
    parent: str
    child: str
    translation: list          # [x, y, z]
    rotation: list             # [qx, qy, qz, qw]
    source: str


def _age_days(iso: Optional[str], now: dt.datetime) -> Optional[float]:
    if not iso:
        return None
    try:
        t = dt.datetime.fromisoformat(iso)
    except ValueError:
        return None
    return (now - t).total_seconds() / 86400.0


def _resolve(entry, poses_dir, now):
    """Return (transform_or_None, reason). reason explains a rejection (for logs)."""
    role = entry.get("_role", "?")
    if not entry.get("calibrated"):
        return None, "entry not marked calibrated"

    # Inline transform (legacy / manual) — used as-is if present.
    if entry.get("translation") and entry.get("rotation"):
        return StaticCameraTransform(
            role=role, serial=str(entry.get("serial", "")),
            parent=entry.get("parent", "base_link"), child=entry["child"],
            translation=list(entry["translation"]), rotation=list(entry["rotation"]),
            source=str(entry.get("source", "inline"))), ""

    cf = entry.get("calibration_file")
    if not cf:
        return None, "no calibration_file and no inline transform"
    path = cf if os.path.isabs(cf) else os.path.normpath(os.path.join(poses_dir, cf))
    if not os.path.isfile(path):
        return None, f"calibration file missing: {path}"
    cal = yaml.safe_load(open(path)) or {}

    if not cal.get("calibrated", False):
        return None, "calibration file not marked calibrated (verdict failed)"
    verdict = cal.get("verdict") or {}
    if verdict and verdict.get("pass") is False:
        return None, "calibration verdict.pass is false"

    want_serial = str(entry.get("serial", ""))
    have_serial = str(cal.get("serial", ""))
    if want_serial and have_serial and want_serial != have_serial:
        return None, f"serial mismatch: entry {want_serial} != calib {have_serial}"

    max_age = entry.get("max_age_days")
    if max_age is not None:
        age = _age_days(cal.get("when"), now)
        if age is not None and age > float(max_age):
            return None, f"calibration is {age:.0f} d old (> {max_age} d)"

    recheck = cal.get("recheck")
    if recheck:
        if str(recheck.get("status")) == "fail":
            return None, "last recheck FAILED (camera may have moved)"
        rc_age = _age_days(recheck.get("when"), now)
        if max_age is not None and rc_age is not None and rc_age > float(max_age):
            return None, f"recheck is {rc_age:.0f} d old (> {max_age} d)"
    elif entry.get("require_recheck"):
        return None, "require_recheck set but no recheck has been run"

    e = cal["base_link_to_camera_optical_frame"]
    return StaticCameraTransform(
        role=role, serial=have_serial or want_serial,
        parent=e.get("parent", "base_link"),
        child=e.get("child", entry.get("child", "")),
        translation=list(e["xyz"]), rotation=list(e["quaternion_xyzw"]),
        source=os.path.basename(path)), ""


def load_static_transforms(poses_path: str, now: Optional[dt.datetime] = None
                           ) -> List[StaticCameraTransform]:
    """Return the VALID static transforms to publish. Rejected entries are simply
    absent (the camera stays presence-only)."""
    out, _ = load_with_reasons(poses_path, now)
    return out


def load_with_reasons(poses_path: str, now: Optional[dt.datetime] = None):
    """Like load_static_transforms but also returns [(role, reason)] for every
    rejected entry, so the launch can log WHY a camera is presence-only."""
    now = now or dt.datetime.now()
    p = poses_path
    if not os.path.isfile(p):
        return [], []
    data = yaml.safe_load(open(p)) or {}
    poses_dir = os.path.dirname(os.path.abspath(p))
    good: List[StaticCameraTransform] = []
    rejected = []
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
