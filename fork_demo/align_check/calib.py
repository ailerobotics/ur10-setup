"""Load the wrist hand-eye calibration and camera intrinsics.

Hand-eye: cams/cellmap/calib/tool0_to_camera_2026-09-03.yaml, the combined
two-tag park solve the cell trusts (same file the cell map + approach demo use).
We take ``tool0_to_camera_color_optical_frame`` -> T_tool0_optical, and invert
to T_optical_tool0 (what project_tines needs).

Intrinsics: live mode reads CameraInfo off the topic. Offline the nominal
D435i colour K from params.yaml is used (1280x720 colour profile); every
offline number that depends on it is flagged as an intrinsics assumption.
"""

from __future__ import annotations

import os

import numpy as np
import yaml

from .geometry import invert_transform, make_K, pose_to_matrix

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_HANDEYE = os.path.join(
    _REPO, "cams", "cellmap", "calib", "tool0_to_camera_2026-09-03.yaml"
)


def load_handeye(path: str = DEFAULT_HANDEYE):
    """Return (T_tool0_optical, T_optical_tool0, meta)."""
    with open(path) as f:
        data = yaml.safe_load(f)
    block = data["tool0_to_camera_color_optical_frame"]
    t_tool0_optical = pose_to_matrix(block["xyz"], block["quaternion_xyzw"])
    meta = {
        "source": path,
        "optical_frame": data.get("source", {}).get("optical_frame"),
        "calibrated": data.get("calibrated"),
    }
    return t_tool0_optical, invert_transform(t_tool0_optical), meta


def K_from_params(params: dict) -> np.ndarray:
    cam = params["camera"]["wrist"]
    return make_K(cam["fx"], cam["fy"], cam["cx"], cam["cy"])


def K_from_camera_info(info: dict) -> np.ndarray:
    """CameraInfo 'k' (row-major 3x3) -> K matrix. Live mode."""
    k = np.asarray(info["k"], dtype=float).reshape(3, 3)
    return k
