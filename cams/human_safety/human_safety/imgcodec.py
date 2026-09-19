# SPDX-License-Identifier: Apache-2.0
"""Minimal, dependency-free ROS ``sensor_msgs/Image`` <-> NumPy decoding.

Why not cv_bridge: the detector node runs in a virtualenv with its own torch and
a pinned NumPy. cv_bridge is a compiled C extension bound to the system NumPy
ABI, so importing it across that boundary is fragile. The encodings actually on
the RealSense/webcam contract here are few and trivial to decode by hand, so we
do, and every node uses the same codec. This keeps the venv free of any
ABI-coupled ROS C extension.
"""
from __future__ import annotations

import numpy as np


def image_to_ndarray(msg) -> np.ndarray:
    """Decode a ``sensor_msgs/Image`` to an ndarray.

    Returns HxWx3 uint8 RGB for colour encodings, HxW float32 metres for
    ``32FC1`` depth, HxW uint16 for ``16UC1`` depth. Raises on anything else.
    """
    enc = msg.encoding
    h, w = msg.height, msg.width
    buf = bytes(msg.data)

    if enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 3)
        if enc == "bgr8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)
    if enc in ("rgba8", "bgra8"):
        arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        if enc == "bgra8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)
    if enc == "mono8":
        g = np.frombuffer(buf, dtype=np.uint8).reshape(h, w)
        return np.ascontiguousarray(np.stack([g, g, g], axis=-1))
    if enc == "32FC1":
        return np.frombuffer(buf, dtype=np.float32).reshape(h, w).copy()
    if enc == "16UC1":
        return np.frombuffer(buf, dtype=np.uint16).reshape(h, w).copy()
    raise ValueError(f"unsupported image encoding: {enc!r}")


def depth_to_metres(arr: np.ndarray, encoding: str) -> np.ndarray:
    """Normalise a decoded depth array to float32 metres.

    ``32FC1`` is already metres; ``16UC1`` is RealSense millimetres. Zeros
    (invalid/holes) become NaN so downstream range filtering drops them.
    """
    if encoding == "32FC1":
        out = arr.astype(np.float32)
    elif encoding == "16UC1":
        out = arr.astype(np.float32) * 1e-3
    else:
        raise ValueError(f"not a depth encoding: {encoding!r}")
    out[out <= 0.0] = np.nan
    return out


def intrinsics_from_caminfo(msg) -> tuple:
    """(fx, fy, cx, cy) from a ``sensor_msgs/CameraInfo``."""
    k = msg.k
    return float(k[0]), float(k[4]), float(k[2]), float(k[5])
