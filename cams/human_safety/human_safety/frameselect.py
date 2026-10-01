# SPDX-License-Identifier: Apache-2.0
"""Cheap per-camera change / motion score on a downscaled frame.

The efficient frame-processing path (see ``scheduler.py``) needs a way to decide,
without running the expensive neural detector, whether a camera's view has
*changed* since the last full detection. This module provides exactly that: a
dependency-free, pure-NumPy downscale + frame-difference score.

It is deliberately tiny and conservative:

* **Downscale** by integer block-mean to a thumbnail (default factor 8), so the
  score costs microseconds and ignores sensor noise / sub-pixel jitter.
* **Change score** = mean absolute difference of the two thumbnails, normalised
  to ``[0, 1]`` (``/255``). A whole person walking into view moves a large block
  of pixels and produces a large score; a static scene scores ~0.

The score only ever *raises* a camera's detection priority (triggers an earlier
full detection); it can never lower the hard ``t_max`` bound or mark a camera
fresh on its own (see ``scheduler.py``). So a miscalibrated threshold can make
the path *less efficient*, never *less safe*.

Pure and torch-free: the detector node computes the thumbnail from the raw image
it already decoded and passes the scalar score to the scheduler. Unit-tested
offline with synthetic arrays.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def downscale_gray(rgb: np.ndarray, factor: int = 8) -> np.ndarray:
    """Downscale an HxWx3 (or HxW) image to a small grayscale thumbnail by
    integer block-mean.

    Block-mean (not nearest-neighbour) is used so the thumbnail averages out
    per-pixel sensor noise — the change score then reacts to real scene motion,
    not to depth/RGB speckle. ``factor`` is clamped to ``[1, min(H, W)]``.

    Returns an ``(h, w)`` float32 array in ``[0, 255]``.
    """
    a = np.asarray(rgb)
    if a.ndim == 3:
        # Luma-ish average over channels (uniform weights keep it dependency
        # free and good enough for a motion score).
        a = a.mean(axis=2)
    a = a.astype(np.float32)
    h, w = a.shape[:2]
    factor = int(max(1, min(factor, h, w)))
    if factor == 1:
        return a
    hh, ww = h // factor, w // factor
    if hh == 0 or ww == 0:
        return a
    # Crop to a whole number of blocks, then block-mean.
    a = a[: hh * factor, : ww * factor]
    return a.reshape(hh, factor, ww, factor).mean(axis=(1, 3))


def change_score(prev_small: Optional[np.ndarray],
                 cur_small: np.ndarray) -> float:
    """Normalised mean-absolute-difference between two thumbnails, in ``[0, 1]``.

    ``prev_small`` is ``None`` on the very first frame (no baseline yet); the
    score is then ``1.0`` so the scheduler always runs a full detection on the
    first frame it sees (fail toward *looking*). If the two thumbnails differ in
    shape (resolution changed mid-stream), the score is also ``1.0``.
    """
    cur = np.asarray(cur_small, dtype=np.float32)
    if prev_small is None:
        return 1.0
    prev = np.asarray(prev_small, dtype=np.float32)
    if prev.shape != cur.shape:
        return 1.0
    return float(np.abs(cur - prev).mean() / 255.0)
