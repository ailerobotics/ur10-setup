# SPDX-License-Identifier: Apache-2.0
"""Detector module tests. The base class + registry import with no torch (so
this runs in the offline suite); the actual inference test is torch-gated and
skips when torch is absent (the offline suite runs under system python, which
has no torch — the live detector check runs in the venv via
tools/detector_selfcheck.py)."""
import numpy as np
import pytest

from human_safety.detector import PersonDetector, build_detector
from human_safety.messages import Detection2D


def test_registry_rejects_unknown():
    with pytest.raises(ValueError):
        build_detector("does_not_exist")


def test_detection2d_center_and_foot():
    d = Detection2D(bbox=(10, 20, 30, 80), score=0.9, keypoints=None)
    assert d.bbox_center() == (20.0, 50.0)


def test_keypoint_rcnn_on_blank_image_if_torch():
    torch = pytest.importorskip("torch")
    det = build_detector("keypoint_rcnn", device="cpu")
    assert isinstance(det, PersonDetector)
    assert "BSD-3-Clause" in det.license          # licence recorded
    dets = det.detect(np.zeros((240, 320, 3), np.uint8))
    assert dets == []                              # no person in a blank frame
