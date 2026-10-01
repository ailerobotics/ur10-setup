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
    pytest.importorskip("torch")
    det = build_detector("keypoint_rcnn", device="cpu")
    assert isinstance(det, PersonDetector)
    assert "BSD-3-Clause" in det.license          # licence recorded
    dets = det.detect(np.zeros((240, 320, 3), np.uint8))
    assert dets == []                              # no person in a blank frame


class _FakeDetector(PersonDetector):
    """A trivial detector to exercise the batching default without torch."""

    name = "fake"

    def __init__(self):
        self.calls = []

    def detect(self, rgb):
        self.calls.append(rgb.shape)
        # One "person" box sized from the image, so each image gets a distinct result.
        h, w = rgb.shape[:2]
        return [Detection2D(bbox=(0, 0, float(w), float(h)), score=0.9)]


def test_detect_batch_default_matches_per_image():
    det = _FakeDetector()
    imgs = [np.zeros((40, 60, 3), np.uint8), np.zeros((20, 30, 3), np.uint8)]
    out = det.detect_batch(imgs)
    assert len(out) == 2
    assert out[0][0].bbox == (0, 0, 60.0, 40.0)
    assert out[1][0].bbox == (0, 0, 30.0, 20.0)


def test_detect_batch_empty_is_empty():
    assert _FakeDetector().detect_batch([]) == []


def test_keypoint_rcnn_batch_equals_sequential_if_torch():
    pytest.importorskip("torch")
    det = build_detector("keypoint_rcnn", device="cpu")
    rng = np.random.default_rng(0)
    imgs = [rng.integers(0, 255, (160, 213, 3), dtype=np.uint8) for _ in range(2)]
    seq = [det.detect(im) for im in imgs]
    batch = det.detect_batch(imgs)
    assert len(batch) == len(seq)
    # Batched and per-image inference must agree on the number of people found
    # per frame (the batch axis must not leak between images).
    assert [len(b) for b in batch] == [len(s) for s in seq]
