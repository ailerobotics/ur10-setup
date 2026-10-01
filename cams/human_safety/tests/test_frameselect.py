# SPDX-License-Identifier: Apache-2.0
"""Offline tests for the cheap change/motion score (no torch, no ROS)."""
import numpy as np

from human_safety.frameselect import change_score, downscale_gray


def test_downscale_shrinks_and_averages():
    img = np.full((80, 160, 3), 100, dtype=np.uint8)
    small = downscale_gray(img, factor=8)
    assert small.shape == (10, 20)
    assert np.allclose(small, 100.0)


def test_downscale_block_mean_averages_noise():
    # A checkerboard of 0/200 averages to ~100 under block-mean downscale.
    img = np.indices((16, 16)).sum(axis=0) % 2 * 200
    small = downscale_gray(img.astype(np.uint8), factor=8)
    assert small.shape == (2, 2)
    assert np.allclose(small, 100.0, atol=1.0)


def test_downscale_factor_one_is_identity_gray():
    img = np.full((4, 4, 3), 50, dtype=np.uint8)
    assert np.allclose(downscale_gray(img, factor=1), 50.0)


def test_change_score_first_frame_is_one():
    cur = downscale_gray(np.zeros((40, 40, 3), np.uint8))
    assert change_score(None, cur) == 1.0


def test_change_score_identical_is_zero():
    small = downscale_gray(np.full((40, 40, 3), 123, np.uint8))
    assert change_score(small, small) == 0.0


def test_change_score_scales_with_difference():
    a = downscale_gray(np.zeros((40, 40, 3), np.uint8))
    b = downscale_gray(np.full((40, 40, 3), 255, np.uint8))
    assert change_score(a, b) == 1.0                      # full swing -> 1.0
    half = downscale_gray(np.full((40, 40, 3), 128, np.uint8))
    assert abs(change_score(a, half) - 128 / 255) < 1e-6


def test_change_score_person_block_exceeds_quiet_threshold():
    # A quiet scene vs the same scene with a "person" (a bright block over ~15%
    # of the frame) must score well above a typical motion threshold (0.02).
    base = np.full((48, 64, 3), 40, dtype=np.uint8)
    moved = base.copy()
    moved[10:40, 20:40] = 220          # a tall block, ~20% of the frame
    a = downscale_gray(base)
    b = downscale_gray(moved)
    assert change_score(a, b) > 0.02


def test_change_score_shape_mismatch_is_one():
    a = downscale_gray(np.zeros((40, 40, 3), np.uint8))
    b = downscale_gray(np.zeros((80, 80, 3), np.uint8))
    assert change_score(a, b) == 1.0
