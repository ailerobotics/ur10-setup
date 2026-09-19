# SPDX-License-Identifier: Apache-2.0
"""Offline, deterministic tests for the fixed-camera calibration geometry
(cams/cellmap/staticcam.py): PnP pose composition, the base<-camera frame
convention, robust multi-view averaging, cross-tag agreement, and the
moved-camera re-check — all against synthetic tag views with known ground truth.
No ROS, no camera, no GPU.

Run:  python3 -m pytest cams/cellmap/tools/test_staticcam.py -q
"""
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import staticcam as sc  # noqa: E402


# A plausible D415 colour intrinsic (848x480), no distortion for exact synthesis.
K = np.array([[615.0, 0, 424.0], [0, 615.0, 240.0], [0, 0, 1.0]])
W, H = 848, 480

# Two tags flat on the table, like tags/tag_world_poses.yaml (normal +z).
TAGMAP = {
    0: sc.TagWorldPose(0, 0.12, sc.T_from_xyz_quat([0.116, -1.058, -0.057],
                                                   [0, 0, 0, 1])),
    1: sc.TagWorldPose(1, 0.12, sc.T_from_xyz_quat([-0.505, -1.058, -0.058],
                                                   [0, 0, 0, 1])),
}


def look_at_camera(eye, target, up=(0, 0, 1)):
    """Build T_base_cam for an optical-frame camera (z forward, x right, y down)
    at `eye` looking at `target`."""
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    up = np.asarray(up, float)
    x = np.cross(z, up)          # optical x (right)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)           # optical y (down)
    Rbc = np.column_stack([x, y, z])
    T = np.eye(4)
    T[:3, :3] = Rbc
    T[:3, 3] = eye
    return T


def synth_view(T_base_cam, tag_ids, tagmap=TAGMAP, noise_px=0.0, seed=0):
    """Project the given tags' corners through the GT pose into pixels."""
    rng = np.random.default_rng(seed)
    T_cam_base = sc.invert_T(T_base_cam)
    obs = []
    for tid in tag_ids:
        tw = tagmap[tid]
        px = sc.project_points(T_cam_base, sc.tag_corners_base(tw.T_base_tag, tw.size), K)
        if noise_px:
            px = px + rng.normal(0, noise_px, px.shape)
        obs.append(sc.TagObs(tid, px))
    return obs


# GT: a fixed camera up and to the side, looking down at the tags.
GT = look_at_camera([-0.2, -0.3, 1.4], [-0.2, -1.058, -0.057])


# ------------------------------------------------------------------- SE(3) math
def test_invert_T_roundtrip():
    T = GT
    I = T @ sc.invert_T(T)
    assert np.allclose(I, np.eye(4), atol=1e-9)


def test_pose_diff_zero_and_known():
    assert sc.pose_diff(GT, GT) == pytest.approx((0.0, 0.0), abs=1e-9)
    T2 = GT.copy()
    T2[:3, 3] += np.array([0.01, 0, 0])         # 10 mm
    dt_mm, dr_deg = sc.pose_diff(GT, T2)
    assert dt_mm == pytest.approx(10.0, abs=1e-6)
    assert dr_deg == pytest.approx(0.0, abs=1e-9)


def test_tag_corners_base_centre_and_size():
    tw = TAGMAP[0]
    c = sc.tag_corners_base(tw.T_base_tag, tw.size)
    assert np.allclose(c.mean(axis=0), tw.T_base_tag[:3, 3], atol=1e-9)
    # opposite corners are one diagonal (size*sqrt2) apart
    assert np.linalg.norm(c[0] - c[2]) == pytest.approx(0.12 * np.sqrt(2), abs=1e-9)


# ------------------------------------------------------- PnP pose + convention
def test_single_tag_recovers_pose():
    obs = synth_view(GT, [1])
    sol = sc.solve_camera_pose(obs, TAGMAP, K)
    dt_mm, dr_deg = sc.pose_diff(GT, sol.T_base_cam)
    assert dt_mm < 0.5 and dr_deg < 0.05
    assert sol.reproj_px_max < 0.01


def test_multi_tag_recovers_pose_and_agrees():
    obs = synth_view(GT, [0, 1])
    sol = sc.solve_camera_pose(obs, TAGMAP, K)
    dt_mm, dr_deg = sc.pose_diff(GT, sol.T_base_cam)
    assert dt_mm < 0.5 and dr_deg < 0.05
    # two perfect synthetic tags must agree essentially exactly
    assert sol.cross_tag_mm is not None and sol.cross_tag_mm < 0.5
    assert sol.n_tags == 2


def test_frame_convention_is_base_to_camera():
    """T_base_cam translation must be the camera's position IN base_link, and its
    z-axis (optical forward) must point from the camera toward the tags — proving
    we return base<-camera, not its inverse."""
    obs = synth_view(GT, [0, 1])
    sol = sc.solve_camera_pose(obs, TAGMAP, K)
    assert np.allclose(sol.T_base_cam[:3, 3], GT[:3, 3], atol=5e-4)
    fwd = sol.T_base_cam[:3, 2]
    to_tags = np.array([-0.2, -1.058, -0.057]) - sol.T_base_cam[:3, 3]
    to_tags /= np.linalg.norm(to_tags)
    assert float(fwd @ to_tags) > 0.99


def test_range_matches_geometry():
    obs = synth_view(GT, [1])
    sol = sc.solve_camera_pose(obs, TAGMAP, K)
    true_rng = np.linalg.norm(GT[:3, 3] - TAGMAP[1].T_base_tag[:3, 3])
    assert sol.range_m_per_tag[1] == pytest.approx(true_rng, abs=2e-3)


def test_unknown_tag_ignored_and_empty_raises():
    obs = synth_view(GT, [1])
    obs.append(sc.TagObs(9, np.array([[10, 10], [20, 10], [20, 20], [10, 20.0]])))
    sol = sc.solve_camera_pose(obs, TAGMAP, K)   # tag 9 not in map -> ignored
    assert sol.tag_ids == [1]
    with pytest.raises(ValueError):
        sc.solve_camera_pose([sc.TagObs(9, obs[-1].corners_px)], TAGMAP, K)


# --------------------------------------------------------- multi-view averaging
def test_average_views_reduces_noise():
    # 0.25 px corner noise ~ a RealSense colour image after AprilTag corner
    # refinement. Averaging many single-view solves must land close to GT and
    # meet the <5 px reprojection design target.
    views = []
    for i in range(30):
        obs = synth_view(GT, [0, 1], noise_px=0.25, seed=i)
        views.append(sc.solve_camera_pose(obs, TAGMAP, K))
    cal = sc.average_views(views)
    dt_mm, dr_deg = sc.pose_diff(GT, cal.T_base_cam)
    assert dt_mm < 6.0 and dr_deg < 0.3
    assert cal.n_views == 30
    assert cal.view_spread_mm > 0.0        # noise makes views disagree a little
    assert cal.reproj_px_max < 5.0         # the human-safety-s1 §6 target


def test_oblique_view_beats_frontoparallel_depth_bias():
    """Coplanar-tag PnP has a systematic depth bias that shrinks with viewing
    obliquity (documented in the calibration report). An oblique multi-tag view
    must recover the pose better than a near-fronto-parallel one at equal noise —
    the reason the tool warns on low-obliquity captures."""
    front = look_at_camera([-0.2, -0.5, 1.5], [-0.2, -1.058, -0.057])  # ~18 deg
    obliq = look_at_camera([-0.2, 0.6, 0.8], [-0.2, -1.058, -0.057])   # ~55 deg
    def solve(GTx):
        vs = [sc.solve_camera_pose(synth_view(GTx, [0, 1], noise_px=0.4, seed=i),
                                   TAGMAP, K) for i in range(30)]
        return sc.pose_diff(GTx, sc.average_views(vs).T_base_cam)[0]
    assert solve(obliq) < solve(front)
    assert solve(obliq) < 5.0


def test_average_views_empty_raises():
    with pytest.raises(ValueError):
        sc.average_views([])


# --------------------------------------------------------- moved-camera recheck
def test_recheck_passes_for_correct_pose():
    obs = synth_view(GT, [0, 1])
    res = sc.recheck_pose(GT, obs, TAGMAP, K, threshold_px=5.0)
    assert res.passed and res.reproj_px_max < 0.01


def test_recheck_fails_when_camera_moved():
    """Stored pose is GT; the camera has actually been nudged 4 cm + 3 deg, so the
    live tags no longer land where GT predicts -> recheck must fail."""
    moved = GT.copy()
    moved[:3, 3] += np.array([0.04, 0.0, 0.0])
    moved[:3, :3] = (R.from_matrix(GT[:3, :3]) *
                     R.from_euler("y", 3, degrees=True)).as_matrix()
    obs_moved = synth_view(moved, [0, 1])
    res = sc.recheck_pose(GT, obs_moved, TAGMAP, K, threshold_px=5.0)
    assert not res.passed
    assert res.reproj_px_max > 5.0
    assert "reproj" in res.reason


def test_recheck_no_known_tag_fails_closed():
    res = sc.recheck_pose(GT, [sc.TagObs(9, np.zeros((4, 2)))], TAGMAP, K)
    assert not res.passed and "no known tag" in res.reason


def test_recheck_small_wobble_still_passes():
    """A sub-threshold detection jitter must NOT flip a good camera to
    uncalibrated (no false alarms)."""
    obs = synth_view(GT, [0, 1], noise_px=0.5, seed=3)
    res = sc.recheck_pose(GT, obs, TAGMAP, K, threshold_px=5.0)
    assert res.passed


# -------------------------------------------- end-to-end with the real tag PNGs
def test_end_to_end_render_detect_solve():
    """Full pipeline against the ACTUAL printed tag assets: render the real
    36h11 PNGs as seen by a known fixed-camera pose, detect them with cv2.aruco,
    solve, average, and recover the ground-truth extrinsic. Exercises detection +
    PnP + averaging + the fixture check together, no camera."""
    import cv2
    scene = __import__("make_synthetic_tag_scene")
    tagmap = sc.load_tag_map(scene.TAGMAP)
    Kbig = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
    W2, H2 = 1280, 720
    GTv = scene.look_at([-0.2, -0.35, 0.95], [-0.2, -1.058, -0.057])

    views = []
    for i in range(10):
        img = scene.render(GTv, Kbig, W2, H2, tagmap, noise_px=0.3, seed=i)
        dets = sc.detect_tags(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        obs = [sc.TagObs(t, c) for t, c in dets.items() if t in tagmap]
        assert len(obs) == 2, f"expected both tags, got {sorted(dets)}"
        views.append(sc.solve_camera_pose(obs, tagmap, Kbig))
    cal = sc.average_views(views)
    dt_mm, dr_deg = sc.pose_diff(GTv, cal.T_base_cam)
    assert dt_mm < 8.0 and dr_deg < 0.5      # recovered the fixed-camera pose
    assert cal.reproj_px_max < 5.0           # design reprojection target
    assert cal.fixture_sep_err_mm < 10.0     # tag-separation fixture target


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # for make_synthetic_tag_scene


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
