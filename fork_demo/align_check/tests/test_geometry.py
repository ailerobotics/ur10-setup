"""Projection / FK / comparison geometry -- pure, offline.

Covers: the UR10 measured-URDF FK validated against the REAL teach-002 pose,
projection<->backprojection round-trip, the full shelf recovery on a synthetic
wrist scene with ground truth, and the table base-frame comparison.
"""
import numpy as np

from fork_demo.align_check import align_check, geometry, synth
from fork_demo.align_check.fk_ur10 import UR10Kinematics
from fork_demo.align_check.geometry import TineGeometry


# Real teach-002 "pre_grip" dwell: joints -> recorded base_link fork pose.
PRE_GRIP_JOINTS = [-0.8907, -3.23206, 1.75973, 4.66326, 0.89565, 0.02202]
PRE_GRIP_FORK_BASE = [-0.2154, 1.0467, 0.8126]
FORK_TCP_TOOL0 = [0.0, -0.038, 0.427]


def test_fk_matches_real_teach_pose():
    """FK(pre_grip) @ fork TCP reproduces the recorded base_link fork pose < 1 mm."""
    kin = UR10Kinematics.from_urdf()
    t = kin.fk_tool0(PRE_GRIP_JOINTS)
    fork = geometry.transform_points(t, FORK_TCP_TOOL0)[0]
    err_mm = np.linalg.norm(fork - np.array(PRE_GRIP_FORK_BASE)) * 1000.0
    assert err_mm < 1.0, f"FK off by {err_mm:.3f} mm"


def test_transform_inverse_round_trip():
    t = geometry.pose_to_matrix([0.1, -0.2, 0.3], [0.1, 0.2, 0.3, 0.9])
    inv = geometry.invert_transform(t)
    assert np.allclose(t @ inv, np.eye(4), atol=1e-9)


def test_project_backproject_round_trip():
    k = geometry.make_K(915, 915, 640, 360)
    pts = np.array([[0.02, -0.03, 0.5], [-0.1, 0.05, 0.42]])
    uv, valid = geometry.project_points(k, np.eye(4), pts)
    assert valid.all()
    for i in range(len(pts)):
        back = geometry.backproject_pixel(k, uv[i], pts[i, 2])
        assert np.allclose(back, pts[i], atol=1e-9)


def test_quat_identity():
    assert np.allclose(geometry.quat_xyzw_to_matrix([0, 0, 0, 1]), np.eye(3))


def test_yaw_about_z():
    r = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)  # +90 deg
    assert abs(geometry.yaw_about_z_deg(r) - 90.0) < 1e-6


def test_signed_angle_diff_wraps():
    assert abs(geometry.signed_angle_diff_deg(179, -179) - (-2)) < 1e-9
    assert abs(geometry.signed_angle_diff_deg(10, 4) - 6) < 1e-9


def test_shelf_synthetic_recovery():
    """Inject a known lateral error + yaw into a wrist scene; align_shelf recovers it."""
    params = align_check.load_params()
    tg = align_check.tine_geometry(params)
    k = geometry.make_K(915, 915, 640, 360)
    for lat, yaw in [(0.0, 0.0), (10.0, 0.0), (-8.0, 3.0), (3.0, -2.0)]:
        bgr, depth, gt = synth.render_shelf_scene(tg, lateral_err_mm=lat, yaw_deg=yaw,
                                                  plane_z_m=0.5, K=k)
        v, _, _ = align_check.align_shelf(bgr, params, depth=depth, K=k)
        assert abs(v.dx_mm - lat) < 1.0, f"lateral {lat}: got {v.dx_mm}"
        assert abs(v.dyaw_deg - yaw) < 0.5, f"yaw {yaw}: got {v.dyaw_deg}"


def test_shelf_no_depth_uses_plane():
    """Without depth the pipeline still returns a verdict (assumed plane)."""
    params = align_check.load_params()
    tg = align_check.tine_geometry(params)
    k = geometry.make_K(915, 915, 640, 360)
    bgr, _, _ = synth.render_shelf_scene(tg, lateral_err_mm=0.0, plane_z_m=0.45, K=k)
    v, _, _ = align_check.align_shelf(bgr, params, depth=None, K=k)
    assert any("no wrist depth" in a for a in v.assumptions)


def test_compare_place_to_tag_recovers_offset():
    """Base-frame place-vs-tag comparison recovers injected dx/dy/dyaw exactly."""
    th = np.radians(15.0)
    r = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    t_base_tag = np.eye(4)
    t_base_tag[:3, :3] = r
    t_base_tag[:3, 3] = [-0.5165, -1.2173, 0.05]
    tag_xy = t_base_tag[:2, 3]
    tag_yaw = geometry.yaw_about_z_deg(r)
    for dx, dy, dyaw in [(0, 0, 0), (5, -3, 2), (-4, 6, -1.5)]:
        taught = [tag_xy[0] + dx / 1000.0, tag_xy[1] + dy / 1000.0]
        gdx, gdy, gdyaw, *_ = geometry.compare_place_to_tag(
            t_base_tag, taught, tag_yaw + dyaw, (0.0, 0.0))
        assert abs(gdx - dx) < 1e-6 and abs(gdy - dy) < 1e-6 and abs(gdyaw - dyaw) < 1e-6
