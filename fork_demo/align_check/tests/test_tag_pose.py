"""AprilTag detector + pose tests -- on the real guide photo (downscaled
fixture) and on synthetic scenes with known optical-frame pose."""
import cv2
import numpy as np

from conftest import FIXTURES

from fork_demo.align_check import geometry, synth, tag_detect


def test_detects_tag_id1_on_guide_photo():
    bgr = cv2.imread(str(FIXTURES / "table-tag-800.jpg"))
    assert bgr is not None
    det = tag_detect.detect_tag(bgr, tag_id=1)
    assert det.found and det.tag_id == 1
    assert det.corners_px.shape == (4, 2)
    assert det.mean_side_px > 100


def test_wrong_id_not_found():
    bgr = cv2.imread(str(FIXTURES / "table-tag-800.jpg"))
    det = tag_detect.detect_tag(bgr, tag_id=7)
    assert not det.found


def test_synthetic_camera_pose_recovery():
    """Render a tag at a known optical pose; solvePnP recovers distance + yaw."""
    k = geometry.make_K(915, 915, 640, 360)
    for dist, yaw in [(0.5, 0.0), (0.45, 10.0), (0.6, -12.0)]:
        bgr, t_gt = synth.render_tag_scene(tag_yaw_deg=yaw, distance_m=dist, K=k)
        det = tag_detect.detect_tag(bgr, 1)
        assert det.found
        pose = tag_detect.solve_tag_pose(det, k, 0.12)
        assert abs(pose.distance_m - dist) < 0.005, f"dist {dist}: {pose.distance_m}"
        assert pose.reproj_px < 1.5
        t_solved = tag_detect.tag_matrix_in_camera(pose)
        gt_yaw = geometry.yaw_about_z_deg(t_gt[:3, :3])
        sv_yaw = geometry.yaw_about_z_deg(t_solved[:3, :3])
        assert abs(geometry.signed_angle_diff_deg(gt_yaw, sv_yaw)) < 0.5


def test_full_table_pipeline_recovers_base_offset():
    """Physically consistent look-down scene: align_table recovers injected dx/dy/dyaw."""
    from fork_demo.align_check import align_check
    from fork_demo.align_check.geometry import invert_transform
    params = align_check.load_params()
    k = geometry.make_K(915, 915, 640, 360)
    place_xy = [-0.5165, -1.2173]
    t_base_opt = synth.look_down_camera(place_xy, height_m=0.6, yaw_deg=0.0)
    th = np.radians(15.0)
    r = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    t_base_tag = np.eye(4)
    t_base_tag[:3, :3] = r
    t_base_tag[:3, 3] = [place_xy[0], place_xy[1], 0.05]
    t_opt_tag = invert_transform(t_base_opt) @ t_base_tag
    bgr = synth.render_tag_at(t_opt_tag, K=k)
    det = tag_detect.detect_tag(bgr, 1)
    assert det.found
    # Recover tag-in-base and compare against a taught pose offset by a known amount.
    t_bt = t_base_opt @ tag_detect.tag_matrix_in_camera(tag_detect.solve_tag_pose(det, k, 0.12))
    tag_xy = t_bt[:2, 3]
    tag_yaw = geometry.yaw_about_z_deg(t_bt[:3, :3])
    for dx, dy, dyaw in [(0, 0, 0), (5, -3, 2)]:
        taught = {"xy": [tag_xy[0] + dx / 1000.0, tag_xy[1] + dy / 1000.0],
                  "yaw_deg": tag_yaw + dyaw}
        v, _, _ = align_check.align_table(bgr, params, K=k, t_base_optical=t_base_opt, taught=taught)
        assert abs(v.dx_mm - dx) < 1.0 and abs(v.dy_mm - dy) < 1.0 and abs(v.dyaw_deg - dyaw) < 0.5
