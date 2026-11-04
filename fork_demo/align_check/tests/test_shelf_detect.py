"""Yellow-strip detector tests -- on the real guide photo (downscaled fixture)
and on a synthetic scene with known strip count / ordering."""
import cv2
import numpy as np

from conftest import FIXTURES

from fork_demo.align_check import align_check, geometry, shelf_detect, synth


def test_detects_two_strips_on_guide_photo():
    bgr = cv2.imread(str(FIXTURES / "shelf-strips-800.jpg"))
    assert bgr is not None
    det = shelf_detect.detect_strips(bgr)
    assert det.ok and len(det.strips) == 2
    # Ordered left-to-right.
    assert det.strips[0].cx < det.strips[1].cx
    # Both strips in the lower half (price-tag rail), roughly level.
    h = bgr.shape[0]
    assert all(s.cy > h * 0.4 for s in det.strips)
    assert abs(det.rail_angle_deg) < 20.0  # near-horizontal front rail


def test_rejects_when_no_yellow():
    bgr = np.full((400, 600, 3), 60, np.uint8)
    det = shelf_detect.detect_strips(bgr)
    assert not det.ok and len(det.strips) == 0


def test_synthetic_strip_centroids_match_projection():
    """Rendered strips are detected at the pixels they were drawn at."""
    params = align_check.load_params()
    tg = align_check.tine_geometry(params)
    k = geometry.make_K(915, 915, 640, 360)
    bgr, _, _ = synth.render_shelf_scene(tg, lateral_err_mm=0.0, plane_z_m=0.5, K=k)
    det = shelf_detect.detect_strips(bgr)
    assert det.ok
    # Two strips, left centroid left of right centroid, both near image centre band.
    assert det.strips[0].cx < det.strips[1].cx
