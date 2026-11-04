"""Table side: detect AprilTag 36h11 id 1 and solve its pose.

AprilTag decoding (cv2.aruco, DICT_APRILTAG_36h11) is orientation-unambiguous,
so the tag's own frame fixes the dock orientation; the printed 'TOP' arrow is
the human-readable reference that, at print time, is aligned with the tag's +Y
(up) axis (captain convention -- documented in README, verified visually on the
guide photo by the TOP arrow and the A/B/C/D corner labels).

Tag frame: origin at the tag centre, X right, Y up, Z out of the sheet (toward
the camera). Returned pose is tag-in-camera-optical (rvec, tvec). The caller
carries it to base_link with hand-eye + FK.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import cv2.aruco as aruco
import numpy as np

_DICT = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)


@dataclass
class TagDetection:
    found: bool
    tag_id: int
    corners_px: np.ndarray | None   # (4,2) TL,TR,BR,BL as decoded
    center_px: np.ndarray | None    # (2,)
    mean_side_px: float


@dataclass
class TagPose:
    rvec: np.ndarray      # (3,) Rodrigues, tag-in-camera
    tvec: np.ndarray      # (3,) metres, tag-in-camera
    reproj_px: float      # RMS reprojection error
    distance_m: float


def _object_points(size_m: float) -> np.ndarray:
    h = size_m / 2.0
    # TL, TR, BR, BL in tag frame (X right, Y up, Z out).
    return np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64
    )


def detect_tag(bgr: np.ndarray, tag_id: int = 1) -> TagDetection:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    params = aruco.DetectorParameters_create()
    # Sub-pixel corner refinement: tightens solvePnP distance from ~0.6% to
    # ~0.1% on the synthetic fixtures (see tests/test_tag_pose.py).
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = aruco.detectMarkers(gray, _DICT, parameters=params)
    if ids is None:
        return TagDetection(False, tag_id, None, None, 0.0)
    for c, i in zip(corners, ids.ravel()):
        if int(i) == tag_id:
            pts = c.reshape(-1, 2).astype(np.float64)
            sides = [np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]
            return TagDetection(
                True, tag_id, pts, pts.mean(axis=0), float(np.mean(sides))
            )
    return TagDetection(False, tag_id, None, None, 0.0)


def solve_tag_pose(det: TagDetection, k: np.ndarray, size_m: float, dist=None) -> TagPose:
    if not det.found:
        raise ValueError("cannot solve pose: tag not found")
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
    obj = _object_points(size_m)
    ok, rvec, tvec = cv2.solvePnP(
        obj, det.corners_px, k.astype(np.float64), dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        raise RuntimeError("solvePnP failed")
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k.astype(np.float64), dist)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - det.corners_px) ** 2, axis=1))))
    return TagPose(rvec.ravel(), tvec.ravel(), reproj, float(np.linalg.norm(tvec)))


def tag_matrix_in_camera(pose: TagPose) -> np.ndarray:
    """4x4 camera_optical <- tag."""
    r, _ = cv2.Rodrigues(pose.rvec)
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = pose.tvec
    return m


def annotate(bgr: np.ndarray, det: TagDetection, k=None, size_m=0.12, pose=None) -> np.ndarray:
    out = bgr.copy()
    if not det.found:
        cv2.putText(out, "tag id 1 NOT FOUND", (40, 80), cv2.FONT_HERSHEY_SIMPLEX,
                    2.0, (0, 0, 255), 4, cv2.LINE_AA)
        return out
    pts = det.corners_px.astype(int)
    cv2.polylines(out, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 3)
    labels = ["TL", "TR", "BR", "BL"]
    for (x, y), lab in zip(pts, labels):
        cv2.circle(out, (x, y), 10, (0, 255, 0), -1)
        cv2.putText(out, lab, (x + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0), 2, cv2.LINE_AA)
    cx, cy = det.center_px.astype(int)
    cv2.drawMarker(out, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 40, 3)
    # TOP arrow reference = tag +Y = midpoint(TL,TR) direction from centre.
    top_mid = (det.corners_px[0] + det.corners_px[1]) / 2.0
    tm = top_mid.astype(int)
    cv2.arrowedLine(out, (cx, cy), (tm[0], tm[1]), (255, 0, 255), 3, tipLength=0.2)
    cv2.putText(out, "TOP(+Y)", (tm[0] + 10, tm[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (255, 0, 255), 2, cv2.LINE_AA)
    if pose is not None and k is not None:
        cv2.drawFrameAxes(out, k.astype(np.float64), np.zeros(5),
                          pose.rvec, pose.tvec, size_m * 0.5, 3)
    return out
