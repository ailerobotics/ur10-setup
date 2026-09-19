# SPDX-License-Identifier: Apache-2.0
"""Pure geometry for calibrating a FIXED camera into base_link by AprilTag PnP.

No ROS, no torch, no GPU — only numpy / scipy / OpenCV (cv2 for solvePnP and
tag detection). Everything here is deterministic and unit-tested offline against
synthetic tag views with known ground truth (``tools/test_staticcam.py``); the
ROS capture shells (``calibrate_static_camera.py``, ``recheck_static_camera.py``)
are thin and call into this module.

The method (human-safety-s1 §6 / Phase 2):

  We know each printed 36h11 tag's full 6-DoF pose in base_link
  (``tags/tag_world_poses.yaml``, derived from the trusted wrist hand-eye by
  ``tools/derive_tag_world_poses.py``). A fixed camera sees one or more tags in
  its COLOUR image. PnP with the camera's colour intrinsics recovers the camera
  pose; composing with the known tag poses gives base_link -> camera_optical,
  which is published as a static TF so the camera side reports real positions.

FRAME CONVENTION. cv2.solvePnP(objectPoints_in_WORLD, imagePoints, K, D) returns
(rvec, tvec) that map a WORLD point to the CAMERA optical frame:
    X_cam = R(rvec) @ X_world + tvec
so (R, tvec) IS T_cam_base (base_link -> camera_optical). The static TF we must
publish is its inverse, T_base_cam (parent base_link, child *_color_optical_frame,
i.e. the camera optical frame expressed in base_link). ``solve_camera_pose``
returns T_base_cam; keep this convention straight — it is the single most common
place a hand-eye/PnP pipeline flips a sign.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


# ---------------------------------------------------------------- SE(3) helpers
def T_from_xyz_quat(xyz, quat_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = xyz
    return T


def xyz_quat_from_T(T) -> Tuple[list, list]:
    return ([float(v) for v in T[:3, 3]],
            [float(v) for v in R.from_matrix(T[:3, :3]).as_quat()])


def invert_T(T) -> np.ndarray:
    Ti = np.eye(4)
    Rt = T[:3, :3].T
    Ti[:3, :3] = Rt
    Ti[:3, 3] = -Rt @ T[:3, 3]
    return Ti


def rt_to_T(rvec, tvec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float))[0]
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def quat_mean(quats) -> np.ndarray:
    """Markley quaternion mean (dominant eigenvector of sum(q q^T)); the correct
    way to average rotations. Same method as tools/combine_tags.py."""
    Q = np.array([q if q[3] >= 0 else -q for q in quats], dtype=float)
    M = sum(np.outer(q, q) for q in Q)
    w, v = np.linalg.eigh(M)
    q = v[:, int(np.argmax(w))]
    return q / np.linalg.norm(q)


def pose_diff(Ta, Tb) -> Tuple[float, float]:
    """(translation mm, rotation deg) between two SE(3) poses."""
    dt_mm = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]) * 1e3)
    dR = R.from_matrix(Ta[:3, :3]).inv() * R.from_matrix(Tb[:3, :3])
    return dt_mm, float(np.degrees(dR.magnitude()))


# ------------------------------------------------------------------- tag model
def tag_object_points(size: float) -> np.ndarray:
    """The 4 tag corners in the TAG frame (centre origin, +z out of the tag),
    in the corner order cv2.aruco returns: TL, TR, BR, BL. Matches
    validate_tag1.py / cv2.aruco.detectMarkers."""
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def tag_corners_base(T_base_tag: np.ndarray, size: float) -> np.ndarray:
    """The 4 tag corners in base_link (4x3), given the tag's base_link pose."""
    obj = tag_object_points(size)
    hom = np.hstack([obj, np.ones((4, 1))])
    return (T_base_tag @ hom.T).T[:, :3]


# ---------------------------------------------------------------- tag world map
@dataclass
class TagWorldPose:
    tag_id: int
    size: float
    T_base_tag: np.ndarray


def load_tag_map(path: str) -> Dict[int, TagWorldPose]:
    import yaml
    d = yaml.safe_load(open(path)) or {}
    out: Dict[int, TagWorldPose] = {}
    for tid, e in (d.get("tags") or {}).items():
        out[int(tid)] = TagWorldPose(
            tag_id=int(tid), size=float(e["tag_size_m"]),
            T_base_tag=T_from_xyz_quat(e["position"], e["quaternion_xyzw"]))
    return out


# --------------------------------------------------------------------- project
def project_points(T_cam_base: np.ndarray, pts_base: np.ndarray,
                   K: np.ndarray, D: Optional[np.ndarray] = None) -> np.ndarray:
    """Project base_link points into the image, given base_link -> camera."""
    rvec = cv2.Rodrigues(T_cam_base[:3, :3])[0]
    tvec = T_cam_base[:3, 3]
    Dz = np.zeros(5) if D is None else np.asarray(D, float)
    px, _ = cv2.projectPoints(np.asarray(pts_base, float).reshape(-1, 1, 3),
                              rvec, tvec, np.asarray(K, float), Dz)
    return px.reshape(-1, 2)


# --------------------------------------------------------------------- detect
def _aruco_dict():
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def _aruco_params():
    # Prefer the legacy constructor: OpenCV 4.6's detectMarkers segfaults on a
    # 4.7-style DetectorParameters() object (validate_tag1.py "aruco_params").
    p = (cv2.aruco.DetectorParameters_create()
         if hasattr(cv2.aruco, "DetectorParameters_create")
         else cv2.aruco.DetectorParameters())
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return p


def detect_tags(gray: np.ndarray) -> Dict[int, np.ndarray]:
    """Detect 36h11 tags in a greyscale image -> {tag_id: 4x2 corner pixels}."""
    d, params = _aruco_dict(), _aruco_params()
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(d, params).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=params)
    out: Dict[int, np.ndarray] = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        out[int(i)] = c[0].astype(np.float64)
    return out


# ------------------------------------------------------------- single-view solve
@dataclass
class TagObs:
    tag_id: int
    corners_px: np.ndarray            # 4x2


@dataclass
class ViewSolution:
    T_base_cam: np.ndarray            # base_link -> camera optical frame
    n_tags: int
    tag_ids: List[int]
    reproj_px_per_tag: Dict[int, float]
    reproj_px_max: float
    range_m_per_tag: Dict[int, float]  # PnP range camera->tag centre
    cross_tag_mm: Optional[float]      # spread of per-tag base->cam estimates (informational)
    cross_tag_deg: Optional[float]
    # |measured tag-tag separation - known separation|, max over visible pairs.
    # A well-constrained, calibration-independent scale/geometry check.
    fixture_sep_err_mm: Optional[float] = None


def _tag_center_cam(obs: TagObs, tagmap: Dict[int, TagWorldPose], K, D) -> np.ndarray:
    """The tag centre in the CAMERA optical frame from single-tag PnP. Both
    branches of the planar two-fold ambiguity share this translation, so it (and
    the range) is trustworthy even where the rotation branch is not."""
    tw = tagmap[obs.tag_id]
    ok, _, tvec = cv2.solvePnP(tag_object_points(tw.size), obs.corners_px, K, D,
                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError(f"PnP failed for tag {obs.tag_id}")
    return np.asarray(tvec, float).ravel()


def _refine_pose_one_tag(base_corners, corners_px, K, D, seed_T_cam_base):
    """base->cam fit to ONE tag's four corners, ITERATIVE seeded from the joint
    solution so it stays on the physically-correct branch (a lone coplanar tag has
    a mirror ambiguity; the multi-tag prior resolves which branch is real). The fit
    still uses ONLY this tag's data, so the disagreement between tags is a genuine
    independent cross-check."""
    r0 = cv2.Rodrigues(seed_T_cam_base[:3, :3])[0]
    t0 = seed_T_cam_base[:3, 3].reshape(3, 1)
    ok, rvec, tvec = cv2.solvePnP(np.asarray(base_corners, float), corners_px, K, D,
                                  rvec=r0.copy(), tvec=t0.copy(),
                                  useExtrinsicGuess=True,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return invert_T(rt_to_T(rvec, tvec))


def solve_camera_pose(observations: List[TagObs],
                      tagmap: Dict[int, TagWorldPose],
                      K: np.ndarray, D: Optional[np.ndarray] = None,
                      ) -> ViewSolution:
    """Solve base_link -> camera_optical from all KNOWN tags seen in one view.

    Primary estimate = a single joint PnP over every visible known tag's four
    base_link corners (SQPNP: robust for coplanar and non-coplanar sets), using
    all tags at once. When >=2 known tags are visible it ALSO fits each tag
    independently (seeded from the joint solve to resolve the planar mirror
    branch) and reports their disagreement as the honest cross-tag accuracy
    figure — the same "two independent estimates" logic as combine_tags.py.
    """
    D = np.zeros(5) if D is None else np.asarray(D, float)
    K = np.asarray(K, float)
    known = [o for o in observations if o.tag_id in tagmap]
    if not known:
        raise ValueError("no observed tag is in the tag world map")

    obj_all, img_all = [], []
    range_per_tag: Dict[int, float] = {}
    center_cam: Dict[int, np.ndarray] = {}
    for o in known:
        tw = tagmap[o.tag_id]
        obj_all.append(tag_corners_base(tw.T_base_tag, tw.size))
        img_all.append(o.corners_px)
        center_cam[o.tag_id] = _tag_center_cam(o, tagmap, K, D)
        range_per_tag[o.tag_id] = float(np.linalg.norm(center_cam[o.tag_id]))

    obj_all = np.vstack(obj_all).astype(np.float64)
    img_all = np.vstack(img_all).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj_all, img_all, K, D,
                                  flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError("joint PnP failed")
    T_cam_base = rt_to_T(rvec, tvec)
    T_base_cam = invert_T(T_cam_base)

    per_tag_pose: Dict[int, np.ndarray] = {}
    if len(known) >= 2:
        for o in known:
            tw = tagmap[o.tag_id]
            Tp = _refine_pose_one_tag(tag_corners_base(tw.T_base_tag, tw.size),
                                      o.corners_px, K, D, T_base_cam)
            if Tp is not None:
                per_tag_pose[o.tag_id] = Tp

    # Per-tag reprojection under the JOINT solution (the number the target
    # <5 px is judged on): reproject each tag's base corners through T_cam_base.
    reproj_joint: Dict[int, float] = {}
    for o in known:
        tw = tagmap[o.tag_id]
        pred = project_points(T_cam_base, tag_corners_base(tw.T_base_tag, tw.size),
                              K, D)
        reproj_joint[o.tag_id] = float(
            np.linalg.norm(pred - o.corners_px, axis=1).mean())

    cross_mm = cross_deg = None
    if len(per_tag_pose) >= 2:
        poses = list(per_tag_pose.values())
        # spread = max pairwise disagreement of the independent per-tag estimates
        dts, drs = [], []
        for i in range(len(poses)):
            for j in range(i + 1, len(poses)):
                dt_mm, dr_deg = pose_diff(poses[i], poses[j])
                dts.append(dt_mm)
                drs.append(dr_deg)
        cross_mm, cross_deg = float(max(dts)), float(max(drs))

    # Fixture check: does the recovered tag-to-tag separation match the known
    # map separation? Well constrained (uses only the well-determined tag CENTRES),
    # calibration-independent, and the honest scale/geometry number.
    fixture_err = None
    if len(center_cam) >= 2:
        ids = list(center_cam)
        errs = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                meas = np.linalg.norm(center_cam[ids[i]] - center_cam[ids[j]])
                known_sep = np.linalg.norm(
                    tagmap[ids[i]].T_base_tag[:3, 3] - tagmap[ids[j]].T_base_tag[:3, 3])
                errs.append(abs(meas - known_sep) * 1e3)
        fixture_err = float(max(errs))

    return ViewSolution(
        T_base_cam=T_base_cam, n_tags=len(known),
        tag_ids=[o.tag_id for o in known],
        reproj_px_per_tag=reproj_joint,
        reproj_px_max=float(max(reproj_joint.values())),
        range_m_per_tag=range_per_tag,
        cross_tag_mm=cross_mm, cross_tag_deg=cross_deg,
        fixture_sep_err_mm=fixture_err)


# ------------------------------------------------------------ multi-view average
@dataclass
class Calibration:
    T_base_cam: np.ndarray
    n_views: int
    view_spread_mm: float
    view_spread_deg: float
    reproj_px_mean: float
    reproj_px_max: float
    cross_tag_mm: Optional[float]
    cross_tag_deg: Optional[float]
    fixture_sep_err_mm: Optional[float]
    tags_used: List[int]
    per_view: List[ViewSolution] = field(default_factory=list)


def average_views(views: List[ViewSolution]) -> Calibration:
    """Robustly average accepted single-view solutions into one extrinsic:
    translation median, rotation Markley quaternion mean; report the view spread
    (the stability of the estimate) and the worst reprojection / cross-tag error."""
    if not views:
        raise ValueError("no accepted views")
    Ts = [v.T_base_cam for v in views]
    pos = np.median(np.array([T[:3, 3] for T in Ts]), axis=0)
    q = quat_mean([R.from_matrix(T[:3, :3]).as_quat() for T in Ts])
    T_avg = T_from_xyz_quat(pos, q)

    dts = [pose_diff(T_avg, T)[0] for T in Ts]
    drs = [pose_diff(T_avg, T)[1] for T in Ts]
    reprojs = [r for v in views for r in v.reproj_px_per_tag.values()]
    crosses_mm = [v.cross_tag_mm for v in views if v.cross_tag_mm is not None]
    crosses_deg = [v.cross_tag_deg for v in views if v.cross_tag_deg is not None]
    fixtures = [v.fixture_sep_err_mm for v in views if v.fixture_sep_err_mm is not None]
    tags = sorted({t for v in views for t in v.tag_ids})
    return Calibration(
        T_base_cam=T_avg, n_views=len(views),
        view_spread_mm=float(np.median(dts)), view_spread_deg=float(np.median(drs)),
        reproj_px_mean=float(np.mean(reprojs)), reproj_px_max=float(np.max(reprojs)),
        cross_tag_mm=(float(np.max(crosses_mm)) if crosses_mm else None),
        cross_tag_deg=(float(np.max(crosses_deg)) if crosses_deg else None),
        fixture_sep_err_mm=(float(np.median(fixtures)) if fixtures else None),
        tags_used=tags, per_view=views)


# ---------------------------------------------------------- moved-camera recheck
@dataclass
class RecheckResult:
    passed: bool
    reproj_px_per_tag: Dict[int, float]
    reproj_px_max: float
    tags_seen: List[int]
    threshold_px: float
    reason: str = ""


def recheck_pose(T_base_cam: np.ndarray, observations: List[TagObs],
                 tagmap: Dict[int, TagWorldPose], K, D=None,
                 threshold_px: float = 5.0) -> RecheckResult:
    """The moved-camera guard: does the STORED pose still reproject the tags where
    they are seen now? Reproject each known tag's base_link corners through the
    stored extrinsic and compare to the freshly detected pixels. A camera that has
    been nudged makes this error blow past the threshold -> the calibration is no
    longer valid and the camera side must treat it as uncalibrated."""
    D = np.zeros(5) if D is None else np.asarray(D, float)
    K = np.asarray(K, float)
    T_cam_base = invert_T(T_base_cam)
    known = [o for o in observations if o.tag_id in tagmap]
    if not known:
        return RecheckResult(False, {}, float("inf"), [], threshold_px,
                             "no known tag detected")
    per_tag: Dict[int, float] = {}
    for o in known:
        tw = tagmap[o.tag_id]
        pred = project_points(T_cam_base, tag_corners_base(tw.T_base_tag, tw.size),
                              K, D)
        per_tag[o.tag_id] = float(np.linalg.norm(pred - o.corners_px, axis=1).mean())
    mx = float(max(per_tag.values()))
    passed = mx <= threshold_px
    return RecheckResult(
        passed=passed, reproj_px_per_tag=per_tag, reproj_px_max=mx,
        tags_seen=[o.tag_id for o in known], threshold_px=threshold_px,
        reason="" if passed else f"max reproj {mx:.2f} px > {threshold_px} px")
