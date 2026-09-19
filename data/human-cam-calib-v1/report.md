# Fixed-camera → base_link calibration (human-cam-calib-v1)

Calibrate the cell's two fixed Intel RealSense D415s into the robot base frame so
the merged camera-side human detector (`cams/human_safety`) can report **where** a
person is and **how far** from the arm — not just that one is visible. This is
Phase 2 of the human-safety design (`~/firstmate/data/human-safety-s1/report.md`
§6 / §10). Perception only; the robot was never touched.

## What "detect humans in 3-D / when the safe distance is good enough" needed

`human_safety` already finds and tracks people and publishes a CLEAR/SLOW/STOP
state, but only the **wrist** camera ever had a base_link pose, and it is
FK-chained through `/joint_states` (driver down → unavailable). Neither fixed
D415 had ever been calibrated into base_link, so both were **presence-only**:
they could say "a person is visible" but every distance was unknown, and an
unknown distance is treated as ≥ SLOW, never as "far". A real 3-D position and a
real separation distance require each fixed camera to know where it sits in the
robot frame. This task provides that.

## What was built

| Piece | File |
|---|---|
| Tag world-pose map (known base_link pose of each printed tag) | `cams/cellmap/tags/tag_world_poses.yaml` (+ `tools/derive_tag_world_poses.py`) |
| Pure calibration geometry (PnP, pose composition, averaging, recheck) — unit-tested | `cams/cellmap/staticcam.py` |
| Static-camera calibrator (RealSense colour + colour-only) | `cams/cellmap/calibrate_static_camera.py` |
| One-time webcam intrinsics (for an ordinary colour camera) | `cams/cellmap/calibrate_webcam_intrinsics.py` |
| Moved-camera re-check guard | `cams/cellmap/recheck_static_camera.py` |
| Camera-side pose loader + calibration guard | `cams/human_safety/human_safety/camera_poses.py` |
| Launch wiring (publishes static TFs for valid calibrations only) | `cams/human_safety/human_safety.launch.py`, `params/camera_poses.yaml` |
| Offline deterministic tests | `cams/cellmap/tools/test_staticcam.py` (16), `cams/human_safety/tests/test_camera_poses.py` (12) |

## Method and why PnP against mapped tags

The repo already knows where the printed 36h11 tags are in base_link, to mm, and
*without needing the robot to move*: the trusted wrist hand-eye calibration
(`calib/tool0_to_camera_2026-09-03.yaml`, cross-tag 8.4 mm / 1.06°, independently
validated to 1.87 mm scale / 0.68 mm table-plane) plus its raw per-view samples.
`derive_tag_world_poses.py` carries every view through
`T_base_tag = T_base_tool · T_tool0_optical · T_cam_tag` and averages
(translation median, rotation Markley quaternion mean) to get each tag's full
6-DoF base_link pose:

| Tag | base_link position (m) | normal | per-view consistency |
|---|---|---|---|
| id0 (120 mm) | (0.116, −1.058, −0.057) | +z (≤0.4° tilt) | med 2.0 / max 4.1 mm |
| id1 (120 mm) | (−0.505, −1.058, −0.058) | +z (≤0.3° tilt) | med 2.0 / max 3.4 mm |

The id0–id1 spacing comes out **621.3 mm**, matching the captain's 622 mm tape
(1.87 mm validated scale) — an independent sanity check that the map is right.
Absolute accuracy is inherited from the wrist budget (~8 mm), not from the 2 mm
internal spread; both figures are recorded in the map's provenance. (The
`human-safety-s1` report cited "tag1 (−0.506,−1.058,−0.057)" — that is exactly
this 2026-09-03 placement; the tags were re-placed after the 08-27 sweep.)

**Why this approach.** A fixed camera has no arm motion to exploit, so hand-eye
(AX=XB) does not apply. With the tags' base_link poses known, a single
colour-image PnP recovers the camera pose directly, and composing with the known
tag pose gives base_link → camera_optical. It needs no depth (works for a
colour-only camera), reuses the exact AprilTag/PnP tooling already trusted here
(`validate_tag1.py` conventions), and validates against the design's stated
target (< 5 px reprojection, < 1 cm depth). The alternative — solving from depth
point clouds — fails for the colour-only case the captain explicitly asked to
support and is no more accurate at these ranges.

**Frame convention** (the usual sign trap): `cv2.solvePnP(objectPoints_in_base,
imagePoints)` returns base_link → camera_optical (`T_cam_base`); the published
static TF is its inverse. `staticcam.solve_camera_pose` returns `T_base_cam` and
this is asserted in the tests.

**Robustness notes** (measured, and why the tool warns / cross-checks):
- Multiple tags are solved jointly (SQPNP over all mapped corners). With ≥2 tags
  the joint solve is well constrained; a single tag from a near-fronto-parallel
  view has a two-fold planar ambiguity and a depth bias, so obliquity and ≥2 tags
  are preferred. The tool reports per-tag obliquity.
- The honest "agreement between tags" figure is the **tag-to-tag separation**
  (well constrained, calibration-independent), not the per-tag pose disagreement
  (single static-view single-tag solves are depth-weak — reported but not gated).

## Validation with numbers (offline, ground truth)

Because no live camera currently sees a mapped tag (below), accuracy is proven
against synthetic views rendered from the **real** tag PNGs at a known
ground-truth camera pose (`tools/make_synthetic_tag_scene.py` → detect → solve),
i.e. every number below has a true value to compare against:

| Metric | Result | Target |
|---|---|---|
| Reprojection (mean / max) | 0.33 / 0.58 px | < 5 px |
| Recovered pose error vs ground truth | **3.0 mm / 0.18°** | — |
| Tag-separation fixture error | 0.9 mm | < 10 mm |
| View spread (stability) | 5.7 mm / 0.26° | — |
| Obliquity effect | oblique view ≈ 1–2 mm vs fronto-parallel ≈ 5–9 mm at equal noise | — |

Moved-camera re-check: reproject the mapped tags through the stored pose.
Unmoved → **1.06 px** (PASS); camera nudged 4 cm + 3° → **12.55 px** (FAIL, marks
`recheck: status: fail`). See `images/synthetic_worked_example.png` (green =
detected corners, red = reprojected model corners).

## Live calibration attempt — result: both fixed cameras stay presence-only

Both fixed D415s were brought up on isolated `ROS_DOMAIN_ID=80` (UDPv4), streamed
cleanly (≈85 colour frames each), and were released by PID afterwards (cameras
confirmed free). **Neither camera has a mapped AprilTag (id 0 or 1) in its field
of view**, so neither could be calibrated, and I may not move anything physical:

- **table camera `025522060953`** (wide side view of the arm + gondola shelf):
  sees the arm and shelf across the cell; the table surface where tags 0/1 sat is
  below/outside the frame. No 36h11 tag in view. `images/live_table_025522060953.png`.
- **shelf camera `021422061772`** (close view of the gondola shelf): sees only the
  small shelf-label 2-D codes on the rails — those are **not** the 120 mm 36h11
  tags and are not in the map. No mapped tag in view. `images/live_shelf_021422061772.png`.

Both therefore remain presence-only in `camera_poses.yaml` (`cameras: {}`),
exactly as before — the camera side already handles this correctly.

### What the owner must place for each camera (then run one command)

The printables are `cams/cellmap/tags/apriltag36h11-id0-120mm.pdf` and
`…-id1-120mm.pdf` (print at 100 %, black-border edge = 120 mm).

1. **Preferred (uses the existing map, no new measurement):** put a 120 mm 36h11
   tag back **flat and face-up** at a mapped base position and make sure it is in
   the camera's view (both tags visible is better — enables the fixture check):
   - id0 at base_link **(0.116, −1.058, −0.057)**
   - id1 at base_link **(−0.505, −1.058, −0.058)**
   These sit on the table near the arm base. For the **shelf** camera that region
   is out of view, and for the **table** camera it is low in/below frame, so a
   mapped tag likely must be repositioned into view **or** option 2 used.
2. **If a mapped position is not in a camera's view:** place a tag anywhere in
   that camera's view, establish its base_link pose (the wrist camera can measure
   it once the driver is back up, or a tape measure to the base), add it to
   `tags/tag_world_poses.yaml`, then calibrate.

Then, per camera (example, table):
```
ROS_DOMAIN_ID=80 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 \
  cams/cellmap/calibrate_static_camera.py --serial 025522060953 --role table \
  --image /table/color/image_raw --info /table/color/camera_info \
  --depth /table/aligned_depth/image --depth-scale 1.0 \
  --optical-frame table_color_optical_frame
```
It writes `calib/base_to_camera_<serial>_<date>.yaml`; add an entry to
`cams/human_safety/params/camera_poses.yaml` (serial-keyed) and the camera side
publishes the TF and reports 3-D positions from that camera automatically.

## Guard against a moved camera

`recheck_static_camera.py` re-detects the tags and checks they still reproject
where the stored pose predicts; a failure stamps `recheck: status: fail` in the
calibration file, and the camera-side loader (`camera_poses.py`) then treats that
camera as uncalibrated (presence-only) — as does a stale calibration
(`max_age_days`) or a serial mismatch. All of this is unit-tested.

## Not a certified safety function

Unchanged from `human_safety`: this is a supplementary awareness layer. A better
camera pose makes its distances real, but the certified stops (pendant Force
limit; a safety-rated presence device) remain the actual protection.
