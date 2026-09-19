# Decisions — fixed-camera base_link calibration (human-cam-calib-v1)

## D1. PnP against mapped AprilTags (not point-cloud ICP, not hand-eye)
A fixed camera has no arm motion, so hand-eye (AX=XB) does not apply. Two real
options: (a) colour PnP against tags whose base_link poses are known, composed
into base_link → camera; (b) fit the camera's depth cloud to a base_link model.
**Chose (a).** It is the only one that works for the **colour-only** camera the
captain asked to support, reuses the AprilTag/PnP tooling already trusted here
(`validate_tag1.py`), meets the design target directly (reprojection in px, depth
in cm), and does not depend on the arm being up. Measured offline accuracy 3.0 mm
/ 0.18° at 0.33 px reprojection — well inside the < 5 px / < 1 cm target.

## D2. Tag world poses derived from the trusted wrist hand-eye, not re-measured
The known base_link tag poses come from the validated wrist hand-eye
(`derive_tag_world_poses.py`), giving full 6-DoF (position + orientation), keyed
to the 2026-09-03 tag placement. Absolute accuracy inherits the wrist budget
(~8 mm); the internal 2 mm spread is only consistency. Independent check: the
derived id0–id1 spacing (621.3 mm) matches the 622 mm tape. Re-derive if the tags
are ever re-placed.

## D3. "Agreement between tags" = tag-separation fixture, not per-tag pose spread
A single static view's single-tag PnP is depth-weak (coplanar two-fold ambiguity
+ planar depth bias), so two per-tag pose estimates can disagree by cm even when
the joint solve is accurate to mm. Gating on that would falsely fail good
calibrations. Gate instead on reprojection (< 5 px) + depth (< 1 cm where depth
exists) + the well-constrained tag-to-tag separation (< 10 mm); keep the per-tag
pose disagreement as a reported diagnostic only.

## D4. Identify cameras by serial; leave the role/params files alone
`camera_poses.yaml` entries are serial-keyed and the loader refuses to publish if
the calibration file's serial differs (a swapped, interchangeable D415 cannot
inherit another's pose). Per the brief, `cams/params/shelf_d415.yaml` /
`table_d415.yaml` and the role question were left untouched. Live enumerate
(`rs-enumerate-devices -s`) confirmed the two attached fixed serials:
025522060953 and 021422061772 (+ wrist 018322070619).

## D5. Failed/stale re-check ⇒ uncalibrated (fail toward presence-only)
The moved-camera guard writes `recheck: status: fail` into the calibration file;
the camera-side loader then drops the camera to presence-only, as it does for a
stale (`max_age_days`) or serial-mismatched file. Consistent with the layer's
fail-toward-STOP philosophy.

## D6. Live calibration not possible now — no mapped tag in view
Both fixed cameras streamed fine but neither has a 36h11 tag (id 0/1) in view
(table cam sees arm+shelf from the side; shelf cam sees only small shelf-label
codes). Physical objects may not be moved by this task, so both cameras are left
presence-only and exact placement instructions are given for the owner
(report.md "What the owner must place"). Everything else (calibrator, guard,
wiring, tests, docs) is complete and validated offline.

## D7. Camera etiquette
Cameras are shared and exclusive per process. Every hold checked `fuser
/dev/video*` free first, ran on isolated domain 80 (UDPv4), stayed well under
60 s, and was torn down by explicit PID with `fuser` reconfirmed free — never by
name/pattern, and no process this task did not start was touched.
