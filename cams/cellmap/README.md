# cams/cellmap — 3-D obstacle map of the cell from the wrist camera, for cuMotion (v1)

The D435i stays strapped to the tool flange. The captain freedrives the arm
so the camera sweeps the cell; the camera pose is pure kinematics
(`base_link → tool0` from `/joint_states` plus a **hand-eye-calibrated**
`tool0 → camera_link`), nvblox fuses the depth into a 2 cm ESDF directly in
`base_link`, the map is saved once, and every later session cuMotion plans
against it. Design history: firstmate's `cell-map-slam.html` (handheld +
cuVSLAM, 2026-08-26 rev 2) → captain decision the same day: camera stays on
the arm, no SLAM (FK is sub-mm and drift-free; the only error left is the
mount transform, so calibration becomes step 1). This README is the "why"
and the procedure; recurring commands are in `RUNBOOK.md` §10.

Status 2026-08-26: pipeline built, **bench-rehearsed end to end on synthetic
depth** (`bench/`, no arm motion) and wired live to the wrist camera
(segmenter + nvblox integrating through the mount guess); the hand-eye
calibration, validation, scan and box test are the captain's physical steps
and are recorded in `cellmap-report.html` as they happen.

## Pieces

| File | Runs where | Does |
|---|---|---|
| `handeye_calibrate.py` | host | AX = XB eye-in-hand calibration from AprilTag 0 + TF; gated sampling, 5 solvers, residuals, `calib/tool0_to_camera_<date>.yaml` |
| `../approach/wrist_rig.launch.py` | host, domain 0 | the wrist camera; `calib:=auto` publishes the calibrated `tool0 → camera_link` (default), `calib:=guess` the old mount guess; `camera_params:=../cellmap/d435i_wrist_serial.yaml` pins the D435i by serial |
| `tools/make_tags.py` → `tags/` | host | print-exact AprilTag 36h11 sheets (120 mm, corner labels A-D, 100 mm ruler), drawn as **vector** cells and self-verified by re-rendering the finished PDF and re-detecting it |
| `tools/tag_corners.py` | host | arm-measured pose of tag 1 (TCP touches corners A, B, C; dwell-detected from TF or `--pendant` readings) |
| `tools/tag_validate.py` | host | second-landmark check: tag 1 seen through the calibrated chain vs its arm-measured pose |
| `run_scan.sh` → `scan.launch.py` | Isaac container `ur10-cellmap-scan`, domain 0 | robot segmenter + nvblox fed by the wrist rig's topics; refuses to run without a calibration file |
| `nvblox_cell.yaml` | container | nvblox deltas vs NVIDIA's manipulator config (2 cm, `base_link`, workspace box, no decay) |
| `d435i_wrist_serial.yaml` | host | quoted params file that pins `serial_no` to the wrist D435i (the trap below) |
| `scan_session.sh` | host | sweep recipe → Enter → `save_map` + `.ply` |
| `tools/map_ctl.py` | container | `save` / `load` / `ply` / `esdf` (column probes, per-sphere clearances) |
| `run_rviz.sh` → `cellmap.rviz` | container on the workstation display | RViz **with the nvblox plugin** (host RViz cannot decode `nvblox_msgs/Mesh`) |
| `tools/render_map.py` | host | headless top/side renders of a saved `.ply` with tool paths and boxes (report figures without a display) |
| `planner.launch.py` | container `ur10-cumotion` | nvblox (serves the saved map, no camera) → `load_map` → cuMotion `read_esdf_world:=true` |
| `../../cumotion/start_cumotion_map.sh` | host | opt-in sibling of `start_cumotion.sh`; newest `cell-*.nvblx` |
| `../../cumotion/ur10_map.xrdf` | container | world buffers 0.04 m on forearm/wrists/tool, 0.02 on shoulder/upper arm (why: below) |
| `tools/plan_test.py` | host | plan-only legs (joint or **pose** goal, `--start-from` chaining), FK tool path, arm-sweep clearance vs a box, plots |
| `tools/fake_depth.py` | host | synthetic depth camera + TF for the bench rehearsal |

## Frame chain

```
world → base_link → tool0 → camera_link → camera_color_optical_frame
        (robot, /joint_states)  ↑ calib/tool0_to_camera_<date>.yaml   (RealSense static TF)
```

* No `map`/`odom` frames, no registration: the depth is integrated in
  `base_link` at the image stamp through TF. The image stamps run ~160 ms
  behind wall clock (measured); TF keeps 10 s, so lookups at the stamp are
  exact — as long as the arm is *moving slowly*: any timestamp error shows
  up as smear proportional to joint speed (see "what good looks like").
* The calibration is solved for `tool0 → camera_color_optical_frame` (what
  the tag is seen in) and converted to `tool0 → camera_link` through the
  driver's own static `camera_link → optical` (0, 0.015, 0) so the
  RealSense's internal tree keeps working; `wrist_rig.launch.py` publishes it.
* The pendant's TCP readout is in UR's `base` frame, which ur_description
  rotates **180° about Z** relative to `base_link` (verified live:
  `tf2_echo base_link base` = yaw 180°), and this robot's active TCP is
  ~0.5 m off the flange. `tag_corners.py` therefore reads TF; `--pendant`
  converts pendant readings through the same live TF.

## Design decisions (dated 2026-08-26 unless noted)

**Hand-eye calibration is the gate, not an option.** The mount transform in
`wrist_rig.launch.py` is a documented guess (±5 cm translation, 4.2°
gravity-fit residual): 4° at 1 m is 7 cm of map error, more than the
planner's 4 cm buffer, so a map on the guess is *worse* than no map (it
would clear paths that hit real shelves). `run_scan.sh` refuses to start
without a `calib/tool0_to_camera_*.yaml` (override `nocalib` for wiring
tests only). Target < 5 mm / 1°; validation with a second, independently
measured tag < 10 mm / 1°.

**Why the calibration tool is picky.** Samples are taken only when the arm
is still (joint velocities < 0.01 rad/s for 1 s, gyro < 0.03 rad/s) because
the colour sensor is rolling-shutter and the TF is looked up at "now";
the tag must be 0.3–0.8 m away and fully inside the frame with a 40 px
margin (corner refinement degrades at the border); mean reprojection error
< 1 px; the tag pose must be stable over 10 frames (< 2 mm); and each pose
must differ from the last by > 8° or > 4 cm, because AX = XB is only
constrained by *rotation differences* between poses — twenty near-identical
poses give a confident-looking but wrong answer. Five solvers are run and
their agreement is reported; the residual that matters is the spread of the
tag's base_link pose across samples when each is mapped through X (that is
literally what the map will feel). Synthetic check: Park/Daniilidis/Tsai/
Horaud recover a known X to 1.3 mm / 0.1° with 1 mm / 0.2° tag noise;
Andreff is the outlier (linear method) and is never chosen.

**2 cm voxels, `base_link`, bounding box = reach + 0.5 m, y to +2.4.** 1 cm
(NVIDIA's manipulator default) is ~8× the memory for a map whose error is
itself ~1–2 cm; the box (`nvblox_cell.yaml`) is the UR10 envelope plus
margin, extended to y 2.4 so the gondola front is mapped even where the arm
cannot reach it — a plan may still sweep the wrist toward it. Decay and
radius-clearing are OFF (the manipulator config expects a fixed camera and a
moving scene; for a snapshot they would erase the far side while the arm
sweeps elsewhere). Max integration range 3 m (D435i depth noise grows
~quadratically). `projective_integrator_max_weight` 20 instead of 5: a
snapshot wants noise averaging over many viewpoints, not reactivity.

**Robot segmenter with a 0.10 m buffer.** The wrist camera sees the arm from
very close where the XRDF spheres are a coarse model, and the strapped
camera + cable are not in the model at all; without the mask the arm maps
itself and every plan starts "in collision". The masked halo stays
*unobserved*, which cuMotion treats as **free** (bench-verified: planning
from the park pose succeeded on a map whose park region was never observed),
so the halo costs nothing.

**Unobserved = free** (bench finding; `map_ctl.py esdf` shows the −1000
sentinel). Consequence: the map only protects against what the camera SAW.
The sweep recipe covers everything the arm can reach; unscanned obstacles
are invisible to the planner exactly as before the map. The ground plane and
the approach demo's whitelist boxes stay on as independent guards.

**Buffers: 0.04 m on forearm/wrist_1/2/3/tool0, 0.02 kept on
shoulder/upper_arm.** The extra 2 cm absorbs the map error budget. It cannot
be applied to the upper arm: its root sphere (r 0.09) sits 3.8 cm above the
mounting plane — 1.8 cm clear of cuMotion's ground plane at 0.02, *inside*
it at 0.04, and every start state then fails with
`INVALID_INITIAL_CSPACE_POSITION` (bench). Those links never leave the base
region anyway.

**Load-then-plan, gated on the loader's exit code.** cuMotion with
`read_esdf_world:=true` blocks forever if the ESDF service is absent
(`start_cumotion.sh`'s warning), and an nvblox that is up but EMPTY makes
`World update failed` on every request — a planner that looks alive and
refuses everything. `planner.launch.py` starts cuMotion only after
`map_ctl.py load` returns 0, else shuts down loudly. Same container name as
the default planner (`ur10-cumotion`), so docker refuses to run both.

**Pose goals for collision-aware IK.** With a map, a joint goal pre-solved by
MoveIt's `/compute_ik` (empty scene) can be in collision for cuMotion —
bench: the goal's forearm ran 1–9 cm into the box and trajopt failed with
`UNKNOWN_STATUS`. A tool0 *pose* goal lets cuMotion pick the configuration
against its own world (`plan_test.py --pose-goal`; `--start-from <label>`
starts a leg from a configuration cuMotion itself found valid). The approach
demo's 170° rewind guard is re-applied to every returned trajectory.

## Bench rehearsal (2026-08-26, synthetic depth, arm never moved)

`tools/fake_depth.py` rendered a table plane, a shelf slab at y 1.4 and a
20 cm cube at the midpoint of park → (0.60, 0.70, 0.75), publishing the
camera TF directly; the real nvblox/cuMotion stack did the rest.

| plan | planner | tool path | max dev. from line | arm vs box (spheres + buffer) |
|---|---|---|---|---|
| `sim5-box-cumotion` | cuMotion + map, pose goal | 1.13 m (1.40×) | 34 cm | **clear, +3.5 cm** |
| `sim5-box-pilz` | Pilz PTP (no map) | 0.93 m (1.14×) | 19 cm | **through, −11 cm** |
| `sim6-nobox-cumotion` | cuMotion + map, box removed & re-scanned | 0.95 m (1.18×) | 22 cm | through the *former* box volume, −6.7 cm → detour gone |

![bench plans](bench/sim-plans.png)

Also verified: `save_map`/`load_map` round trip (19–33 MB per synthetic
map), ESDF hand-off (`Initialized grid from nvblox: 193×217×121 @ 0.02 m`),
planning 0.1–0.13 s, ESDF zero-crossing at the table surface within a voxel,
and — live, through the guess TF — segmenter output at 18 Hz and 105 k
observed voxels from the parked wrist view.

## Procedure

Safety: no commanded arm motion in steps 1–3 (freedrive only); e-stop in
reach whenever the arm is powered; one writer at a time (RUNBOOK §6b);
freedrive kills the external-control program (RUNBOOK §4) — the rearm
watchdog restores it, and `/joint_states` (RTDE telemetry) keeps flowing
regardless (confirm on the first session: the calibration tool prints the
tool0 pose live).

### 0. Tags on the table
Print `tags/apriltag36h11-id0-120mm.pdf` and `id1` at **100 %** (the 100 mm
ruler must measure 100 mm), mount on rigid board. Tag 0 flat on the table
inside reach where the wrist camera can see it from many angles (this is the
calibration target); tag 1 ~0.5 m away, also flat, in reach (validation).
Both stay put for the whole session. Keep `wrist_rig.launch.py` running.

### 1. Hand-eye calibration
```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/handeye_calibrate.py --target 18
```
Pose recipe (freedrive, tag 0 always fully in view, 0.3–0.8 m): 5 poses
from the left, 5 from the right, 5 from above, 3 close/low; between poses
rotate the wrist ~30° about the optical axis and tilt 15–30°; hold still
~2 s. The tool beeps and prints `sample N/18 ACCEPTED` or the reason it did
not take one (moving, too far, tag at the border, same pose as before…).
Read the final block: all four of Park/Daniilidis/Tsai/Horaud within ~2 mm
of each other and residual < 5 mm / 1° → good. Restart `wrist_rig.launch.py`
(it now loads the yaml; check its `[wrist_rig] tool0->camera_link from
CALIBRATION` line).

### 2. Validation with tag 1
```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/tag_corners.py --tags 1 --tip tcp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/tag_validate.py --tag cams/cellmap/anchor/tag1.yaml --seconds 60 --out cams/cellmap/calib/validate-$(date +%F).yaml
```
Corners: freedrive the tip to A, B, C of tag 1 (labels on the sheet), still
~3 s each; the tool checks |AB| = |BC| = 120 ± 4 mm and ∠ABC = 90 ± 1°.
Validation: move the arm around while the camera looks at tag 1; accept at
median < 10 mm / 1°. A residual that grows with angle/range = rotation
error; a constant offset = translation error → re-calibrate with more
varied poses.

### 3. Scan and save
```bash
cams/cellmap/run_scan.sh            # segmenter + nvblox, refuses without a calib yaml
cams/cellmap/run_rviz.sh            # map growing on the display
cams/cellmap/scan_session.sh cell-$(date +%F)
```
Sweep recipe is printed by the session script. **What good looks like:** a
surface that stays put while the arm moves; table at z ≈ 0 (`map_ctl.py esdf
--probe x,y`), shelf front at y ≈ 1.5 (approach demo measured 1.516 at
x ≈ 0), arm absent. **Smear** while moving = calibration or timestamp error;
stop, do not save. Maps: `cumotion/maps/` if < 50 MB (committed), else
`~/ur10/cumotion/maps/`.

### 4. Plan with the map
```bash
systemctl --user stop ur10-cumotion; docker stop ur10-cumotion   # the unit stop alone can orphan the container
cumotion/start_cumotion_map.sh                                   # newest cell-*.nvblx, voxels:=true
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/plan_test.py --label with-box --goal-pose X,Y,Z --pose-goal
```
Acceptance: a cardboard box scanned on the straight line between two poses →
the plan routes around it (arm-sweep clearance ≥ 0 via `--recheck
--check-box`); box removed + re-scan → the plan straightens. Execution stays
captain-confirmed (approach demo rules, ≤ 0.05 scaling).

## Traps hit

- **A print-ready PDF that renders as stripes** (2026-08-27, cost the captain
  a print run). `tools/make_tags.py` drew the tag with
  `ax.imshow(marker, cmap='gray')`; matplotlib 3.6.3's PDF backend encoded
  that two-valued array as a 1-bit `/Indexed /DeviceRGB` image with
  `/Predictor 10` and packed the rows wrong. Poppler *and* ghostscript render
  diagonal black/white stripes and `detectMarkers` finds nothing at any
  scale — while the **PNG written from the same figure was perfect**, so
  every thumbnail and preview looked fine. Fix: draw the 8x8 cells as vector
  `Rectangle` patches (no image codec path at all, and resolution-independent),
  and *verify the artefact you actually ship* — `make_tags.py` now renders its
  own PDF with `pdftoppm`, re-detects the id and measures the black square
  (120.01 mm) before it prints "wrote". Regression-checked: the gate fails on
  the old PDF and passes on the new one. This is the task's own thesis applied
  to its tooling — a failure that is invisible until it is physical is exactly
  what the map is meant to eliminate.
- **`serial_no` cannot be pinned through a launch argument** (see
  `../README.md`): launch_ros re-serialises the parameter dict through YAML
  without quoting and rcl types `018322070619` as a double. A params *file* is
  passed straight to `--params-file` untouched, so
  `d435i_wrist_serial.yaml` + `wrist_rig.launch.py camera_params:=...` pins the
  wrist D435i. Needed since 2026-08-27, when a second RealSense (D415) was
  connected and began enumerating *first*.
- **The colour stream can stall on its own while depth stays healthy**
  (2026-08-26 19:24 → 2026-08-27 14:47). Nothing logged an error; the
  calibration tool simply sat at `0/18 samples, no tag in view` for 19 h.
  Restarting `wrist_rig.launch.py` fixed it with no USB power-cycle. Check
  `ros2 topic hz /camera/rgb` **and** the image stamp age, not just that the
  camera node is running.

## Known limitations (v1)

- **Snapshot map.** Anything moved after the scan is invisible until
  re-scanned; unobserved space is free. Re-scan after moving anything the
  arm can reach; the whitelist boxes and ground plane stay on.
- **Coverage = the wrist camera's view from the reach envelope**; walls
  beyond reach are out of scope by design.
- **Error budget** ≈ calibration residual + 2 cm voxels + D435i depth noise
  (~1 % of range); the 0.04 buffer absorbs it, thin objects may not.
  Min range ~17 cm: surfaces closer than that to the lens are not mapped.
- **Camera body not in the arm's collision model yet** (spheres to add to
  `ur10_map.xrdf` on `tool0` once the calibrated offset is known — next step).
- **Version drift**: Isaac 4.x pins librealsense 2.56.3 / fw 5.16.0.1; this
  box runs 2.58.1 / 5.12.8.200.
- **Planner swap is manual**: default and map planners share the container
  name; `systemctl --user stop ur10-cumotion` can leave the container running
  (observed 2026-08-26) — `docker stop`.

## Future work (out of scope for v1)

Live refresh: keep nvblox integrating from the wrist camera during normal
operation (same node, same cuMotion switch) so moved trays stop being
invisible; a second fixed camera (the tray rig) for the table region.
