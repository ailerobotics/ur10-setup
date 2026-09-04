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

Status 2026-08-27: **hand-eye calibration DONE and independently validated**
(below); the calibrated transform is what `wrist_rig.launch.py` publishes.
The mapping stack (segmenter + nvblox) is verified launchable and was left
running. **Still pending, next session: the scan sweep (step 3), the map
save, and the box-avoidance test (step 4).** Pipeline was bench-rehearsed end
to end on synthetic depth first (`bench/`, no arm motion). Results are
recorded in `cellmap-report.html` as they happen.

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

### Recalibration for the UPSIDE-DOWN mount (2026-09-03)

The wrist D435i is physically mounted **upside down**. A flip node publishes a
180deg-corrected stream alongside the raw one:

```
/wrist/color_upright/image_raw          /wrist/aligned_depth_upright/image
/wrist/color_upright/camera_info        /wrist/aligned_depth_upright/camera_info
        frame: wrist_color_upright_optical_frame
```

The correction is a real 180deg z-rotation in TF **plus mirrored principal
point** (cx 430.45 -> 416.55, cy 230.17 -> 248.83 on 848x480; fx/fy unchanged).
Verified 2026-09-03: `cx_upright = (width-1) - cx_raw` exactly, likewise cy.

**Calibrate against the stream the map consumes — the upright one.** Solving on
the raw stream and applying the answer to the upright stream is a 180deg roll
error, and no residual in `handeye_calibrate.py` would reveal it: the residual
is the spread of the tag through the *same* stream it was fitted on, so it
stays small while the whole map is bodily rotated. Every result file therefore
carries a `source:` block naming the image topic, the camera_info topic and the
four frames it refers to. **An extrinsic without its stream is not
interpretable** — quote the stream whenever you quote the numbers.

The upright **depth** stream shares the upright colour frame and intrinsics
exactly (verified 2026-09-03: same `wrist_color_upright_optical_frame`, same
fx/cx/cy), which is what makes a colour-solved extrinsic valid for the depth
that nvblox integrates.

Frames on the multi-camera bring-up are `wrist_link` (camera body) and
`wrist_color_upright_optical_frame`, not `camera_link` /
`camera_color_optical_frame`, so the tools take them as arguments
(`--cam-link/--optical`) and `wrist_rig.launch.py` publishes the child frame
**named in the file** rather than assuming `camera_link`.

### Why two tags

One tag reports a flattering residual: it is fitted and scored on the same
poses. A second tag elsewhere in the cell, captured from different poses, is an
**independent** estimate of the same fixed mount, and the disagreement between
the two is the honest accuracy figure.

The two sets are solved **separately**. Concatenating them into one
`cv2.calibrateHandEye` call is the tempting mistake: that solver assumes one
target rigidly fixed in the base frame and cancels it through relative motions,
so every cross-tag motion pair silently encodes the tag-to-tag offset as camera
motion. It does not error — it returns a wrong extrinsic with a plausible
residual. `tools/combine_tags.py` therefore solves per tag, reports the
agreement, averages rotation as a quaternion mean, and **refuses to write** a
combined mount when the tags disagree beyond tolerance.

### Freedrive from the workstation

`tools/freedrive.py on|off`. The pendant's own freedrive button cannot be used
for this: holding it **kills the external-control program** (RUNBOOK 4), which
drops the `/joint_states` the calibration depends on. Driving freedrive through
the ROS `freedrive_mode_controller` keeps the driver's connections up.

Two traps are encoded as refusals, both observed live on 2026-09-03:

* **Robot state is read from the dashboard (TCP 29999), never from
  `/io_and_status_controller/robot_mode` or `/safety_mode`.** Those topics are
  *latched-only* — 0 messages in 8 s of live subscription — so `ros2 topic echo
  --once` replays a historical sample to each new subscriber. They read
  `POWER_OFF` / `ROBOT_EMERGENCY_STOP` while the robot was genuinely
  `IDLE` / `NORMAL`.
* **The joint feed is checked for encoder JITTER, not for rate.** A dropped
  RTDE link keeps republishing the last packet at a perfect 125 Hz. Twice now
  a "healthy" 125 Hz feed has been frozen: 0.000 mdeg range across hundreds of
  samples, with a true pose completely different from the displayed one. Live
  looks like 4-9 mdeg of jitter.

`freedrive_mode_controller` must be activated in the **same**
`switch_controller` call that deactivates `scaled_joint_trajectory_controller`,
and `enable_freedrive_mode` is a **deadman**, not a latch — it is republished
at 4 Hz for as long as freedrive is wanted.

**This CB3 never reports BACKDRIVE for ROS freedrive, and that is correct.**
On PolyScope 3.15.8, mode 6 `BACKDRIVE` is the pendant's own brake-release
hand-guide state; ROS freedrive runs as URScript `freedrive_mode()` *inside*
the still-running external-control program, so the robot stays in mode 7
`RUNNING` throughout. There is also no Local/Remote toggle to check here — the
dashboard rejects `is in remote control` and `get operational mode` as unknown
commands (both e-Series features). The only real proof that freedrive engaged
is that **the arm actually moves** when pushed.

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

## Calibration result (2026-08-27)

18 poses, freedriven by the captain, tag 0 at 0.36-0.76 m. Solved with
`handeye_calibrate.py`; plot `calib/tool0_to_camera_2026-08-27.png`.

```
tool0 -> camera_link   xyz [0.0517, -0.0101, 0.0579] m
                       rpy [-134.95, -89.13, -44.48] deg
```

| solver | tool0 -> optical xyz (mm) | tag spread rms | tag rot rms |
|---|---|---|---|
| **park** (chosen) | 52.0, -25.1, 57.9 | **3.04 mm** | **1.60 deg** |
| horaud | 52.0, -25.2, 57.9 | 3.01 mm | 1.60 deg |
| tsai | 51.9, -25.4, 58.1 | 3.05 mm | 1.60 deg |
| daniilidis | 51.9, -25.3, 54.9 | 3.40 mm | 1.60 deg |
| andreff | 50.9, -23.4, 92.3 | 9.10 mm | 1.60 deg |

Park/horaud/tsai agree within **0.4 mm**; daniilidis is 3 mm off in z;
andreff (the linear method) is 34 mm out and is never chosen. The calibrated
transform differs from the old mount guess by **44.7 mm / 28.0 deg** — a map
built on the guess would have been wrong by far more than the 0.04 m
collision buffer, which is why calibration was made the gate.

**The translation target was met (3.0 mm rms < 5 mm); the rotation target was
NOT (1.60 deg rms > 1 deg). No sample was dropped to fix that, and here is
why.** The per-sample rotation residual is not a few outliers but a gradient
that tracks viewing geometry: it correlates **-0.72** with the tag's viewing
obliquity, and the error is ~3x *tilt* (mean 1.31 deg, about in-plane axes)
versus *spin* (0.47 deg, about the tag normal). That is the signature of the
planar-tag pose ambiguity — tilt is poorly constrained when you look straight
at a flat tag — i.e. noise in the *tag pose measurement*, not error in X.
Re-solving on subsets confirms it: dropping the worst samples barely moves
the residual, and the subsets that *do* meet 1 deg only manage it by
destroying pose diversity, which swings X itself by up to 44 mm:

| subset | n | rot rms | shift in X | max rotation spread |
|---|---|---|---|---|
| all 18 | 18 | 1.60 deg | — | 177 deg |
| drop 5 worst | 13 | 1.20 deg | 4.6 mm / 0.37 deg | 158 deg |
| obliquity >= 10 deg | 8 | 0.93 deg | 12.6 mm / 0.86 deg | 64 deg |
| obliquity >= 15 deg | 6 | 0.54 deg | **44.2 mm / 1.85 deg** | 64 deg |

AX = XB is constrained by rotation *differences* between poses, so a subset
that "passes" on 6 near-identical views is less trustworthy, not more.
Evaluated against the *unchanged* full-set X, the samples whose tag pose is
itself well conditioned (obliquity >= 15 deg) agree to **0.81 deg / 2.96 mm**
— inside target. The residual metric conflates X error with tag-measurement
noise, so it is an upper bound, not an estimate. The tie was broken by
independent validation rather than by argument.

### Validation — camera-only, and it PASSED

The flange carries only the camera: no gripper, no pointer, and the pendant's
active TCP measured **533 mm off the flange** (a leftover). So the
corner-touch method in `tools/tag_corners.py` is unusable here — it would have
recorded every corner half a metre out, and its own geometry checks would
*not* have caught it if the wrist orientation stayed roughly constant.
`validate_tag1.py` validates with the camera alone instead (799 views):

| check | result | threshold |
|---|---|---|
| consistency, tag 1 (566 views) | pos med **3.8** / p95 6.3 / max 11.4 mm; normal med **0.59** / p95 2.13 deg | med < 10 mm / 1 deg |
| consistency, tag 0 (233 views) | pos med **3.5** / p95 7.0 mm; normal med **0.55** deg | |
| scale, tag0-tag1 centres | **620.1 mm** vs 622.0 mm tape -> **1.9 mm** | < 10 mm |
| table plane | dz **0.7 mm**, normals differ **0.32 deg** | < 5 mm / 1 deg |
| coverage | 0.30-0.71 m, obliquity 2-55 deg, **392 views beyond 30 deg** | must include slanted views |

**Judge on the median, not the rms**: fast freedrive motion throws outliers up
to ~6 cm (max tool speed 211 mm/s) that dominate an rms while saying nothing
about the calibration. Coverage is checked as a histogram, not min/max — a
small spread from near-identical views bounds nothing, and the tool warns
when the sweep was too uniform.

This settles the 1.60 deg question: **two coplanar tags agree to 0.32 deg and
0.7 mm in z, and scale is right to 1.9 mm against a tape.** X is good; the
solve's rotation residual was pessimistic. An independent overhead-only read
(25 frames) gave 7 mm in z and 1.6 deg of tilt — worse, and consistently so,
because overhead views sit exactly in the ill-conditioned regime.

## Procedure

Safety: no commanded arm motion in steps 1–3 (freedrive only); e-stop in
reach whenever the arm is powered; one writer at a time (RUNBOOK §6b);
freedrive kills the external-control program (RUNBOOK §4) — the rearm
watchdog restores it, and `/joint_states` (RTDE telemetry) keeps flowing
regardless (confirm on the first session: the calibration tool prints the
tool0 pose live).

### Resuming a part-finished session
The calibration is a **one-off**: once `calib/tool0_to_camera_<date>.yaml`
exists, steps 0–2 do not need repeating unless the camera is unstrapped or
the arm re-calibrated. To resume at the scan: reconnect the robot
(RUNBOOK §1–§4), turn freedrive back on, confirm the rig is publishing the
calibrated transform (`[wrist_rig] tool0->camera_link from CALIBRATION` in
its log, or `tf2_echo tool0 camera_link`), then go straight to step 3.

### 0. Tags on the table
Print `tags/apriltag36h11-id0-120mm.pdf` and `id1` at **100 %** (the 100 mm
ruler must measure 100 mm), mount on rigid board. Tag 0 flat on the table
inside reach where the wrist camera can see it from many angles (this is the
calibration target); tag 1 ~0.5 m away, also flat, in reach (validation).
Both stay put for the whole session. Keep `wrist_rig.launch.py` running.

### 1. Hand-eye calibration

**Current form (2026-09-03 onward): upright stream, two tags, freedrive from
the workstation.** Cameras are usually already up and owned by another session
— verify by frame stamps, do not restart them.

```bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# freedrive, held by a 4 Hz deadman; leave this running, Ctrl-C releases it
python3 cams/cellmap/tools/freedrive.py on

# one capture per tag, --tag-id 0 then --tag-id 1
python3 -u cams/cellmap/handeye_calibrate.py \
    --image /wrist/color_upright/image_raw \
    --info  /wrist/color_upright/camera_info \
    --cam-link wrist_link --optical wrist_color_upright_optical_frame \
    --tag-id 0 --tag-size 0.120 --vmax 0.03 --target 18

# combine; REFUSES if the two tags disagree beyond 10 mm / 2 deg
python3 cams/cellmap/tools/combine_tags.py \
    calib/tool0_to_camera_<date>-tag0.yaml calib/tool0_to_camera_<date>-tag1.yaml
```

`--vmax 0.03` (rad/s) is the stillness gate: a hand resting on a freedriven arm
never gets to 0.01. Only the combined, date-only file is picked up by
`calib:=auto`; the per-tag files are intermediates and are deliberately
excluded from that glob.

Original single-tag form (raw stream, standalone rig):
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

### 2. Validation with tag 1 — camera only
```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/validate_tag1.py --expected-mm <tape>
```
Measure tag 0 centre → tag 1 centre with a tape (mm) and pass it as
`--expected-mm`. Freedrive the camera around **both** tags: ~1 min each,
0.35–0.75 m, deliberately **slanted** (20–45° off straight-on) as well as
overhead, plus wrist rotation. Ctrl-C when done; the verdict prints and a
yaml is written. Accept on the **median**: < 10 mm position spread and < 1°
normal spread, scale within 10 mm, both tags within 5 mm in z and 1° in
normal. A spread that grows with angle/range = rotation error; a constant
offset between tags = translation/scale error.

`tools/tag_corners.py` (TCP touches three corners) is the arm-measured
alternative and is **not usable on this cell** — nothing is mounted on the
flange but the camera, so there is no tip and the pendant TCP is meaningless.
It is kept for a cell that does have a pointer.

Every accepted view is streamed to `calib/validate-raw-<date>.jsonl`, so a
sweep survives any crash and can be re-analysed offline with
`--from-raw <file>` (optionally `--max-speed` to drop fast-motion views)
without asking the captain for another one.

### 3. Scan and save

**Current form (2026-09-03 onward): the UPRIGHT stream.** The camera is mounted
upside down and the map must be built on the same stream the calibration was
solved against, or the whole map is rotated 180 deg about the optical axis.

```bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 0. the pipeline's depth contract is 16UC1 MILLIMETRES, and only the RAW
#    orientation has such a topic - bridge the upright one (additive; starts
#    no camera, touches nothing in zz_rigcams)
python3 cams/cellmap/tools/depth_to_mm.py

# 1. the mount TF, straight from the calibration file. wrist_rig normally
#    publishes it; the multi-camera bring-up deliberately does not.
python3 cams/cellmap/tools/publish_mount.py \
    cams/cellmap/calib/tool0_to_camera_<date>.yaml

# 2. segmenter + nvblox, on the upright topics
cams/cellmap/run_scan.sh \
    color_image:=/wrist/color_upright/image_raw \
    color_info:=/wrist/color_upright/camera_info \
    depth_image:=/wrist/aligned_depth_upright/image_mm \
    depth_info:=/wrist/aligned_depth_upright/camera_info

# 3. the viewer, and freedrive for the captain
DISPLAY=:0 XAUTHORITY=$(ls -t /tmp/xauth_* | head -1) \
    cams/cellmap/run_rviz.sh cellmap_upright.rviz
python3 cams/cellmap/tools/freedrive.py on

# 4. sweep, then save + mesh
cams/cellmap/scan_session.sh cell-$(date +%F)

# 5. verify against geometry we already know
python3 cams/cellmap/tools/verify_map.py ~/ur10/cumotion/maps/cell-<date>.ply
```

**Confirm the stack is really on the upright stream before the captain sweeps** —
a wasted sweep is a wasted captain. The segmenter prints its intrinsics at
startup: they must be the UPRIGHT ones (`cx=416.55, cy=248.83` at 848x480, i.e.
mirrored), not the raw `cx=430.45, cy=230.17`. nvblox also names the frame in
`/nvblox_node/back_projected_depth/wrist_color_upright_optical_frame`.

**The screen may be locked.** RViz will be running behind an xscreensaver
password prompt and only the captain can unlock it; `xscreensaver-command
-deactivate` does not bypass the lock.

Original single-stream form (standalone wrist rig, raw orientation):

```bash
cams/cellmap/run_scan.sh            # segmenter + nvblox, refuses without a calib yaml
cams/cellmap/run_rviz.sh            # map growing on the display (needs DISPLAY/XAUTHORITY of the session)
cams/cellmap/scan_session.sh cell-$(date +%F)
```
Sweep recipe is printed by the session script. **Sweep the camera slowly**
(~0.1–0.2 m/s) at 0.4–1.0 m standoff, and cover every surface **from at
least two directions** — a surface seen once stays noisy. Order that works:
the whole table top from three sides, then the reachable shelf faces from two
heights each, then anything standing on table or floor within reach, then the
reachable walls and fixtures low and high. Never point closer than ~0.2 m
(below the D435i minimum range nothing is mapped at all). **What good looks like:** a
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

### 5. Clean a map in place, then plan AprilTag 1 → shelf (2026-09-04)

Clean-in-place instead of re-scanning when the map is right but carries an
unwanted obstacle — here the robot base/pedestal (never masked during a scan)
plus isolated speckle. `tools/nvblx_clean.py` edits the `.nvblx` SQLite TSDF
directly (voxel-precise box crop-out + isolated-block denoise), writes a NEW
file, leaves the original for rollback, and self-checks; offline test
`tools/test_nvblx_clean.py` (no rig). Full procedure and verify commands:
`RUNBOOK.md` §10 "Clean an existing map in place". Descriptor +
before/after evidence: `cumotion/maps/cell-2026-09-04-cleaned.md`.

Collision-aware demo against the cleaned map — hover above AprilTag 1 (base_link
−0.506, −1.058, −0.057) to a standoff short of the gondola shelf, tool held as
parked, plan-only:

```bash
cumotion/start_cumotion_map.sh cell-2026-09-04-cleaned.nvblx        # ur10_map.xrdf = full collision model
# Always a FRESH label: plan_test.py writes plans/<label>.yaml unconditionally
# and cuMotion trajopt is not bit-deterministic, so re-using the label
# tag1-shelf-cumotion would replace the committed VALIDATED plan (and drop its
# box_check) with an un-swept re-plan. Never re-use that label.
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/plan_test.py \
    --label tag1-shelf-replan --pipeline isaac_ros_cumotion \
    --start-joints 1.2971236651069784,-3.0399117660822967,-0.2455590220532742,5.930127513948878,-1.1606540077724152,-0.5524234521170055 \
    --goal-pose=-0.10,0.80,0.95 --pose-goal
#   --start-joints = the committed plan's start_joints, so the re-validation
#   checks exactly its start -> goal wherever the arm is parked. (The original
#   run used --start-pose=-0.506,-1.058,0.25, which solves IK from the LIVE
#   joints and can pick a different branch — use that only for a NEW route.)
#   pneumatic line on the arm? add --joint-limits <windows yaml> (RUNBOOK §10
#   "Reduced joint windows"); the planner stays on ur10_map.xrdf regardless.
# Then BOTH arm-sweep clearances on that label must be >= 0 (each run overwrites
# the label's single box_check slot):
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/plan_test.py \
    --recheck tag1-shelf-replan --check-box=-1.0,1.0,-1.92,-0.90,-0.12,-0.03    # table slab
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/plan_test.py \
    --recheck tag1-shelf-replan --check-box=-0.6,0.4,1.10,1.94,0.45,1.55       # shelf / cartons
```

The committed `plans/tag1-shelf-cumotion.yaml` was produced once, on 2026-09-04,
by that same plan command under `--label tag1-shelf-cumotion`: cuMotion arcs up
over the table and stops short of the shelf — 1.27× straight line, +13.7 cm to
the shelf/carton volume, +4.8 cm to the table (both `--recheck --check-box`
commands + outputs, and why the yaml keeps only the last one:
`data/cell-clean-approach-v1/plan-clearance-checks.md`). Cross-checked vs live
wrist-cam carton detections (base_link y≈1.13–1.32) — the map shelf is where
the cartons really are. That yaml is the collision-validated EVIDENCE for the
demo, NOT an executable plan: it drives wrist_1 to 355.1° (100 of 313 waypoints
above 350°, 61 above the 353.2° bind the captain measured by hand-sweep in task
joint-limits-freedrive-v1 — that task's measured-limits file is its deliverable
and is not in this branch), so it must not be sent to the arm; the executable
plan and captain command come from that task. Re-runs must use a different
label.

Cross-check the map's shelf against reality with the wrist (hand-eye-calibrated)
camera, not the shelf cam — only the wrist places detections in base_link. If
the mount TF is not up (`zz_rigcams` runs without it), restore it with
`tools/publish_mount.py calib/tool0_to_camera_<date>.yaml
--key tool0_to_camera_color_optical_frame` (TF only, touches no camera). The
eye-level parked view boxes shelf *boards* as `tray`, not the cartons (recorded
viewpoint limitation) — per-carton confirmation lands during the captain's
approach. Captain execute + safety checklist: prepared per run in the task's
`data/cell-clean-approach-v1/captain-execute-note.md` — SUPERSEDED for execution
(wrist_1 exceeds the measured bind); kept as the collision-evidence record and
safety-checklist reference for task joint-limits-freedrive-v1's executable plan.

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
