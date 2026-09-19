# nvblox dynamic layer feeding the planner — decisions & evidence (plan-nvblox-dynamic-v1)

Option (3) of the captain's "build all three top planning options". Simulation-only, plan-only.
Code: `cams/cellmap/dynamic/`. Proof artifacts: this directory.

## Decisions

1. **Use nvblox `mapping_type: dynamic`, fed by depth, as the delivered dynamic layer.**
   Verified on the planner image (nvblox_ros **4.4.0**): in `dynamic` mode the `get_esdf_and_gradient`
   service — the one cuMotion queries — returns a 3D ESDF that gains appearing/moving obstacles and
   decays them out. A person-sized volume is handled as a generic dynamic obstacle (routed around,
   then forgotten). No segmentation model, minimal GPU. This satisfies the captain's ask directly.

2. **Reject the people-segmentation modes (`human_with_static_tsdf/_occupancy`) for this purpose.**
   The mode ships but (a) the **PeopleSemSegNet model is not in the image** — only NGC installer
   scripts (`install_peoplesemsegnet_{shuffleseg,vanilla}.sh`), needing a download + TensorRT build;
   and (b) measured on this build, masked-human voxels are **removed from the served ESDF** (peak 0–2
   voxels vs 1129 for the same volume in `dynamic` mode), i.e. it would make cuMotion *ignore* people.
   Wrong tool for planner-side avoidance. Evidence: `esdf_human.json` vs `esdf_dynamic.json`.

3. **The map is a routing aid, never the stop.** Enforced by `routing_gate.py` (acceptance point keyed
   on the merged `humangate` decision, independent of map contents) and proved end-to-end in
   `gate_principle_proof.log`. Removing the dynamic layer changes routes but can never turn STOP into
   motion.

4. **Fixed cameras stay refused until calibrated.** `extrinsics_guard.py` mirrors
   `cams/human_safety/human_safety/camera_poses.py`: `sim` entries return a labelled provisional
   extrinsic; `real` entries are refused until a passing `calibrate_static_camera.py` file exists.
   The synthetic proof camera pose is `PROVISIONAL-SIM`, never a survey.

5. **Respect `cumotion/measured_joint_limits.yaml` exactly as the cell does.** The route proof checks
   every planned waypoint against those windows (as `execute_plan.py --limits` does) and fails if any
   waypoint is out of window. Observed: 0 violations.

## A parser bug that cost real time (recorded so no one repeats it)

`get_esdf_and_gradient` returns a **dense 3D grid**, `layout.dim = [x,y,z]`, `len(data)=x*y*z`,
**single channel** (ESDF distance; no packed gradient). An early `esdf_probe._parse_grid` wrongly
treated the last dim (z) as a channel count and read only one z-layer — so low obstacles registered
but high ones read as 0, which looked like "the service serves a 2D slice" / "dynamic mode is frozen".
It is neither. Fixed parser reads all `x*y*z` values as ESDF. A second red herring was probe AABBs
that included the always-occupied ground plane; probe floors now sit above it.

## Proof results (all PASS)

- **ESDF appear/decay** (`esdf_dynamic.json`): box appears in the served ESDF ~0.5 s after it appears
  and clears ~1.0 s after it leaves; person appears <0.5 s, clears 0.6 s; occ=0 before each appears.
- **Route** (`route_proof.json`): obstacle appears (in ESDF, occ 1372) → cuMotion detours **0.42 m**
  (TCP path shift), plan 0.18 s; obstacle leaves → route returns to the **identical** baseline
  (shift 0.0), plan 0.16 s; all waypoints within measured limits.
- **Gate principle** (`gate_principle_proof.log`): map re-routes, never STOP→motion.
- **Recorded-depth ingestion** (`replay_realism.json`): real D415 depth flows to nvblox at ~29 Hz
  (valid 0.64–13.6 m); ESDF population from the *uncalibrated* bag is presence-only/inconclusive
  (real-camera stamp/sync nuance, not a dynamic-layer defect).

## Measurements (RTX 3060 12 GB; voice ~2.2 GB baseline)

Map-update latency ~0.5–1.0 s; ESDF service call ~10 ms median / 13 ms p95; cuMotion plan ~0.16–0.20 s
with the live map; peak GPU ~3.2–3.4 GB total (≈1.0–1.1 GB for nvblox+cuMotion over the voice
baseline). Budget: voice 2.2 + this layer ~1.1 + person detector ~1.87 ≈ 5.2 GB of 12 GB co-reside;
the 8 GB sim does not (not booted).

## Not done / follow-ups

- Real people-*classification* (vs volume) needs the NGC PeopleSemSegNet model installed AND a change
  so cuMotion consumes a combined static+human ESDF — nvblox does not do that via this service here.
  Out of scope; `dynamic` mode already routes people-as-volumes.
- Full ESDF population from a recorded fixed-camera bag awaits that camera's calibration
  (`calibrate_static_camera.py`) and a stamp/sync pass; the tooling is in `proof/`.
- Real-arm anything: out of scope and forbidden by the task.
