# nvblox dynamic layer feeding the planner (plan-nvblox-dynamic-v1)

**What this is.** The cell's nvblox map is a *static snapshot* consulted at plan time
(`cams/cellmap/nvblox_cell.yaml`, `live.launch.py` — decay OFF, "the world only ever gains
obstacles"). This package runs nvblox's **`dynamic`** multi-mapper instead, fed by depth, so obstacles
that **appear or move** — a box, a person-sized volume — enter the ESDF cuMotion consults and, when
they leave, **decay back out**. cuMotion then plans a route that goes *around* them, and comes back
once they are gone.

**SIMULATION ONLY. PLAN-ONLY. Not a certified safety function.** Everything here runs in a throwaway
container built from the planner image, on ROS domain 85, on a private container network. No robot,
no live camera during proofs (synthetic + one recorded bag), and cuMotion plans with
`plan_only=True` — nothing is ever executed. This layer never stops the arm; see the principle below.

---

## The principle that governs everything here

> **A person — or any obstacle — in the map is a ROUTING aid, never the stop mechanism.**

The finished human-safety design (`~/firstmate/data/human-dynamic-plan-s1/report.md` §5) rejected the
map as a stop because a *missed* detection means *no* obstacle voxel and the arm would proceed — it
fails **open**. So the merged human-safety gate (`humangate/`, fail-toward-stop) remains the single
authority over motion; this layer only changes the **shape** of the plan. Adding or removing a
dynamic obstacle can reroute the arm, but it can **never** turn a STOP into motion.

This is enforced in code by `routing_gate.py` (the acceptance point forwards a plan only when the gate
says RUN, independent of map contents) and proved by `tests/test_routing_gate.py` /
`proof/run_gate_principle_proof.sh` — end to end through the *real* `humangate` decision core: the
same dynamic-obstacle plan is held under a STOP camera state and forwarded under a CLEAR-dwelled one;
**only the camera state changed the outcome, never the map.**

---

## What nvblox actually ships here (checked, not assumed)

Planner image `nvcr.io/nvidia/isaac/ros:isaac_ros-shelfbot_manual-amd64`, **nvblox_ros 4.4.0**:

| `mapping_type` | needs | feeds cuMotion's ESDF? | used here |
|---|---|---|---|
| `static_tsdf` / `static_occupancy` | depth | yes (snapshot) — the cell today | no |
| **`dynamic`** | **depth only** | **YES — obstacles appear and decay** | **YES (delivered)** |
| `human_with_static_tsdf` / `_occupancy` | depth **+ a segmentation mask** | **NO for humans** (see below) | investigated, rejected |

Two findings that decided the design (evidence in `data/plan-nvblox-dynamic-v1/`, `decision.md`):

1. **`dynamic` mode is the mode that feeds the planner.** `get_esdf_and_gradient` (the service cuMotion
   queries) returns a 3D ESDF that, in `dynamic` mode, includes appearing/moving obstacles and decays
   them out. A person is handled as a generic dynamic *volume* — routed around at plan time, no
   segmentation model required. This is exactly the "routing aid, never the stop" role.
2. **The people-segmentation path is the wrong tool and its model isn't present.**
   `human_with_static_*` needs `use_segmentation:true` + a mask topic; the **PeopleSemSegNet model is
   NOT baked into the image** (only NGC installer scripts `install_peoplesemsegnet_*.sh` — a download +
   TensorRT build). More importantly, measured on this build, masked-human voxels are **removed from
   the served ESDF** (peak 0–2 voxels), so `human_with_static_*` would make cuMotion *ignore* people —
   the opposite of what is wanted. `dynamic` mode is the correct choice.

---

## Components

Pure (no ROS/GPU, offline-tested):
- `synth_scene.py` — pinhole depth + segmentation-mask renderer of boxes/planes.
- `decay_model.py` — model of nvblox occupancy decay (fresh-not-missed / stale-does-not-linger).
- `routing_gate.py` — the acceptance point; map re-routes, gate alone starts/stops motion.
- `extrinsics_guard.py` — fixed-camera extrinsics with the SAME refuse-until-calibrated guard as
  `cams/human_safety/human_safety/camera_poses.py`.
- `fk_ur10.py` — exact FK from the calibrated URDF (used to place obstacles on the planned TCP path).

ROS nodes / launch (run in the container on domain 85):
- `nvblox_dynamic.yaml` — the `dynamic`-mode overrides (layered like `nvblox_cell.yaml`).
- `dynamic_live.launch.py` — nvblox dynamic (+ optional cuMotion `read_esdf_world`) fed by a depth topic.
- `synth_depth_pub.py` — synthetic depth (+ optional mask, + runtime obstacle control) from a fixed
  provisional camera pose.
- `esdf_probe.py` — query `get_esdf_and_gradient` over an AABB (measurement/assertion tool).
- `publish_extrinsics.py` — publish base_link→camera TFs, refusing uncalibrated real cameras.
- `provisional_extrinsics.yaml` — the fixed-camera entries (sim = provisional; real = refused until calibrated).

## Proofs (self-asserting, logs under `data/plan-nvblox-dynamic-v1/`)

Every GPU run takes the fleet lock: `flock -w 3600 ~/.cache/fleet-gpu.lock <script>`.

- `proof/run_gate_principle_proof.sh` — **NO GPU.** The map re-routes but never turns STOP into motion.
- `proof/run_dynamic_esdf_proof.sh` — **GPU.** A box then a person-sized volume appear in the served
  ESDF and decay after they leave.
- `proof/run_dynamic_route_proof.sh` — **GPU.** cuMotion detours when the obstacle appears and returns
  to the identical baseline route when it leaves; every waypoint stays inside
  `cumotion/measured_joint_limits.yaml`.
- `proof/record_fixed_bag.sh` + `proof/run_replay_realism.sh` — **GPU.** Record ONE short guarded bag
  from a fixed D415 (fuser check, ≤60 s, release) and replay real depth through the same nvblox
  dynamic layer (realism; presence-only because the fixed cameras are uncalibrated).

## Measured (RTX 3060, 12 GB; voice holds ~2.2 GB)

| quantity | value |
|---|---|
| ESDF map-update latency (obstacle present → in served ESDF) | **~0.5–1.0 s** (box appear 0.5 s, clear 1.0 s; person appear <0.5 s, clear 0.6 s) |
| `get_esdf_and_gradient` call latency | **~10 ms median, 13 ms p95** |
| cuMotion planning time against the live dynamic ESDF | **~0.16–0.20 s** per plan |
| route change when obstacle appears / after it leaves | **0.42 m** TCP detour / **0.0 m** (identical to baseline) |
| peak GPU (nvblox + cuMotion, over the voice baseline) | **~3.2–3.4 GB total → ~1.0–1.1 GB for this layer** |

GPU budget check: voice ~2.2 + this layer ~1.1 + the person detector budget (`cams/human_safety`
~1.87 GB) ≈ **5.2 GB of 12 GB** — they co-reside; the 8 GB simulator does not (it is not booted here).

## Reality it is built for (limits)

- **Fixed cameras are uncalibrated.** Neither fixed D415 has a `base_link` pose yet
  (`cams/cellmap/calibrate_static_camera.py` is merged, tags not placed). `extrinsics_guard.py`
  **refuses** a real camera until a passing calibration file exists — real use is presence-only until
  then, exactly like `cams/human_safety`. The synthetic camera pose is a clearly-labelled provisional
  extrinsic (`PROVISIONAL-SIM`), never a survey.
- **Recorded-bag replay is realism, not a metric proof.** Recorded depth *ingestion* is verified (real
  depth flows to nvblox at ~29 Hz); ESDF population from the uncalibrated bag is presence-only /
  inconclusive (a real-camera stamp/sync nuance, not a dynamic-layer defect — the same launch
  populates the ESDF from synthetic depth and drives cuMotion). See `data/.../replay_realism.json`.
- **Not a certified safety function**, and this does **not** change how the real cell launches
  (`cams/cellmap/live.launch.py`, `cumotion/start_cumotion_map.sh` are untouched).
