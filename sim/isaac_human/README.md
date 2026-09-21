# isaac_human — testing the human-detection layer in Isaac Sim

Runs the **existing** `cams/human_safety` detection graph, unchanged in its
decision logic, against RGB-D cameras simulated in **Isaac Sim 5.1** that watch a
rendered person walk toward a simulated UR10 cell and back. It answers, with a
person at a **known** position (impossible on the real rig): does the graph see
the person, how far off is its distance, and does the **CLEAR / SLOW / STOP**
state change where it should — with pictures of what the detector saw.

> **SIMULATION ONLY.** Everything here runs on the isolated `ROS_DOMAIN_ID=88`
> (`ROS_LOCALHOST_ONLY=1`, FastDDS UDPv4) under the fleet GPU lock. It never
> connects to the real robot or its network, never opens the RealSense cameras,
> never touches the `ur10-*` / `shelfbot_ros` containers, and never runs
> `cams/human_safety/run_live.sh`. This layer is **not a certified safety
> function** (see `cams/human_safety/README.md`).

Nothing under `cams/human_safety/human_safety/` (the pure decision logic) is
changed, and **no** safety radius, score threshold, dwell or budget is altered.
The sim config only re-points topics/frames/poses and selects the compute device;
`tests/test_sim_config.py` asserts the `safety:` block is identical to the
production `params/human_safety.yaml`.

## Headline finding (original run) and the fix (`data/human-safety-nearest-point-fix-v1/`)

The **original** run (PR #22, `data/human-detect-isaac-sim-v1/report.md`) found:
the detector **sees** the simulated person on 100% of in-view frames, but the
published STOP-ring **distance overestimated clearance by +0.4–0.9 m in the
UNSAFE direction** (the person reported FARTHER from the robot than truth), so
with a healthy (distance-driven) pipeline STOP engaged only once the person's
nearest body point was already ~0.9 m **inside** the 2.0 m ring — a missed STOP.
Root cause: the zone was sized from the depth-median body **centre**
(`t.position`), which the clean sim floor drags behind the person, while the
accurate `t.nearest_point` was carried but unused; the teleported **T-pose**
(~2 m-wide, background-heavy box) amplified it. Extrinsics were cleared.

**This task fixed it in the production layer** (`cams/human_safety`): the zone now
reacts to the **nearest credible body point** (speckle-filtered, never larger than
the centre), and `geometry.lift_depth` computes the centre from the **foreground
cluster only** so the background floor no longer drags it back. See
`data/human-safety-nearest-point-fix-v1/report.md` for the before/after and
`compare_fix.py` for the old-vs-new replay.

**Natural vs T-pose (harness follow-up, still not built here):** giving the
character a body-filled standing/walking pose needs `omni.anim.people` (a skeletal
animation graph + character-command scripting) — it is **not** a low-cost swap in
this single-12 GB-GPU harness (the RTX renderer already shares the card with the
detector), so the run keeps the teleported T-pose. The fix is validated with the
T-pose *because* it is the harder case: the T-pose maximises the centre bias the
fix must overcome, so passing here is the conservative demonstration.

## What it does

1. **Scene** (`isaac_scene.py`): a small cell — ground, lights, primitive
   box stand-ins for the UR10 base/column, shelf and table — plus a realistic
   human character from Isaac's own People assets (public NVIDIA asset server, no
   login). The character is **teleported** along a scripted straight path (no
   skeletal walk cycle) from ~4.5 m out, through the SLOW band, deep into the
   STOP ring, a pause, then back out. The true root position is recorded every
   frame in `base_link` with a wall-clock stamp (`ground_truth.jsonl`).
2. **Cameras**: 2–3 simulated RGB-D cameras (wrist / shelf / table stand-ins,
   640×480, ~69° HFOV) publish exactly what the detector consumes — `rgb8`
   colour, `32FC1`-metres aligned depth, `CameraInfo`, and a static
   `base_link ← optical_frame` TF. The camera pose, the TF handed to the detector
   and the analysis projection all come from ONE look-at (`sim_poses.py` +
   `cell_layout.py`), so the detector's TF is exactly the pose Isaac renders
   from. Images are stamped with **system (wall) time** (`useSystemTime=True`) so
   the detector's `latency = now − image_stamp` is meaningful.
3. **The real graph** (`cams/human_safety/human_safety.launch.py`) runs with the
   sim config; `recorder_node.py` logs `/human_safety/state`, tracks and
   detections plus saves annotated colour frames (detector boxes + keypoints).
4. **Judge** (`analyze.py` + pure `analysis_core.py`): detection rate per camera
   over in-view frames; position & nearest-distance error vs ground truth
   (median / p95 / worst); STOP-ring crossing latency; and — first and loudly —
   any **missed STOP** (a true body point inside the STOP ring while the state is
   not STOP beyond the reaction budget; uses the recorded nearest body point).
5. **Diagnose** (`diagnose.py`): the signed distance error (published − true) vs
   true distance against three truth references (root / centroid / nearest body),
   the per-camera body-centre-vs-nearest split, and the true distance at which a
   distance-driven STOP actually fires. Writes `diagnosis.json` + `signed_error.png`.
6. **Report** (`make_report.py`): `report.md` + a self-contained phone-friendly
   `report.html` in the task data dir, verdict first.

## How to run the real simulation (exact commands)

Everything is orchestrated by one wrapper that takes the GPU lock, boots Isaac
(bundled ROS 2), starts the detector graph + recorder (system ROS 2), runs the
scripted path, and stops everything it started by PID:

```bash
# 1. boot smoke test first (order-of-work step 1): Isaac boots headless and one
#    camera frame reaches the ROS side. ~1 min.
sim/isaac_human/run_boot_smoke.sh

# 2. full session. Default is a SINGLE camera (wrist): on this shared 12 GB card
#    only one camera keeps the RTX renderer + the detector within budget so the
#    img->state latency stays under the 0.70 s reaction budget. 2+ cameras run the
#    latency PAST the budget (fail-safe STOP, masks the distance behaviour) and the
#    script warns; a 2-camera run is only for the per-camera accuracy split.
#    device is cuda (default) or cpu.
sim/isaac_human/run_session.sh wrist cuda
#   -> prints RUNDIR=<...> ; logs + frames land under ~/.cache/isaac_human_sim/run_*

# 3. analyse the run (offline, no GPU), diagnose the distance bias, build report.
RUN=<the RUNDIR printed above>
python3 sim/isaac_human/analyze.py --run "$RUN" \
    --config sim/isaac_human/params/human_safety.sim.yaml
python3 sim/isaac_human/diagnose.py --run "$RUN"
python3 sim/isaac_human/make_report.py --run "$RUN" \
    --per-camera-run <a 2-camera RUNDIR, optional, for the shelf/wrist split> \
    --out /home/manishsaini/firstmate/data/human-detect-isaac-sim-v1

# 4. old-vs-new before/after for the nearest-point safety fix, from the SAME run
#    (no second sim run): replays the recorded live states through the safety
#    machine with old (centre) vs new (nearest) logic. Prints zone changes, STOP
#    time, missed STOPs and signed distance error for each; a fidelity line
#    confirms the NEW reconstruction reproduces the live zone stream.
python3 sim/isaac_human/compare_fix.py --run "$RUN" --json "$RUN/compare_fix.json"
```

**A single camera** (e.g. `run_session.sh wrist cuda`) keeps the image→state
latency inside the 0.70 s reaction budget on this shared 12 GB card, so the
SLOW/STOP changes are **distance-driven** rather than pinned at STOP by the
latency fault — that is the run the published report uses. A 2-camera run gives
the per-camera accuracy split but runs the latency past the budget.

Regenerate the camera-pose TF file after editing `cell_layout.py`:

```bash
python3 sim/isaac_human/gen_sim_config.py     # writes params/camera_poses.sim.yaml
```

### Environment / GPU / memory guards

- `isaac_env.sh` sets the conda env (`env_isaaclab`) and the bundled-ROS2-Jazzy
  DDS env for the Isaac process; the detector graph + recorder use system
  `/opt/ros/jazzy`; the two meet over DDS on domain 88.
- **Domain guard** (`domain_guard.sh`, sourced first by `isaac_env.sh`): defaults
  `ROS_DOMAIN_ID` to the SIM-only 88 and **refuses** to run on 0 (real robot),
  42 (`shelfbot_ros` container) or 77 (the live human_safety camera graph), so an
  ambient `ROS_DOMAIN_ID` in the shell can never cross this harness into a live
  graph. Unit-tested standalone (`tests/test_domain_guard.py`).
- `preflight.sh` refuses to boot unless ≥ 8 GB GPU free and ≥ 10 GB RAM are
  available, and stops if the **Blender application** is running (the
  `blender-mcp` bridge is not Blender and is ignored). A RAM watchdog aborts the
  run by PID if available RAM falls under 3 GB. Each session is capped well under
  20 minutes of wall clock.
- The detector runs on the GPU by default. If Isaac + 3 detectors do not fit,
  pass `cpu` as the device argument (sim config only) — it is slower, so the
  reported latency will be pessimistic.

## Design notes / sharp edges

- **Latency vs the reaction budget.** The detector forces STOP when
  `img→state latency > reaction_time_budget` (0.7 s). Isaac's RTX rendering
  competes with the detectors for the one GPU, so **2+ cameras** push the
  latency past 0.7 s and pin the state at STOP (masking the distance logic). The
  default run uses a **single camera** to keep the whole pipeline under the budget.
  This is a property of sharing one 12 GB card, not of the detector — the real
  rig (cameras on USB, no renderer competing) measured ~0.26 s.
- **SLOW is presence-driven, not a distance ring.** `slow_on_any_person` (small
  cell SSM) means any visible person forces ≥ SLOW, so CLEAR→SLOW happens at
  first detection and CLEAR returns only when nobody is detected. The
  quantitative boundary the analysis validates is therefore the **STOP ring**
  (2.0 m of base); CLEAR/SLOW are reported descriptively.
- **Wall-clock ground truth.** The person's position is recorded with a wall
  stamp each frame, regardless of Isaac's tick rate, so the analysis time-aligns
  ground truth, detections and state on one clock. Position error is aligned to
  each state's `evidence_stamp` (the image that produced it), so pipeline latency
  does not masquerade as position error.

## Tests (offline, no simulator)

```bash
python3 -m pytest sim/isaac_human/tests/ -q
```

- `test_analysis_core.py` — the analysis on synthetic logs: perfect tracking, a
  constant offset, a missed STOP (loud), a late-but-acceptable transition,
  in-view-only detection rate.
- `test_sim_config.py` — the sim config loads through the real human_safety
  loader, changes no safety threshold vs the production config, and its generated
  TF file is in sync with `cell_layout`.
- `test_sim_poses.py` — the TF handed to the detector round-trips through
  `human_safety.geometry` back to the true base point (so the pose the detector
  uses is exactly the pose Isaac renders from).
- `test_domain_guard.py` — the domain guard defaults to 88 and refuses 0/42/77.
- `test_compare_fix.py` — the old-vs-new replay reconstructs zones from recorded
  states, using the nearest point (new) vs the centre (old).

## Files

```
sim/isaac_human/
  isaac_env.sh          conda + bundled-ROS2 DDS env for Isaac (domain 88)
  domain_guard.sh       refuse ROS_DOMAIN_ID 0/42/77, default 88 (sourced first)
  preflight.sh          GPU/RAM/Blender guard + RAM watchdog
  run_boot_smoke.sh     order-of-work step 1: headless boot + 1 frame on ROS
  boot_smoke_test.py    Isaac side of the boot smoke test
  ros_frame_probe.py    system-ROS frame counter (proves cross-process DDS)
  run_session.sh        full session: Isaac + real graph + recorder, one GPU lock
  isaac_scene.py        the scene: cell, cameras, person, ground-truth recording
  recorder_node.py      logs state/tracks/detections + annotated frames (venv)
  sim_poses.py          PURE pose maths (look-at -> USD xform + base<-optical TF)
  cell_layout.py        PURE scene geometry: cameras + person path (source of truth)
  gen_sim_config.py     regenerate params/camera_poses.sim.yaml from cell_layout
  analysis_core.py      PURE judgement (unit-tested)
  analyze.py            run the analysis + plots -> metrics.json + png
  diagnose.py           signed distance-error diagnosis -> diagnosis.json + png
  make_report.py        metrics + diagnosis -> report.md + self-contained report.html
  compare_fix.py        old-vs-new (centre vs nearest) before/after from one run
  params/
    human_safety.sim.yaml   sim config (re-points cameras; safety block verbatim)
    camera_poses.sim.yaml   GENERATED inline base<-optical TFs for the sim cameras
  tests/                offline pytest suite
```
