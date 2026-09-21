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
   over in-view frames; 3-D position & nearest-distance error vs ground truth
   (median / p95 / worst); STOP-ring crossing latency; and — first and loudly —
   any **missed STOP** (person truly inside the STOP ring while the state is not
   STOP beyond the reaction budget).
5. **Report** (`make_report.py`): `report.md` + a self-contained phone-friendly
   `report.html` in the task data dir.

## How to run the real simulation (exact commands)

Everything is orchestrated by one wrapper that takes the GPU lock, boots Isaac
(bundled ROS 2), starts the detector graph + recorder (system ROS 2), runs the
scripted path, and stops everything it started by PID:

```bash
# 1. boot smoke test first (order-of-work step 1): Isaac boots headless and one
#    camera frame reaches the ROS side. ~1 min.
sim/isaac_human/run_boot_smoke.sh

# 2. full session. Default 2 cameras (shelf,table) keeps the RTX renderer and the
#    detectors within the 12 GB card so the img->state latency stays under the
#    0.7 s reaction budget; 3 cameras (wrist,shelf,table) also works but adds GPU
#    contention. device is cuda (default) or cpu.
sim/isaac_human/run_session.sh shelf,table cuda
#   -> prints RUNDIR=<...> ; logs + frames land under ~/.cache/isaac_human_sim/run_*

# 3. analyse the run (offline, no GPU) and build the report.
RUN=<the RUNDIR printed above>
python3 sim/isaac_human/analyze.py --run "$RUN" \
    --config sim/isaac_human/params/human_safety.sim.yaml
python3 sim/isaac_human/make_report.py --run "$RUN" \
    --out /home/manishsaini/firstmate/data/human-detect-isaac-sim-v1
```

Regenerate the camera-pose TF file after editing `cell_layout.py`:

```bash
python3 sim/isaac_human/gen_sim_config.py     # writes params/camera_poses.sim.yaml
```

### Environment / GPU / memory guards

- `isaac_env.sh` sets the conda env (`env_isaaclab`) and the bundled-ROS2-Jazzy
  DDS env for the Isaac process; the detector graph + recorder use system
  `/opt/ros/jazzy`; the two meet over DDS on domain 88.
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
  competes with the detectors for the one GPU, so **3 cameras** can push the
  latency past 0.7 s and pin the state at STOP (masking the distance logic). The
  default run uses **2 cameras** to keep the whole pipeline under the budget.
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

## Files

```
sim/isaac_human/
  isaac_env.sh          conda + bundled-ROS2 DDS env for Isaac (domain 88)
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
  make_report.py        metrics.json -> report.md + self-contained report.html
  params/
    human_safety.sim.yaml   sim config (re-points cameras; safety block verbatim)
    camera_poses.sim.yaml   GENERATED inline base<-optical TFs for the sim cameras
  tests/                offline pytest suite
```
