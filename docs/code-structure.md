# Code structure

A directory- and module-level map so you can find anything without reading code first. Every
path below exists in the repository; each subsystem has its own `README.md` with full detail.

## Top level

| Path | What it is |
|---|---|
| `README.md` | Whole-cell overview (start here) |
| `RUNBOOK.md` | Operator commands, health checks, recovery — authoritative day-to-day |
| `AGENTS.md` | Project memory: build/test/architecture sharp-edge notes for contributors |
| `ur-bringup.sh` | Launch the UR driver (`ur_control.launch.py`, headless) |
| `install-ur-driver.sh` | Pinned apt install that defeats the Isaac-repo `99.x` version override |
| `ur-detect.sh` | Ping + dashboard probe; detects CB3 vs e-Series, writes `ur_type.txt` |
| `ur-rearm-watchdog.sh` | Re-assert the control program after a network blip |
| `motion_test.py`, `motion_test_visible.py` | Self-verifying ±0.05 rad motion checks |
| `nudge_wrist3.py` | First-light ±0.10 rad motion script |
| `ur10_calibration.yaml` | This arm's extracted factory kinematics |
| `report.html`, `torque-report.html` | Commissioning reports (browser; mirror the runbook) |
| `moveit/`, `rviz/` | MoveIt launch bits and RViz configs |
| `docs/` | This documentation set + `docs/media/` demo artifacts and their generator |
| `data/` | Per-task decision/report records — the "why" behind each change |

## `fork_demo/` — real-arm scripted fork tray-pick (plan-only by default)

| Module | Role |
|---|---|
| `fork_pick_demo.py` | Builds the base_link TCP cycle (HOME→APPROACH→INSERT→LIFT→CARRY→PLACE); `--dry-run` plots it; the execute path has a multi-gate typed confirm |
| `check_joint_feed_live.py` | Jitter gate — flags a frozen re-published RTDE joint feed |

## `cams/` — cameras: bring-up, mapping, calibration, human perception

| Path | Role |
|---|---|
| `README.md` | Camera bring-up traps (serials, upside-down wrist, USB faults, stale ros2 daemon) |
| `rig.launch.py`, `rig_multicam.launch.py` | Multi-camera RealSense bring-up |
| `params/` | Per-camera YAML (serial-pinned RealSense configs) |
| `approach/` | Seeded-IK approach demo (`approach_demo.py`) for predictable motion vs MoveIt sampling |
| `tools/` | Frame-grab / rig-feed / image-rotation utilities |
| **`cellmap/`** | nvblox mapping, AprilTag calibration, plan/execute tooling (see below) |
| **`human_safety/`** | Camera-side human detection + RUN/SLOW/STOP state (see below) |

### `cams/cellmap/`

| Module | Role |
|---|---|
| `calibrate_static_camera.py` | Fixed-camera → base_link via AprilTag colour-PnP |
| `recheck_static_camera.py` | Moved-camera guard (invalidates a stale calibration) |
| `handeye_calibrate.py` | Wrist eye-in-hand hand-eye calibration |
| `staticcam.py` | Pure, unit-tested calibration geometry |
| `live.launch.py`, `scan.launch.py`, `planner.launch.py` | nvblox mapping + planner launch |
| `tools/execute_plan.py` | **The motion chokepoint** — runs a plan with `--limits`/`--exact` guards |
| `tools/loop_tags.py` | Tag-following pick loop (the cell's running controller) |
| `tools/record_joint_limits.py` | Hand-sweep / declare real joint limits |
| `tools/map_ctl.py`, `tools/nvblx_clean.py`, `tools/verify_map.py` | nvblox map save/clean/verify |
| `tools/collision_force.py` | ROS-side gross-contact backstop |
| `dynamic/` | nvblox **dynamic** ESDF layer feeding cuMotion (sim-only); `routing_gate.py` enforces "map routes, never stops" |

### `cams/human_safety/`

| Path | Role |
|---|---|
| `human_safety/detector.py` | Keypoint R-CNN wrapper (`build_detector("keypoint_rcnn", …)`) |
| `human_safety/tracker.py` | Person tracker with hysteresis / OCCLUDED-LOST states |
| `human_safety/geometry.py`, `lifting.py` | Image→base_link geometry, depth lift to nearest body point |
| `human_safety/safety.py` | `SafetyEvaluator` — zones, STOP trigger + `stop_margin` |
| `human_safety/fusion.py` | Fuse per-camera detections into one state |
| `human_safety/scheduler.py`, `frameselect.py`, `frameproc_eval.py` | Optional efficient frame path |
| `human_safety/json_util.py`, `messages.py`, `config.py` | Versioned JSON wire format + config |
| `nodes/detector_node.py`, `fusion_node.py`, `viz_node.py`, `shared_detector_node.py` | Thin ROS shells |
| `tests/` | Offline unit tests for every pure module (ROS/torch-free) |

## `cumotion/` — limits and collision config

| Module | Role |
|---|---|
| `measured_joint_limits.yaml` | Real hand-swept joint travel windows (every executed plan checks these) |
| `make_limited_urdf.py` | Build a limit-clamped URDF with world collision |
| `collision_force_limits.yaml` | ROS-side gross-contact backstop config |
| `maps/` | nvblox cell maps (large ones live on the host) |

## Safety & planning packages

| Package | Role | Domain |
|---|---|---|
| `humangate/` | Robot-side safety **decision core**, PLAN-ONLY (`gate_core.py`, `state_adapter.py`); `test_no_driver_interface.py` proves no driver import | 78 |
| `humangate_driver/` | Driver-backed gate, **simulated arm only**: speed-slider SLOW + goal-cancel STOP; `arming_interlock.py` refuses the real robot | 79 |
| `dynamic_pick/` | Moving-target picking via segmented cuMotion re-plan + preempt; `gate_acceptance.py` shim; `pick_target_tracker.py`, `pick_phases.py`, `segmented_pick_node.py` | 83 |
| `mpc_curobo/` | cuRobo V2 reactive-MPC prototype (own CUDA+Humble container); `mpc_prototype.py`, `ros_bridge/mpc_gate_bridge.py` | 84 |

## `sim/isaac_human/` — Isaac Sim ground-truth test harness

| Module | Role |
|---|---|
| `isaac_scene.py`, `gen_sim_config.py` | Build the scene + generate camera-pose config from the cell layout |
| `recorder_node.py`, `record_health.py` | Capture the run with latency health |
| `analyze.py`, `diagnose.py`, `compare_fix.py`, `make_report.py` | Offline accuracy analysis |
| `make_video.py`, `live_view.py` | Record a phone-playable clip / WebRTC live view |
| `run_boot_smoke.sh`, `run_session.sh`, `run_live_view.sh` | Entry points |
| `tests/` | Offline tests incl. `test_after_run_replay.py` (guards the nearest-point fix + ring) |

## `data/` — decision and report records

One directory per task, each with a `decision.md` and/or `report.md` capturing the reasoning,
measurements, and before/after for a change. These are the authoritative rationale for every
tuning choice referenced from the READMEs (e.g. `planner-map-fix-v1`, `collision-force-v1`,
`human-safety-stop-margin-v1`, `ur10-fork-demo-readiness-v1`).
