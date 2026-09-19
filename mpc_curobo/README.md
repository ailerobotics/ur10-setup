# mpc_curobo — cuRobo model-predictive-control prototype for the cell UR10

**SIMULATION / EVALUATION ONLY.** Option (2)/candidate **C3** of the
human-dynamic-plan-s1 investigation: NVIDIA cuRobo's reactive **MPC** follow-and-
avoid mode, which NVIDIA's `isaac_ros_cumotion` ROS package does not expose and
which needs a CUDA toolkit absent on this host. Runs on `ROS_DOMAIN_ID=84`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Never touches the real robot, the cell
containers, or Isaac Sim.

Full write-up, measured numbers, comparison and the owner licence decision:
`~/firstmate/data/plan-mpc-curobo-v1/report.md` and `decision.md`.

## LICENCE (read first)

cuRobo **v1** (`v0.7.x`, where the MPC docs live) is under the **NVIDIA
non-commercial** Source Code License. cuRobo **V2** (`v0.8.0`, Apache-2.0) is
commercial-OK **and still exposes MPC**. This prototype pins **v0.8.0
(Apache-2.0)**. It is evaluation-grade and **not a certified safety function**.
cuRobo is deliberately kept OUT of any shipped dependency list pending the owner
decision (report §0, `decision.md` D1).

## Layout

- `Dockerfile`, `build_container.sh` — the CUDA-devel + ROS 2 Humble + UR driver +
  torch + cuRobo V2 image (`fm-plan-mpc-curobo-v1`, ~8 GB, ~10 min build).
- `robot/make_curobo_robot.py` → `robot/ur10_cell_curobo.yml` — cuRobo robot config
  built from the cell's measured `cumotion/ur10_measured.{urdf,xrdf}` +
  `measured_joint_limits.yaml` (cuRobo reads the captain windows from the URDF).
- `robot/joint_windows.py` — independent, **wrap-aware** measured-window guard (pure).
- `world/cell_world.py`, `world/cell_scene.yml` — ground + representative shelf +
  a box standing in for a person; analytic clearance metric.
- `mpc_prototype.py` — reach-and-follow prototype; scenarios `follow|person|
  unreachable|blocked`; measures control rate, warm solve time, peak GPU, tracking,
  clearance, window violations.
- `bench/compare_c1_c3.py` — option (1) segmented re-plan (cuRobo `MotionPlanner` ==
  cuMotion trajopt) side of the comparison.
- `ros_bridge/stream_gate.py` — pure gate **acceptance shim** (forward only on RUN,
  hold otherwise, window + step guards).
- `ros_bridge/mpc_gate_bridge.py` — ROS 2 node: gate-mediated MPC stream to the UR
  driver's `forward_position_controller`, behind the **unchanged** arming interlock.
- `sim/` — end-to-end proofs (mock hardware + CB3 URSim) and assertions.
- `tests/` — offline unit tests (no GPU/ROS).

## What was proven (report §2–4)

- MPC on the RTX 3060: **~55–59 Hz**, **~17 ms** warm solve, **~215 MiB** GPU;
  follows the target (3.8 cm), stays bounded when the target is unreachable, keeps
  positive clearance to a person when the path is blocked, **0 measured-window
  violations**. Honest negative: a fast-moving person can be transiently intruded
  on (soft collision cost) — MPC alone is not a safety function.
- ROS bridge (mock hardware): the arm moves only while the gate is **RUN**, holds
  on SLOW/STOP/BACKSTOP, **resumes** toward the freshest goal, and on
  **command-stream starvation** (the **bridge** killed mid-motion, **driver still
  alive**) the arm **ends stopped** — `forward_position_controller` holds its last
  setpoint (answers RD-3 for the stream-death case). **This is not "the bridge
  alone guarantees a stop":** the hold depends on the driver being alive; if the
  **driver** itself dies, the stop comes from a separate layer — the robot's
  external-control program ending. `forward_velocity_controller` would coast on
  starvation. Live cuRobo MPC in the loop passes too. CB3 URSim: driver connects + controller
  activates + gate streams, but the CB3 sim arm did not register motion under
  headless servoj streaming (documented gap; mock is the faithful proof).
- Comparison: C1 re-plan ~77 ms/leg (~50 % single-shot pose-goal success without
  seeded IK) vs C3 MPC ~17 ms/step continuous with graceful degradation; MPC is
  the cheapest of the three on VRAM and fits beside voice (2.2 GB) + person
  detection (1.9 GB) on the 12 GB card.

## Reproduce

```bash
mpc_curobo/build_container.sh
# metrics (hold the fleet GPU lock; --user to avoid root-owned outputs):
flock -w 3600 ~/.cache/fleet-gpu.lock docker run --rm --gpus all \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/repo:ro -v ~/firstmate/data/plan-mpc-curobo-v1:/out:rw \
  fm-plan-mpc-curobo-v1:latest \
  python3 /repo/mpc_curobo/mpc_prototype.py --scenario follow --out /out/runs/mpc_follow.json
# ROS bridge (mock, no GPU):  docker run --rm --user ... bash /repo/mpc_curobo/sim/run_mock_e2e.sh
# live MPC in the loop (GPU): docker run --rm --gpus all --user ... bash /repo/mpc_curobo/sim/run_live_mpc_e2e.sh
# CB3 URSim:                  bash mpc_curobo/sim/run_ursim_e2e_host.sh
# offline unit tests:         python3 -m pytest mpc_curobo/tests -q
```

## Sharp edges

- cuRobo V2 config loader reads YAML as **ASCII**; the generated robot config is
  ASCII-only. Reads of the cell's UTF-8 YAML use explicit `encoding="utf-8"`.
- The cell keeps `wrist_1` **unwrapped** in [2.84, 6.08]; cuRobo/ROS emit the
  2π-equivalent. The window guard and the shim are **wrap-aware** — don't compare
  revolute angles literally.
- The MPC goal must be **re-asserted every tick** (the source does this); setting
  it once and advancing the state leaves `position_error` at 0 and the arm frozen.
- The live-MPC bridge needs a **multi-threaded executor** so the ~17 ms solve does
  not starve the gate heartbeat (→ spurious STOP).
- Containers are `fm-plan-mpc-curobo-v1*`, ports on 127.0.0.1 only, removed on
  exit. Never reuse `ur10-cumotion`/`shelfbot_ros`; never start `start_cumotion*.sh`.
