# dynamic_pick — segmented re-plan + preempt pick controller (SIMULATED arm only)

Phase A of the dynamic-pick design in `~/firstmate/data/human-dynamic-plan-s1/report.md`
(§1 candidate **C1**, §2, §3, §7, §10). It makes the cell able to **follow a moving
pick target**: plan a short leg to the freshest target with the planner, execute it
on `scaled_joint_trajectory_controller`, and **preempt** with a new goal when the
target genuinely moves — all while the merged human-safety gate stays the **single
authority over motion**.

This is the first of the report's three "fast, collision-aware planning" options
(segmented cuMotion re-planning); the other two (cuRobo MPC, an nvblox dynamic
layer) are separate changes.

## Read this first — what it is NOT

**A supplementary capability, NOT a certified safety function.** The camera + CNN
human detector is single-channel and unrated; the real protection remains the CB3
teach-pendant **Force limit** and (owner decision) a safety-rated presence device
(report §0). This layer only lets the arm follow a target and slow/stop/resume
around a person. Dynamic motion makes the certified layer *more* necessary, not
less (report §6/§9).

**SIMULATION ONLY.** Nothing here runs on, arms, or can reach real hardware. It
uses the driver's mock hardware, the URSim CB3 simulator, and the task's OWN
cuMotion container on the isolated task domain **83**. It never starts the cell's
`ur10-driver`, `ur10-cumotion`, `shelfbot_ros`, or `start_cumotion*.sh` (which join
domain 0), and never points anything at `192.168.0.1`.

## The single motion authority (report §2)

The merged gate (`../humangate`, `../humangate_driver`) is unchanged and remains the
sole authority. This controller adds an **acceptance shim**: it is the only thing
that puts a trajectory goal on the wire, and it does so **only while the gate is a
fresh RUN**. The mapping:

| gate | controller |
|---|---|
| **RUN** | forward a new leg to the freshest target |
| **SLOW** | forward NO new leg; the in-flight leg is slowed via the gate's slider |
| **STOP / BACKSTOP** | forward NO leg; the gate freezes/cancels the running leg and holds |
| **KEEP-MOVING** (RUN after the healthy CLEAR dwell) | plan **afresh** to the freshest target (never replay a stale goal) |

The gate driver (slider SLOW, goal-cancel STOP) and the independent watcher run as
their own processes; this controller **never writes the slider and never cancels
for safety**. The invariant — *no tracker/planner input produces motion while the
gate is not RUN* — is proved by an exhaustive unit test over the shim and the
controller core (`tests/`) and re-asserted end-to-end (`sim/assert_dynamic_pick.py`).

## Modules

| File | Role |
|---|---|
| `pick_target_tracker.py` | PURE: single-target constant-velocity Kalman + **deadband/hysteresis committed goal** + explicit **OCCLUDED/LOST** states (fail-toward-hold). Reuses the shape of `cams/human_safety/tracker.py` (report §3). |
| `pick_phases.py` | PURE: FAR/NEAR/GRASP/RETREAT state machine + per-phase re-plan policy (**ALLOWED / LIMITED-to-a-box / FROZEN**) (report §3). |
| `gate_acceptance.py` | PURE: the **acceptance shim** — forward a goal ONLY on a fresh gate RUN (report §2). The load-bearing safety line. |
| `replan_controller_core.py` | PURE: ties tracker + phases + shim into a per-tick **PLAN / NOOP / HOLD** intent; preempts only on a genuine move a phase permits, rate-limited. |
| `planner_port.py` | PURE: `Planner` interface + `AnalyticJointPlanner` (GPU-free, 5% cap, joint-window guard) + `CartesianToJointSurrogate` (SIM stand-in for seeded IK). |
| `cumotion_planner.py` | ROS: the collision-aware `Planner` via cuMotion `move_group` (nvblox ESDF), joint-window guarded exactly as `execute_plan.py`. |
| `segmented_pick_node.py` | the controller node (the dynamic analog of `cams/cellmap/tools/loop_tags.py`): tracker + phases + shim + planner + FJT preemption + latency logging. |
| `params.yaml` | controller params (gate safety thresholds stay in `../humangate/gate_params.yaml`). |
| `sim/` | the end-to-end proofs, scripted moving target + human states, analyzer. |
| `tests/` | offline unit tests for the five pure cores. |

## Offline tests (no ROS, no robot, no docker, no GPU)

```bash
cd dynamic_pick
for t in tests/test_*.py; do python3 "$t"; done      # or: pytest -q tests
```

## End-to-end simulation proofs (repeatable; logs in data/human-dynamic-pick-sim-v1/)

All run on `ROS_DOMAIN_ID=83 FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, start their OWN
driver/planner, stop everything they start BY PID (driver by process group), and
remove only their OWN containers/networks.

```bash
# (a) mock hardware — no docker, no GPU. Proves the whole loop + the gate's
#     fall-through (slider inert in mock, so STOP = goal-cancel backstop).
bash sim/run_mock_e2e.sh

# (b) URSim CB3 in docker — the EFFECTIVE speed slider: SLOW actually slows the
#     leg; command-to-effect delay measurable. Own docker network, no GPU.
bash sim/run_ursim_e2e.sh

# (c) the task's OWN cuMotion container (GPU) — COLLISION-AWARE legs through the
#     nvblox ESDF, on domain 83. Holds the fleet GPU lock for the whole run and
#     records peak GPU. Bring up the planner container is sim/start_cumotion_task.sh.
bash sim/run_cumotion_e2e.sh

# (d) GRASP path (mock hardware) — the target CONVERGES so the tool point enters
#     NEAR then GRASP while it keeps moving; the grasp completes -> begin_retreat ->
#     the FIXED retreat leg. Proves the per-phase re-plan policy holds on the
#     FRESH-leg path too (NEAR limited to the pre-cleared box; GRASP frozen), not
#     only on preemption. Asserted by sim/assert_grasp.py.
bash sim/run_grasp_e2e.sh
```

`sim/assert_dynamic_pick.py` reads the run logs and prints PASS/FAIL plus the
**goal-preemption latency** and the **achievable re-target rate at the 5% cap**,
and exits non-zero if any required behaviour is missing (`sim/assert_grasp.py` does
the same for the GRASP path: FAR/NEAR/GRASP/RETREAT exercised, no PLAN while GRASP,
retreat leg to the fixed pose). Scripted perception is in `sim/scenarios.py` (human
states in the REAL wire format via `../humangate/state_adapter.build_state_json`;
the moving target as a base-frame `PointStamped`).

## Out of scope (report Phase B and later — follow-ups)

The direction-aware "don't drive toward the human" velocity guard and per-mode stop
rings (report §6, Phase B); MoveIt Servo streaming and its servo-mode backstop
(Phase D); the nvblox dynamic layer and cuRobo MPC (the other two planner options);
and any real-robot run (Phase E, owner-present). See the report.

## Decisions + findings

`../data/human-dynamic-pick-sim-v1/decision.md`.
