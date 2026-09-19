# Dynamic pick — segmented cuMotion re-plan + preempt (Phase A) — decisions

Task `human-dynamic-pick-sim-v1`. This is **option (1) of three** the owner asked
for ("build all three top options"): segmented re-planning with NVIDIA cuMotion.
It is **Phase A** of the design in `~/firstmate/data/human-dynamic-plan-s1/report.md`
(§1 candidate **C1**, §2, §3, §7, §10). Options (2) cuRobo MPC and (3) the nvblox
dynamic layer are separate changes.

**Scope: SIMULATION ONLY.** No real robot is started, armed, or reachable. Proven
on the driver's mock hardware, the URSim CB3 simulator, and the task's OWN cuMotion
container, all on the isolated task domain **83**. It never runs the cell's
`ur10-driver`/`ur10-cumotion`/`shelfbot_ros`/`start_cumotion*.sh` (domain 0) and
never points at `192.168.0.1`. This is **not** a certified safety function (report
§0); dynamic motion makes the certified layer *more* necessary (report §6/§9).

## What was built (report §10 Phase A)

New package `dynamic_pick/` (sibling of `humangate_driver/`, so the plan-only
`humangate` stays plan-only):

1. **Pick-target tracker** (`pick_target_tracker.py`) — a single-target
   constant-velocity Kalman filter reusing the shape of
   `cams/human_safety/tracker.py`, extended with a **deadband + sustain
   (hysteresis) committed goal** (the goal only moves when the estimate shifts
   beyond 3 cm and stays shifted for N updates — so the arm chases the target, not
   the 0.1-0.4 cm bench jitter) and explicit **OCCLUDED/LOST** states that
   fail-toward-hold (never extrapolate a phantom the arm would drive toward).
2. **Pick phases** (`pick_phases.py`) — FAR/NEAR/GRASP/RETREAT with the report's
   per-phase re-plan policy: FAR **allowed**, NEAR **limited to a pre-cleared box**,
   GRASP/RETREAT **frozen**.
3. **Gate acceptance shim** (`gate_acceptance.py`) — the single motion-authority
   mediator: the controller may put a goal on the wire ONLY through this shim, and
   it forwards a new goal **only on a fresh gate RUN**. SLOW/STOP/BACKSTOP/stale/absent
   all HOLD. This is the line that makes "a goal arriving while the gate is not RUN
   must not move the arm" true by construction.
4. **Segmented re-plan/preempt controller** (`replan_controller_core.py` pure +
   `segmented_pick_node.py` ROS) — the dynamic analog of `loop_tags.py`: plan a
   short leg to the freshest committed target, execute on
   `scaled_joint_trajectory_controller`, and **preempt** with a new FJT goal when
   the target genuinely moved (deadband-crossed) and the phase policy permits,
   rate-limited. Every leg is validated inside the MEASURED joint windows exactly as
   `execute_plan.py`.
5. **Planner port** (`planner_port.py`, `cumotion_planner.py`) — the `AnalyticJointPlanner`
   (GPU-free, 5 % cap, joint-window guard) drives the mock/URSim proofs; the
   `CuMotionPlanner` plans **collision-aware** legs through the nvblox ESDF for the
   cuMotion proof. Same interface; the controller is planner-agnostic.

## Key decisions

- **C1 (segmented re-plan/preempt), not Servo or MPC.** Per report §8 this is the
  only option deliverable now: it reuses the proven FJT goal-cancel terminal hold
  and every existing guard, is collision-aware through the full nvblox ESDF, and
  fits the observe-while-stopped perception reality. Servo (C2) and MPC (C3) carry
  the unresolved backstop-gap / no-official-ROS risks and stay later phases.
- **The gate stays the single authority, unchanged.** `humangate`/`humangate_driver`
  are reused verbatim; this controller adds only the *acceptance* line (lever 2 of
  report §2). It never writes the slider and never cancels for safety — the gate
  driver (slider SLOW) and the independent watcher (goal-cancel STOP) do, as their
  own processes. The controller reads the gate's published `/human_gate/decision`.
- **SLOW holds new goals.** Per the rule set, a new leg is forwarded only on RUN;
  under SLOW the in-flight leg is slowed via the gate's slider and no NEW leg is
  started (we do not begin fresh motion toward a possibly-moving target while a
  person is near). KEEP-MOVING = the gate returning to RUN after its healthy CLEAR
  dwell; the controller then plans **afresh** to the freshest target, never a stale
  goal.
- **Cartesian target -> joint goal is a SIM surrogate for seeded IK.**
  `CartesianToJointSurrogate` is a deterministic clamped linear map, NOT kinematics;
  it exists only so the sim arm's joints move coherently with the scripted Cartesian
  target. On the real cell this is the seeded `/compute_ik` step (AGENTS.md); the
  controller and the joint-window guard are unchanged.
- **Wrapping-aware joint-window guard.** The sim arm boots at a home whose
  continuous joints (wrist_1 = -1.57 rad) sit a whole 2π turn from the measured
  window branch (centre 4.459 rad) though they are the SAME physical angle. The
  guard checks the physical angle (any 2π branch) and commands the goal on the
  branch nearest the live pose (shortest travel) — identical to the cell on the real
  arm (whose reported values already sit in the window branch), and it never
  loosens the window.
- **Sim bootstrap `prepose`.** The real cell always starts parked inside its
  measured windows; the mock/URSim arm boots at a factory home outside them (for
  wrist_1). `prepose.py` parks the sim arm at the window centre once at startup —
  the analog of URSim's power-on/brake-release, not part of the pick loop.

## Proven in simulation — all three rigs 13/13 (logs under this directory)

| proof | what it uniquely shows | measured result |
|---|---|---|
| `mock/ASSERTIONS.txt` | the full control loop + gate sole authority; STOP effected by the goal-cancel backstop (mock slider inert) | 13/13; controller followed the target, held on STOP/occlusion, resumed to the freshest target |
| `ursim/ASSERTIONS.txt` | real controller timing — a long leg is genuinely running when the target moves, so PREEMPTION fires | 13/13; **5 preemptions**, goal-preemption latency **median 1.5 ms** (1.4–2.1), **re-target rate 0.276 Hz** at the 5 % cap |
| `cumotion/ASSERTIONS.txt` | COLLISION-AWARE legs through the nvblox ESDF via the task's OWN cuMotion container on domain 83 | 13/13; **53 collision-aware legs planned**, cuMotion solve latency **median 206 ms** (169–394 ms wall), **peak GPU 8425 MiB** |

### Measured figures (matching the report)

- **cuMotion solve latency ~0.17–0.39 s wall** (median 206 ms) — consistent with the
  report's measured 0.25–0.48 s wall / 0.10–0.14 s solve; the often-quoted 27 s is
  trajectory EXECUTION at the 5 % cap, not planning (report §0).
- **Goal-preemption latency median 1.5 ms** (URSim, analytic planner: decide → new
  goal accepted, preempting the running leg). With cuMotion the preemption cost is
  dominated by the ~206 ms solve.
- **Achievable re-target rate 0.276 Hz** at the 5 % cap — bounded by execution speed
  + the target's own cadence, NOT by plan time (report §1: "~1–3 Hz re-target,
  bounded by 5 % exec speed + preempt latency, not plan time"). A higher speed cap
  (owner decision, §11.2) would raise it.
- **Rig note:** the mock and cuMotion rigs execute on the MOCK arm, which completes a
  trajectory goal instantly (like its inert slider), so a leg does not outlive the
  next target move and preemption there is a timing artefact — preemption is proven
  on URSim (real timing) and by the offline unit test; on the mock-arm rigs the
  assertion is informational. The cuMotion rig's unique contribution is the
  collision-aware PLAN + its solve latency, not real-time execution.

The load-bearing safety invariant is asserted in every run: **every trajectory
goal was forwarded ONLY under a fresh gate RUN, and a target moving while the gate
is STOP produces no motion** — proved exhaustively offline
(`tests/test_gate_acceptance.py`, `tests/test_replan_controller.py`) and
re-asserted end-to-end (`sim/assert_dynamic_pick.py`).

## Notable bugs found + fixed during bring-up (recorded so they are not re-hit)

- **Re-entrant executor**: a timer-driven node cannot call
  `rclpy.spin_until_future_complete` on the global executor (it is "already
  spinning" under `rclpy.spin`). The cuMotion planner runs on its OWN node + a
  dedicated `SingleThreadedExecutor` (`cumotion_planner.py`).
- **Joint-branch wrapping**: the sim arm boots at wrist_1 = −1.57 rad, the same
  physical angle as the measured window's 4.71 rad branch. The window guard is
  wrapping-aware and prepose parks in the MEASURED branch so cuMotion's URDF/XRDF
  start-state check passes (`INVALID_INITIAL_CSPACE` otherwise).
- **cuMotion start-from-park**: the standard `cell-*.nvblx` carries one stray
  weight-1 voxel that blocks planning from the park region; the cuMotion proof uses
  the despeckled `parkpose-fix-…-despeck.nvblx` (AGENTS.md).
- **Leg-running by duration, not the async result**: `leg_running` is timed by the
  planned leg duration (not the FollowJointTrajectory result callback, which races
  with accept/preempt ordering), so preemption is deterministic on a real-timed arm.

## Out of scope — follow-ups (report Phase B and later)

- **Direction-aware "don't drive toward the human" velocity guard + per-mode stop
  rings** (report §6, Phase B) — the safety upgrade dynamic motion needs; pure and
  offline-testable, not built here.
- **Servoed near approach + servo-mode backstop** (Phase D, gated on the D415
  calibration) — closes RD-1/RD-3.
- **The other two planner options**: cuRobo MPC (needs a CUDA toolkit absent on the
  host) and the nvblox dynamic layer feeding the planner.
- **Owner-present live commissioning** (Phase E) — measure Tr/Ts at dynamic speed,
  re-tighten rings. Real-arm arming is a later, reviewed change.

## GPU discipline

The cuMotion proof holds the fleet GPU lock (`flock ~/.cache/fleet-gpu.lock`) for
the whole run, well under the 30-min cap, records `nvidia-smi` before + peak, and
removes its own container on exit. Baseline at build time: 12 GB total, ~2.3 GB
used (voice), ~9.6 GB free.
