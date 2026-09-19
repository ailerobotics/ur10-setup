# human-gate-sim-v1 — driver-backed gate on a SIMULATED arm (report §10 Phase 3)

This is Phase 3 of the robot-side human-safety gate designed in
`~/firstmate/data/human-safety-s1/report.md`. Phase 1 (`humangate/`) shipped the
plan-only decision core; Phase 3 connects that core to the real driver levers and
proves the behaviour end to end **against a simulated arm only** — the driver's
mock hardware and the Universal Robots CB3 simulator (URSim). Running on the real
arm is a later, separate, owner-present step (Phase 4; see `LIVE_TEST.md`).

**This is a supplementary awareness layer, NOT a certified safety function.** A
camera + CNN detector is single-channel, unrated, no diagnostic coverage; it
cannot be ISO 13849 PL d / Cat 3 and must never be relied on as the protective
stop. The real protection remains the CB3 teach-pendant Force limit and (owner
decision) a safety-rated presence device wired to the safeguard-stop input
(report §0/§4). This layer only makes the arm slow/stop early and resume when
clear, so the certified stops rarely have to fire.

## Why speed scaling is the primary lever and cancel is the backstop

From the design mechanism evaluation (report §2), verified here against URSim:

- **Speed scaling via `set_speed_slider` (`ur_msgs/srv/SetSpeedSliderFraction`,
  field `speed_slider_fraction`) is the PRIMARY lever.** Lowering it slows the
  running trajectory; zero freezes the arm on its planned path at its current
  location; raising it resumes the SAME trajectory with no replan — the literal
  match to the captain's ask. It does not drop the external-control program or
  deactivate controllers. Confirmed on the scaling topic
  `/speed_scaling_state_broadcaster/speed_scaling` (`std_msgs/Float64`).
- **Cancelling the active `FollowJointTrajectory` goal is the INDEPENDENT
  backstop.** It is terminal (a resume needs a fresh plan) and is driven by a
  separate watcher process. We cancel via the action's generic cancel service
  (`action_msgs/srv/CancelGoal`, zero goal-id + zero stamp = cancel all), so the
  gate/watcher can cancel a trajectory they did NOT start — the supervisor
  relationship the cell already uses (`cams/cellmap/tools/execute_plan.py`).

## Package layout and the plan-only guarantee

The driver-touching code lives in a NEW sibling package `humangate_driver/`, NOT
inside `humangate/`. This is deliberate: `humangate/test_no_driver_interface.py`
proves the `humangate/` package imports NO driver interface (`control_msgs`,
`ur_msgs`, `ur_dashboard_msgs`, no dashboard client, no
`FollowJointTrajectory`/`SetSpeedSliderFraction` identifiers). Keeping the driver
coupling in a separate package preserves that Phase-1 guarantee intact while
adding the Phase-3 wiring. The gate core (`humangate.gate_core.HumanGate`),
config, and command-port abstraction are reused unchanged; the driver port
implements the same abstract `CommandPort` and `DriverFeedback` bases.

## The camera-side message: adapter now parses the REAL wire format

The perception package `cams/human_safety` is committed and publishes its safety
state on `std_msgs/String` topic `/human_safety/state` as the JSON produced by
`cams/human_safety/human_safety/json_util.py::state_to_json` (it uses a String
JSON payload by design — that package is not colcon-built, so no custom .msg).
The Phase-1 adapter had been written against report §7 with a *provisional*
placeholder schema. `humangate/state_adapter.py` was rewritten to parse the REAL
message. Key field mappings (all safety-critical):

- **Staleness stamp.** The gate ages against `evidence_stamp` (the oldest
  positioned-detection stamp that sizes the STOP ring) when a person is being
  localised, and falls back to the publication `stamp` on CLEAR / presence-only
  cycles where `evidence_stamp` is null; message recency is then guaranteed by
  the arrival-based heartbeat watchdog. A latched/republished frame keeps its old
  evidence stamp and so cannot look fresh.
- **Distance honesty.** `nearest_distance_m` is populated only when
  `distance_status == "known"` (from `nearest_distance_base`); "unknown"
  (presence-only) and "none" leave it None. The gate never treats unknown as
  "far".
- **Health.** A single `health_ok` is derived from the per-camera booleans
  (`present_in_graph`/`detector_alive`/`fresh`); a CLEAR zone alongside a
  stale/absent camera is an internal contradiction that forces STOP. Only the
  camera side's "lost while close" reason maps to the `TRACK_LOST_CLOSE` flag the
  gate special-cases; the many narration reasons are NOT turned into flags.
- **Schema version.** A message declaring a different `schema_version` than
  expected (1) is refused (the node logs, the watchdog holds STOP).

## Arming interlock — real robot unreachable by configuration alone

`arming_interlock.py` refuses to arm unless ALL hold (fails closed on any
ambiguity): `arm:=true`; `ROS_DOMAIN_ID` not 0 (live cell) and not 42
(shelfbot_ros container); and `robot_ip` is loopback OR a Docker-internal subnet
AND not 192.168.0.1 AND not on any physical NIC (physical is checked first, so an
address on both a physical NIC and a docker bridge is still refused). The node
enumerates physical vs docker subnets from `ip -o -4 addr` (docker-like interface
names -> docker subnets) and hands them to the pure decision. Unit-tested in
`tests/test_arming_interlock.py`; verified live to refuse a disarmed start and
`robot_ip:=192.168.0.1`, and to arm on loopback / a docker-internal URSim IP.
Real-robot arming is a later change — no launch argument here reaches real
hardware.

## Sole-writer discipline and fail-to-zero

- **Sole writer (`sole_writer.py`, report §5 #16).** The gate watches the applied
  scaling; a scaling change to a value it did not command (and that is not the
  inert pre-command value) is a foreign write -> conflict -> self-fault ->
  BACKSTOP, surfaced loudly. It carefully distinguishes a foreign write (scaling
  MOVED to an uncommanded value) from an inert/ineffective slider (scaling did
  NOT move — the mock case), which is the backstop's job, not a conflict.
- **Fail toward zero.** On a lost camera-state heartbeat the gate core already
  emits STOP (slider 0); on a self-fault it emits BACKSTOP; on shutdown the node
  writes the slider to 0. The node NEVER raises the slider on its own — only an
  explicit healthy CLEAR held for the full dwell reaches a RUN decision.

## Independent backstop watcher

`backstop_watcher_node.py` is a SEPARATE PROCESS with its own timer and its own
camera-state subscription, so a hang in the gate cannot disable it. It cancels the
trajectory on: its own camera-state heartbeat loss, the gate's decision-heartbeat
loss, a gate BACKSTOP, or a hazard STOP the slider did not effect within a
deadline. The hazard is the camera zone STOP or the gate's *demand* STOP — NOT the
gate holding STOP while dwelling toward a resume (that would discard the very
trajectory the slider is freezing). "Never received state at startup" cancels but
does NOT latch (it recovers once state flows); a heartbeat lost after being
received latches. Pure core unit-tested in `tests/test_watcher_core.py`.

## Command-to-effect delay and whether slider zero fully halts (MEASURED in sim)

See the end-to-end run logs under `mock/` and `ursim/` and the summary printed by
`sim/assert_e2e.py`. Findings recorded from the sim runs:

- **Mock hardware:** speed scaling is inert (reported on a 0..100 scale, stuck at
  100 = full). The slider cannot halt the arm in mock, so a STOP is effected by
  the BACKSTOP (cancel) — demonstrated by both the gate's own primary->backstop
  fall-through AND the independent watcher. Command-to-effect for the slider is
  therefore N/A in mock (that is the documented mock limitation, report §8).
- **URSim CB3 3.15.8:** the slider IS effective (slider 0.3 -> scaling 30, slider
  1.0 -> scaling 100). The command-to-effect delay for the STOP and the observed
  freeze/resume behaviour are recorded in `ursim/` (see the assert_e2e summary and
  `SLIDER_ZERO_FINDING.md`). Whether slider zero FULLY halts or a floor applies is
  reported there — this is exactly report residual risk R3 and the reason the
  goal-cancel backstop is not optional.

## Can "pause the robot program" be the STOP (freeze-and-continue)? — investigated, NO

The captain asked whether the gate's STOP could **freeze the planned motion in
place and later continue that same motion** rather than stop-then-replan, the
natural candidate being to **pause/resume the robot program** via the driver's
dashboard. This was investigated on URSim CB3 3.15.8 with the driver launched
**headless** (as this cell runs it — `ur-bringup.sh`: `headless_mode:=true`),
repeatable via `humangate_driver/sim/investigate_pause_ursim.sh` (domain 81), and
the answer is **no true freeze-and-continue exists on this controller**:

- **Slider zero (source + measurement).** `set_speed_slider`
  (`ur_msgs/SetSpeedSliderFraction`) accepts `[0.0, 1.0]` — 0.0 is a valid value,
  the service does NOT reject below a minimum (installed .srv doc; `rtde_writer.h
  sendSpeedSlider`). The ~10 % floor is applied DOWNSTREAM by PolyScope, so the
  smallest accepted value (0.0) still leaves the arm creeping at ~10 % scaling.
- **Dashboard pause/play (headless).** Both are **rejected by the robot**
  ("Failed to execute: pause" / "Failed to execute: play"): in headless mode there
  is no dashboard-loaded program (`loaded_program=""`, `program_state=STOPPED`); the
  motion runs from the injected external-control script (`program_running=true`),
  which the dashboard verbs cannot act on. The arm never froze (pan advanced 0.47 rad
  through the "pause"), the FollowJointTrajectory goal was unaffected and completed
  successfully, and — unlike dashboard `stop` — pause did NOT drop the driver
  connection (it is a no-op-that-errors). A gate trusting the pause reply would fail
  *open*, which is exactly why it is not adopted.
- **Other mechanisms.** Dashboard `stop` and goal-cancel both fully stop but are
  terminal (need a fresh plan); MoveIt stop == goal-cancel; cuMotion is plan-time
  only. Nothing gives full-stop AND same-goal continue.

**Decision (per the task's decision rule): the stop path is UNCHANGED.** STOP stays
the goal-cancel backstop (a resume after STOP is a fresh plan-gate-execute); SLOW
stays the speed slider (graded, and a raise above the floor continues the same
trajectory — the only freeze-and-continue behaviour that works, but never a full
stop). Full measurements, root cause, per-candidate failure analysis, and the
regression guard are in `humangate_driver/PAUSE_RESUME_FINDING.md`
(analyzer + guard: `sim/assert_pause_investigation.py`, offline-tested in
`tests/test_pause_investigation.py`).

## Pendant-pause STOP strategy (pendant-pause task, ROS_DOMAIN_ID=82)

The captain asked to "load the program on the pendant and then try pausing it" — the
follow-up to the headless `PAUSE_RESUME_FINDING.md` (where pause is rejected because
headless has no loaded program). Measured on URSim CB3 with an External Control program
LOADED and PLAYING on the (simulated) pendant, driver NON-headless: **dashboard `pause`
gives a true freeze-and-continue** — the arm freezes on its path in ~1.8 s, applied
scaling → 0 (a *confirmable* stop, unlike slider-zero's ~10 % floor), the running
`FollowJointTrajectory` goal SURVIVES, and `play` continues the SAME goal to `code 0`
with no replan. Durability held at 5 s / 60 s / 10 min holds; the failure cases each
still end stopped. Full measurements, side-by-side with headless, and the operational
implications of running the cell pendant-loaded: `humangate_driver/PENDANT_PAUSE_FINDING.md`.

Decision: **add pendant-pause as a SELECTABLE STOP strategy** behind the command port
(`humangate_driver/stop_strategy.py`, chosen by `stop_strategy` in `params.yaml`):

- `cancel` (DEFAULT, unchanged): STOP = slider→0 + goal-cancel backstop (stop-then-replan).
  This is how the cell runs today (headless); nothing in the shipped stop path changes.
- `pendant_pause` (opt-in): STOP = dashboard pause (freeze on path); a healthy CLEAR+dwell
  RESUME = dashboard play (continue the same trajectory). The slider stays the SLOW lever
  and the independent goal-cancel backstop stays the net in BOTH. The strategy resumes
  only on the gate's own RUN, refuses resume beyond `resume_max_pause_s` (→ cancel + replan),
  and re-pauses on a foreign play. Pure + unit-tested (`tests/test_stop_strategy.py`);
  proven end to end through the real gate node (`sim/run_pendant_gate_e2e.sh`).

Running the cell pendant-loaded instead of headless is an OWNER decision, NOT changed
here (`ur-bringup.sh` untouched, default `cancel`): someone must start the program, the
`ur-rearm-watchdog.sh` re-arm conflicts with a pause-STOP (it plays `program_running`
back up) and must be made pause-aware or disabled in that mode, and the pendant's
External Control host IP must point at the workstation. Recommendation and options are
in `PENDANT_PAUSE_FINDING.md`.

## What is NOT done here (later phases)

- No real-robot motion, no real-robot arming (Phase 4, owner present — see
  `LIVE_TEST.md`).
- The provisional zone/dwell/latency numbers are unchanged from Phase 1 and are
  re-derived from measurement only in the owner-present phase.
- The certified presence device (Phase 5) is out of scope.
