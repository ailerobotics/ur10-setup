# humangate_driver — driver-backed human-safety gate (Phase 3, SIMULATED arm only)

This package connects the plan-only decision core in `../humangate` to the real
Universal Robots driver levers and proves the behaviour end to end **against a
simulated arm only** — the driver's mock hardware and the CB3 Universal Robots
simulator (URSim). It is Phase 3 of the design in
`~/firstmate/data/human-safety-s1/report.md` (§10). Running on the real arm is a
later, separate, owner-present step (`LIVE_TEST.md`).

## Read this first — what it is NOT

**A supplementary awareness layer, NOT a certified safety function.** A camera +
CNN detector is single-channel, unrated, no diagnostic coverage; it cannot be
ISO 13849 PL d / Cat 3 and must never be relied on as the protective stop. The
real protection remains the CB3 teach-pendant **Force limit** and (owner decision)
a safety-rated presence device wired to the safeguard-stop input (report §0/§4).
This layer only makes the arm slow/stop early and resume when clear.

## Why a separate package

`../humangate` is provably plan-only: `humangate/test_no_driver_interface.py`
asserts it imports NO driver interface. All driver-touching code lives HERE so
that guarantee stays intact. The gate core, config, and command-port abstraction
are reused unchanged; the driver port implements the same abstract `CommandPort`
and `DriverFeedback` bases from `../humangate`.

## The two levers (report §2)

- **Primary — speed slider** via `set_speed_slider`
  (`ur_msgs/srv/SetSpeedSliderFraction`). SLOW lowers it; STOP commands zero; RESUME
  raises it, resuming the same trajectory — when the driver honours a zero slider
  (see the CB3 finding in `SLIDER_ZERO_FINDING.md`). Effect confirmed on
  `/speed_scaling_state_broadcaster/speed_scaling`.
- **Backstop — cancel the `FollowJointTrajectory` goal** via the action's generic
  cancel service (`action_msgs/srv/CancelGoal`, cancel-all), so the gate/watcher
  can cancel a trajectory they did NOT start. Terminal — a resume needs a fresh
  plan. Driven by a SEPARATE watcher process.

## Selectable STOP strategy (pendant-pause task)

How a full STOP is effected and whether a RESUME continues the SAME trajectory is a
selectable strategy (`stop_strategy` in `params.yaml`); the slider stays the SLOW
lever and the goal-cancel backstop stays the independent net in BOTH:

- **`cancel` (DEFAULT — how this cell runs today, HEADLESS):** STOP drives the slider
  toward zero and the goal-cancel backstop delivers the real full stop
  (stop-then-replan). `SLIDER_ZERO_FINDING.md`, `PAUSE_RESUME_FINDING.md`.
- **`pendant_pause` (opt-in — only for a cell running a pendant-LOADED External Control
  program, non-headless):** STOP issues a dashboard **pause** (freezes the arm on its
  planned path, scaling→0, the running goal survives) and a healthy CLEAR+dwell RESUME
  issues **play** (continues the SAME trajectory, no replan). Measured on URSim CB3 in
  `PENDANT_PAUSE_FINDING.md`. It resumes only on the gate's own RUN, refuses to resume
  once the pause is older than `resume_max_pause_s` (→ cancel + replan), and re-pauses
  on a foreign play. Do NOT select it on the cell's headless bring-up (pause is
  rejected there).

A pendant pause freezes over ~1.8 s (far slower than the slider), so BOTH the gate's
STOP→BACKSTOP deadline and the independent watcher's stop-effect deadline must outlast
it. A SINGLE knob, `pendant_freeze_confirm_s`, sizes both: `params.yaml` expresses it
(and `stop_strategy`) once via a YAML anchor shared by both process sections, and the
watcher — given the same `stop_strategy` — auto-sizes its effective deadline to
`max(stop_effect_deadline_s, pendant_freeze_confirm_s)`. The watcher stays a fully
independent process (its own timer and its own camera-state heartbeat); as a runtime
guard it also cross-checks the strategy the gate ANNOUNCES on `/human_gate/decision`
against its own configured one, and on any mismatch reports it loudly and repeatedly
(log + `/human_gate/watcher_status`) while STILL cancelling at its own deadline — the arm
always ends stopped.

## Modules

| File | Role |
|---|---|
| `arming_interlock.py` | PURE: refuses to arm unless `arm:=true`, domain not 0/42, and robot_ip loopback/Docker-internal (never 192.168.0.1 / physical). |
| `sole_writer.py` | PURE: detects a foreign speed-slider write (scaling moved to an uncommanded value) vs an inert slider (mock). |
| `watcher_core.py` | PURE: the independent backstop decision (heartbeat losses, BACKSTOP, ineffective STOP). |
| `driver_io.py` | ROS clients: slider service, scaling monitor (normalises the mock 0..100 scale), joint-motion rest detector, FJT cancel, host-subnet enumeration. |
| `driver_port.py` | `DriverCommandPort` (primary + backstop + fail-to-zero + per-tick `apply` dispatch to the STOP strategy) and `DriverFeedbackFromTopics` (STOP confirmed only when scaling ~0). |
| `stop_strategy.py` | PURE: the selectable STOP strategy — `CancelHoldStrategy` (default) and `PendantPauseStrategy` (dashboard pause/play, resume window, re-pause on foreign play). |
| `gate_driver_node.py` | armed gate node: interlock -> HumanGate core -> DriverCommandPort (+ DashboardClient when `stop_strategy=pendant_pause`). |
| `backstop_watcher_node.py` | the INDEPENDENT watcher process (own timer, own state heartbeat); publishes `/human_gate/watcher_status` and cross-checks the gate's announced `stop_strategy`. |
| `params.yaml` | driver/interlock params (decision thresholds still live in `../humangate/gate_params.yaml`). |
| `sim/` | the end-to-end proofs + scripted states + analyzer (below). |
| `tests/` | offline unit tests for the interlock, sole-writer and watcher core. |
| `LIVE_TEST.md` | the owner-present real-arm commissioning checklist (written, not run). |

## Arming interlock — real robot unreachable by configuration alone

`gate_driver_node.py` and `backstop_watcher_node.py` both REFUSE to start unless
ALL hold (fails closed on any ambiguity): `arm:=true`; `ROS_DOMAIN_ID` not 0 (live
cell) and not 42 (shelfbot_ros container); and `robot_ip` loopback OR a
Docker-internal subnet, AND not 192.168.0.1, AND not on any physical NIC. Real-arm
arming is a later, reviewed change (see `LIVE_TEST.md`).

## Offline tests (no ROS, no robot, no docker)

```bash
cd humangate_driver
python3 tests/test_arming_interlock.py
python3 tests/test_sole_writer.py
python3 tests/test_watcher_core.py
python3 tests/test_stop_strategy.py          # cancel vs pendant_pause strategy, resume window, re-pause
python3 tests/test_pause_investigation.py    # analyzer + regression guard for the pause findings (headless + pendant)
# or: pytest -q tests
```

## End-to-end simulation proofs (repeatable; logs in data/human-gate-sim-v1/)

Both run everything on `ROS_DOMAIN_ID=79 FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, start
their OWN driver, and stop everything they start by PID (the driver by process
group, so no child nodes leak). They never touch `ur10-driver`, the robot
containers, or any process/container they did not create.

```bash
# (a) mock hardware — no docker. Proves the decision logic, the gate's own
#     primary->backstop fall-through (slider is inert in mock), the independent
#     watcher, sole-writer, interlock, and every fault path.
bash sim/run_mock_e2e.sh

# (b) URSim CB3 in docker — the rig that can exercise the speed slider. Runs the
#     simulator on its OWN docker network (192.168.98.0/24), reachable only via its
#     docker-internal IP, no GPU; removes its container + network on exit.
bash sim/run_ursim_e2e.sh

# slider characterisation (what SLOW / STOP do to a moving arm on CB3):
bash sim/measure_slider_ursim.sh   # -> data/human-gate-sim-v1/ursim/slider_measurement.txt

# (c) can "pause the robot program" freeze-and-continue the SAME motion? Launches
#     URSim + the driver HEADLESS (as this cell runs it) and probes dashboard
#     pause/play against a running scaled-JTC trajectory. Self-asserts the finding.
#     Runs on ROS_DOMAIN_ID=81 (the pause-investigation task's isolated domain).
bash sim/investigate_pause_ursim.sh   # -> data/human-gate-sim-v1/ursim-pause/

# (d) the pendant-pause task (ROS_DOMAIN_ID=82): the SAME probe against a
#     pendant-LOADED External Control program (non-headless). Here pause DOES
#     freeze-and-continue (PENDANT_PAUSE_FINDING.md).
MODE=pendant bash sim/investigate_pause_ursim.sh    # -> data/human-gate-sim-v1/ursim-pendant-pause/
bash sim/run_pendant_pause_matrix.sh                # durability 5s/60s/10min + every failure case
bash sim/run_pendant_gate_e2e.sh                    # the real gate driving pause/play -> ursim-pendant-gate/
```

`sim/assert_e2e.py` analyses the run logs and prints PASS/FAIL plus the
command-to-effect delay. Scripted camera states are in `sim/scenarios.py` and are
published in the REAL camera wire format (`../humangate/state_adapter.build_state_json`).

## Decisions + findings

`../data/human-gate-sim-v1/decision.md` (design decisions, the adapter/real-message
mapping, the interlock rationale), `SLIDER_ZERO_FINDING.md` (whether slider zero
fully halts on CB3, and why the goal-cancel backstop is not optional),
`PAUSE_RESUME_FINDING.md` (whether "pause the robot program" can freeze-and-continue
the SAME motion — measured NO on headless CB3, so the STOP stays the goal-cancel
backstop / stop-then-replan), and `PENDANT_PAUSE_FINDING.md` (the same question with a
pendant-LOADED External Control program — measured YES, so the opt-in `pendant_pause`
STOP strategy; headless-cancel stays the default).
