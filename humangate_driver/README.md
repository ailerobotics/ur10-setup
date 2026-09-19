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

## Modules

| File | Role |
|---|---|
| `arming_interlock.py` | PURE: refuses to arm unless `arm:=true`, domain not 0/42, and robot_ip loopback/Docker-internal (never 192.168.0.1 / physical). |
| `sole_writer.py` | PURE: detects a foreign speed-slider write (scaling moved to an uncommanded value) vs an inert slider (mock). |
| `watcher_core.py` | PURE: the independent backstop decision (heartbeat losses, BACKSTOP, ineffective STOP). |
| `driver_io.py` | ROS clients: slider service, scaling monitor (normalises the mock 0..100 scale), joint-motion rest detector, FJT cancel, host-subnet enumeration. |
| `driver_port.py` | `DriverCommandPort` (primary + backstop + fail-to-zero) and `DriverFeedbackFromTopics` (STOP confirmed only when scaling ~0). |
| `gate_driver_node.py` | armed gate node: interlock -> HumanGate core -> DriverCommandPort. |
| `backstop_watcher_node.py` | the INDEPENDENT watcher process (own timer, own state heartbeat). |
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
```

`sim/assert_e2e.py` analyses the run logs and prints PASS/FAIL plus the
command-to-effect delay. Scripted camera states are in `sim/scenarios.py` and are
published in the REAL camera wire format (`../humangate/state_adapter.build_state_json`).

## Decisions + findings

`../data/human-gate-sim-v1/decision.md` (design decisions, the adapter/real-message
mapping, the interlock rationale) and `SLIDER_ZERO_FINDING.md` (whether slider zero
fully halts on CB3, and why the goal-cancel backstop is not optional).
