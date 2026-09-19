# humangate — robot-side human-safety gate (Phase 1, decision core)

A ROS 2 Jazzy node that consumes the camera-side human-safety **state** and
decides what the robot should do — **RUN / SLOW / STOP / BACKSTOP** — and, in
this phase, **only records what it would command**. It never touches the robot.

This is **Phase 1** of the phased plan in the design report
`human-safety-s1/report.md` (§10): *interface freeze + gate skeleton, no robot.*
Wiring these decisions to the real driver (speed slider + trajectory cancel),
first against a simulated arm, is the next, separate change (Phase 3).

## What this is NOT — the one thing to read first

**This is a supplementary awareness layer, not a certified safety function.** A
camera + neural-network person detector is a single-channel, unrated sensing
chain with no diagnostic coverage; it cannot be ISO 13849 PL d / Cat 3 and must
never be described or relied on as a certified or sufficient safety function
(report §0, §4). The **actual** protection against injury is, and must remain:

1. the robot controller's **safety-rated protective stop** — on this CB3 the
   teach-pendant **Force limit** (pendant-only; see `data/collision-force-v1/`);
   and
2. (owner decision) a **safety-rated presence device** — an IEC 61496 light
   curtain or safety laser scanner, PL d / Cat 3, hard-wired into the
   controller's **safeguard-stop input** — which this camera layer sits *beside*,
   never replaces.

This gate makes the arm politely slow/stop around people early and resume when
clear, so the certified hard stops rarely have to fire. That is all it claims.

## How the feature fits together

```
 cameras ──► human-detect-core-v1 (parallel task)         THIS TASK (Phase 1)
            detect + track + fuse   ──►  /human_safety/state  ──►  human_gate
            publishes CLEAR/SLOW/STOP,        (state message)       decide RUN/SLOW/
            nearest distance, health,                               STOP/BACKSTOP,
            heartbeat                                               RECORD only
```

The gate is the **only** consumer that acts on the state; the perception side is
forbidden from touching the robot, and so is this phase.

## The decision model (report §2, §5, §7)

- **RUN** — run the planned trajectory at the operating speed fraction.
- **SLOW** — scale the running trajectory down to a low fraction.
- **STOP** — speed fraction zero: freeze on the planned path at the current spot.
- **BACKSTOP** — cancel the trajectory goal so the arm decelerates to a hold
  (the independent fall-through; a resume afterwards needs a fresh plan).

Everything fails toward stop. Hard rules implemented in `gate_core.py`:

| Rule (report) | Behaviour |
|---|---|
| No state topic at startup (§7.4) | STOP |
| State older than `state_max_age_s` by its own **evidence** stamp (§7.2) | STOP |
| No message within `heartbeat_timeout_s`, or `heartbeat_seq` stops advancing (§7.3) | STOP |
| Clock skew: stamp in the future beyond tolerance (§5 #6) | STOP |
| Any health fault / flag forcing non-CLEAR (§5, §7.4) | never CLEAR → at least SLOW |
| Unknown distance is never "far" (§7.5) | distance is not on the path to RUN at all |
| Leaving STOP (§3, §7.6) | only after the gate itself observes `clear_dwell_s` of continuous healthy CLEAR (re-checked even though the camera side also dwells) |
| Person **lost close** vs **tracked leaving** (§5 #9) | a CLEAR reached by a person vanishing while close does **not** enable resume; only a tracked departure (…→SLOW→CLEAR) does |
| Primary→backstop fall-through (§2, §5 #15) | after a hazard STOP, if driver feedback does not confirm the arm stopped within `backstop_deadline_s`, or on any gate self-fault, escalate to BACKSTOP |

## Plan-only guarantee

The command interface is behind an abstract `CommandPort` (`command_port.py`)
with exactly one implementation here: `RecorderPort`, which records what it would
command. There is **no** `set_speed_slider` client, **no** FollowJointTrajectory
action client, **no** dashboard client, and **no** publisher on any controller
topic anywhere in this package. `test_no_driver_interface.py` asserts this
statically (no driver-package imports, no driver-interface identifiers) and at
runtime (importing the package pulls no driver package).

## Modules

| File | Role |
|---|---|
| `camera_state.py` | the gate's stable representation of the camera-side state (report §7.1). Pure. |
| `state_adapter.py` | thin adapter wire-message → `CameraState`; **swap point** for the real perception message (see below). Pure. |
| `gate_config.py` / `gate_params.yaml` | thresholds, all provisional report defaults in one place. Pure. |
| `gate_core.py` | the pure, deterministic decision core (all rules above). No ROS. |
| `command_port.py` | abstract command port + `RecorderPort` (records, never sends). Pure. |
| `driver_feedback.py` | injectable "is the arm stopped?" feedback + a simulated impl. Pure. |
| `replay.py` | deterministic replay + fault-injection harness (all report §8 scenarios). Pure. |
| `gate_node.py` | the ROS 2 node: subscribe, decide, publish decision + reason, log. |
| `human_gate.launch.py` | launch the node on the isolated domain. |
| `test_*.py` | offline unit/property tests + the one ROS integration test. |

## Interface contract status (report §7)

The perception package on branch `fm/human-detect-core-v1` is **not yet committed
to this worktree** (its branch is at the same commit as `main`), so there is no
custom message to build against. Per the task brief we built the gate against
report §7 through the adapter and recorded exactly what must be confirmed when
the real message lands — see `state_adapter.CONTRACT_TO_CONFIRM` and
`data/human-gate-core-v1/decision.md`.

**Provisional transport:** until the custom message exists, the gate consumes the
state as a `std_msgs/String` carrying a JSON object with the §7 field names. This
keeps the whole gate (including the ROS integration test) runnable now without
depending on an uncommitted package. When the message lands, diff it against §7,
fill in `state_adapter.parse_custom_msg`, and point the node's subscription at the
real type — a one-line change each (the JSON keys already equal the intended
field names).

## Parameters (all PROVISIONAL — tune up, never down; report §3/§7)

See `gate_params.yaml`. Key defaults: `state_max_age_s=0.2`,
`heartbeat_timeout_s=0.3`, `clear_dwell_s=2.0`, `stop_radius_m=2.0`,
`slow_speed_fraction=0.1`, `stop_speed_fraction=0.0`, `backstop_deadline_s=0.5`.
These are conservative placeholders to be **re-derived from measured** end-to-end
latency (Tr) and stopping time (Ts) in the owner-present phase (report §8 / §10
Phase 4).

## Running the tests (offline, deterministic — no robot, no cameras)

```bash
cd humangate
python3 test_gate_core.py            # every fail-toward-stop rule + fall-through + property test
python3 test_gate_config.py
python3 test_state_adapter.py
python3 test_replay.py               # scripted scenarios (approach/hover/leave/faults)
python3 test_no_driver_interface.py  # the plan-only assertion
python3 test_gate_node_integration.py  # the ONE ROS test, isolated on ROS_DOMAIN_ID=78
# or, all at once:
pytest -q
```

The integration test runs its ROS graph in a subprocess with
`ROS_DOMAIN_ID=78 FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; it **skips** cleanly if
rclpy/DDS is unavailable.

## Running the node (plan-only)

```bash
ROS_DOMAIN_ID=78 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  python3 gate_node.py --ros-args -p decision_log:=/tmp/human_gate/decisions.jsonl
# or via launch (applies the isolation env for you):
ros2 launch humangate/human_gate.launch.py decision_log:=/tmp/human_gate/decisions.jsonl
```

It subscribes to `/human_safety/state`, publishes its decision + reason on
`/human_gate/decision`, and (with `decision_log`/`command_log` set) writes a
structured JSONL log of every decision and every command it *would* have issued.

## Decision log

Dated design decisions, the interface confirm-list, and the provisional defaults
rationale: `data/human-gate-core-v1/decision.md`.
