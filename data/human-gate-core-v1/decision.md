# Robot-side human-safety gate — decision core (human-gate-core-v1)

**Date:** 2026-09-19
**Branch:** `fm/human-gate-core-v1`
**Design source of truth:** `human-safety-s1/report.md` (this task is its **Phase 1**, §10).
**Code:** `humangate/` (see `humangate/README.md`).

**Goal:** the decision-making core of the robot-side gate — it receives the
camera-side stop/slow/clear state and decides what the robot should do
(RUN / SLOW / STOP / BACKSTOP), and **only records what it would command.** No
robot contact of any kind; no driver, controller, dashboard or speed-slider
interface is imported or called. Wiring to the real driver (first against a
simulated arm) is a later, separate change (report §10 Phase 3).

## Non-claim (must travel with the code)

This is a **supplementary awareness layer, not a certified safety function**
(report §0, §4). A camera + CNN detector is single-channel, unrated, no
diagnostic coverage, non-deterministic — it is **not** IEC 61496 / ISO 13849
PL d / Cat 3. The certified protection is the controller's **safety-rated
protective stop** (on this CB3, the pendant Force limit — pendant-only, see
`data/collision-force-v1/decision.md`) and, if installed, a **safety-rated
presence device** on the safeguard-stop input. This gate sits *beside* those,
never replaces them.

## What was built

A pure, deterministic decision core (`gate_core.HumanGate`) plus a plan-only ROS
node (`gate_node.GateNode`), a thin contract adapter, an injectable driver-
feedback port with a simulated impl, a recorder command port, a replay/fault-
injection harness, and offline + one ROS integration test. Full module map in
`humangate/README.md`.

## Key decisions

1. **Primary lever = speed scaling; independent backstop = goal-cancel + hold**
   (report §2). Modelled as two abstract intents on `CommandPort`
   (`set_speed_fraction`, `cancel_and_hold`), both routed to a `RecorderPort`
   only. RUN→operating fraction, SLOW→low fraction, STOP→0, BACKSTOP→cancel.

2. **Fail-toward-stop, every rule from report §5/§7** implemented in the core and
   individually unit-tested: startup-without-topic → STOP; evidence-stamp older
   than `state_max_age_s` → STOP; heartbeat silence or `heartbeat_seq` stall
   → STOP; clock skew (future stamp) → STOP; any health fault/flag → never CLEAR;
   unknown distance never treated as far (distance is not on the path to RUN at
   all); leave STOP only after the gate observes `clear_dwell_s` of continuous
   healthy CLEAR itself (defence in depth over the camera-side dwell).

3. **Resume refusal on "person lost close" (report §5 #9).** The core classifies
   how each CLEAR was reached: a CLEAR entered directly from STOP (or with an
   explicit `TRACK_LOST_CLOSE` health flag) is `LOST_CLOSE` and **never** enables
   a resume, through the whole dwell and beyond; only a *tracked departure*
   (…→SLOW→CLEAR, `TRACKED_LEAVING`) or a clean start does. `LOST_CLOSE` latches
   until a subsequent genuine tracked pass clears it. **Provisional / conservative
   decision:** absent an operator-reset input (added in Phase 3), a person who
   vanishes close leaves the arm stopped until a person is next tracked cleanly
   in and out. This matches "never resume merely because a person became
   invisible" and is safe; the operator-reset/confirmation path is Phase 3.

4. **Primary→backstop fall-through (report §2, §5 #15).** Only a **hazard** STOP
   (a person/fault demands the arm stop) arms the deadline: if driver feedback
   does not confirm the arm stopped within `backstop_deadline_s`, or on any gate
   self-fault, escalate to BACKSTOP. A STOP-hold while *dwelling toward a resume*
   never escalates — cancelling the goal there would discard the very trajectory
   the primary lever exists to freeze and resume. Driver feedback is an injectable
   interface (`DriverFeedback`) with a simulated impl; the node uses an
   always-stopped stand-in (the real feedback is Phase 3).

5. **Plan-only, provably.** No driver interface is imported or referenced.
   `test_no_driver_interface.py` asserts this statically (AST scan: no
   `control_msgs` / `ur_msgs` / `ur_dashboard_msgs` imports, no dashboard client,
   no `FollowJointTrajectory` / `SetSpeedSliderFraction` identifiers — docstrings
   that *name* these to disclaim them are ignored) and at runtime (importing every
   module pulls no driver package into `sys.modules`).

6. **Thresholds are parameters with conservative provisional defaults**
   (`gate_params.yaml`, report §3/§7), to be re-derived from measured Tr/Ts in the
   owner-present phase (report §8 / §10 Phase 4). Tune up, never down.

## Interface contract — status and what must be confirmed (report §7)

The perception package on branch `fm/human-detect-core-v1` is **NOT yet committed
to this worktree** (its branch is at the same commit as `main` — verified
2026-09-19 with `git diff --name-only main fm/human-detect-core-v1`). So the gate
was built against report §7 through a thin adapter (`state_adapter.py`), and the
following must be **confirmed / diffed against the committed message before
Phase 3** (mirrored in `state_adapter.CONTRACT_TO_CONFIRM`):

- exact custom **message name + package** (adapter currently uses a
  `std_msgs/String` JSON transport as a stand-in);
- exact **topic name** (assumed `/human_safety/state`);
- whether the evidence **stamp** is a header `builtin_interfaces/Time` or a float,
  and that it is **evidence** time not publication time (§7.1/§7.2);
- whether **health** is an enum, bool, or bitfield, and the flag naming (the gate
  needs a distinguishable `TRACK_LOST_CLOSE` signal, §5 #9);
- whether a separate machine-readable **heartbeat** is published or `heartbeat_seq`
  is a message field (§7.3);
- the **unknown-distance** encoding (NaN vs field-absent vs null) (§7.5);
- the guaranteed **minimum publish rate** (§7.2, recommend ≥10 Hz).

When the message lands: diff it against §7, implement `parse_custom_msg`, and
point the node's subscription at the real type — one line each. The JSON keys
already equal the intended field names, so alignment is trivial.

## Verification (offline, deterministic; report §8)

- `test_gate_core.py` — every fail-toward-stop rule, the primary→backstop
  fall-through, and a property-style test (200 randomised sequences) that **no
  input sequence yields RUN without a preceding healthy CLEAR dwell**.
- `test_gate_config.py`, `test_state_adapter.py` — config + adapter.
- `test_replay.py` — scripted scenarios: approach, hover-at-boundary, tracked
  leaving→resume, heartbeat drop, heartbeat stall, stale stamps, distance unknown,
  vanish-close (+ flagged), clock skew, STOP→BACKSTOP.
- `test_no_driver_interface.py` — the plan-only assertion.
- `test_gate_node_integration.py` — the one ROS test, on an isolated
  `ROS_DOMAIN_ID=78` with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (skips if rclpy/DDS
  is unavailable).

All robot contact is avoided: no driver/container/ROS process is started, stopped
or restarted; the only ROS graph brought up is the gate + a fake publisher on
domain 78, both torn down by PID at the end of the test.
