# Can the gate STOP by "pause the robot program" and later continue the SAME planned motion? — measured on URSim CB3, HEADLESS

The captain asked for a STOP that **freezes the planned motion in place and later
continues that same motion** (not stop-then-replan), the natural candidate being
to **pause and resume the robot program** via the driver's dashboard interface.
This file records whether that — or any freeze-and-continue mechanism — exists on
this cell's controller, with the measurements, and the resulting decision.

> **Follow-up (pendant-pause task):** the natural next question — "load the program on
> the pendant and pause THAT" — was subsequently measured and the answer flips: with an
> External Control program LOADED and PLAYING on the pendant (non-headless), dashboard
> `pause` DOES give a true freeze-and-continue. See `PENDANT_PAUSE_FINDING.md` and the
> opt-in `stop_strategy: pendant_pause`. This headless finding is unchanged and remains
> the DEFAULT (`cancel`), because this cell runs headless today.

**Answer (HEADLESS, this cell today): no true freeze-and-continue exists.** On the
CB3 driver as this cell runs it (HEADLESS — `ur-bringup.sh`: `headless_mode:=true`),
dashboard `pause`/`play` are *rejected by the robot*, the speed slider floors above
zero, and a goal cancel is terminal. So the gate's STOP stays the goal-cancel
backstop (stop-then-replan), the slider stays the graded SLOW + same-trajectory
resume ABOVE the floor, and this file explains precisely why. This is the same
class of result as, and a companion to, `SLIDER_ZERO_FINDING.md`.

Repeatable: `sim/investigate_pause_ursim.sh` (URSim CB3 `3.15.8`, matching the
cell's PolyScope 3.15.8, real `ur_robot_driver` 3.8.0, headless, on its own docker
network, `ROS_DOMAIN_ID=81`). Raw result:
`../data/human-gate-sim-v1/ursim-pause/pause_investigation.txt`. Analyzed by
`sim/assert_pause_investigation.py` (also an offline regression guard — see below).

---

## Q1 — Why slider zero does not fully halt (source + measurement)

**The `set_speed_slider` service does NOT reject values below a minimum; 0.0 is a
valid, accepted value.** From the installed interface and client library:

- `ur_msgs/srv/SetSpeedSliderFraction` (installed .srv doc): *"Values for
  'speed_slider_fraction' must be from [0; 1.0]; values outside this valid range
  will result in an error being returned to the caller."* — so the **accepted
  range is [0.0, 1.0]** and the **smallest accepted value is 0.0**.
- `ur_client_library/rtde/rtde_writer.h::sendSpeedSlider(double)` doc: *"the new
  speed slider fraction as a value between 0.0 and 1.0"*.

So the floor is **not** a rejection in the ROS/RTDE layer — it is applied
**downstream** by PolyScope / the robot's own speed scaling. Measured at the
smallest accepted value (`../data/human-gate-sim-v1/ursim/slider_measurement.txt`,
`sim/measure_slider_ursim.sh`), sweeping the slider on a moving arm:

| commanded slider | applied scaling | shoulder_pan motion (rad / 2.5 s) |
|---|---|---|
| 1.0            | 100 %            | 0.013 (moving)                   |
| 0.5            | 50 %             | 0.017                            |
| 0.1            | 10 %             | 0.004 (creeping)                 |
| **0.0**        | **held at 10 %** | **0.0045 — STILL CREEPING**      |
| 1.0 (resume)   | 100 %            | 0.064 (SAME trajectory continues)|

At the smallest accepted value (0.0) the **controller holds the applied scaling at
a ~10 % floor and the arm keeps creeping** — it does not fully halt. This is report
residual risk **R3**. The slider is the good lever for graded SLOW and for
same-trajectory RESUME above the floor; it is not a full STOP.

## Q2 — Pause / resume of the robot program (the captain's candidate), HEADLESS

Method: URSim CB3 + the real driver launched **headless** (matching the cell), one
long trajectory on the `scaled_joint_trajectory_controller`, then call
`/dashboard_client/pause`, hold, `/dashboard_client/play`, measuring the dashboard
replies, joint motion, the FollowJointTrajectory goal, the driver connection, and
the dashboard's loaded-program state. Measured (clean run):

| observation | value |
|---|---|
| dashboard `pause` reply | **`success=false`, "Expected: Pausing program, but received: Failed to execute: pause"** |
| dashboard `play` reply | **`success=false`, "Expected: Starting program, but received: Failed to execute: play"** |
| arm clearly moving before pause? | yes (0.059 rad/s pan) |
| arm froze after pause? | **NO** — pan advanced 0.47 rad *during* the 8 s "pause", 0.94 rad total |
| FJT goal aborted by pause? | **NO** — goal stayed active and later **succeeded (error_code 0)** |
| `/joint_states` flowing during pause? | **YES** — the driver connection did NOT drop |
| dashboard `program_state` (before / during motion / during pause) | **`STOPPED`** throughout |
| dashboard `loaded_program` | **`""` (empty)** throughout |
| dashboard `program_running` | `true` throughout |

**Root cause.** In headless mode the driver injects the external-control URScript
over the primary interface; there is **no dashboard-loaded PolyScope program**
(`loaded_program=""`, `program_state=STOPPED`). `program_running=true` reflects only
that injected script. The dashboard `pause`/`play` act on a *loaded/playing
PolyScope program* — there is none — so the robot answers **"Failed to execute"**
and nothing happens: the arm neither stops nor is affected, and the trajectory runs
to completion.

Consequences, point by point on what the captain asked to measure:
- **Does the arm come to a full stop on its path?** No — it does not stop at all.
- **How long after the command?** N/A — the command is rejected.
- **Does `play` continue the same trajectory?** N/A — `play` is also rejected; the
  trajectory was never paused, so it simply finished on its own.
- **How long can it stay paused before the action times out / aborts?** N/A — there
  is no pause; the goal ran normally to a successful completion.
- **State afterwards?** Unchanged and healthy: program_running `true`, goal
  succeeded, connection never dropped.
- **Does PAUSE drop the driver connection (as stopping the headless program does)?**
  **No.** Unlike dashboard `stop` (which tears down external control — report §2
  mechanism table), `pause` is a no-op-that-errors: `/joint_states` kept flowing
  and the goal was not aborted. It neither helps (no stop) nor harms (no drop).

This is exactly what the design report §2 mechanism table predicted for headless:
*"Headless (THIS cell): not a reliable pause — no loaded program to pause/resume
cleanly."* It is now empirically confirmed, not assumed.

## Q3 — Any other freeze-and-continue mechanism on this controller generation?

Evaluated briefly (no low-level control invented):

| mechanism | full freeze? | continue the SAME goal? | verdict |
|---|---|---|---|
| Speed slider → 0 | **no** (10 % floor, Q1) | yes (raise slider → same goal) | graded SLOW + resume only; not a full STOP |
| Dashboard `pause`/`play` (headless) | — | — | **rejected by the robot** (Q2) |
| Dashboard `stop` | yes | **no** — tears down external control; needs `resend_robot_program` + a fresh goal (report §2; RUNBOOK §4) | terminal, not freeze-and-continue |
| Cancel FollowJointTrajectory goal | yes (holds at position) | **no** — terminal; a fresh plan is required | the existing backstop |
| MoveIt runtime stop | yes | no — delegates to JTC cancel | == goal cancel |
| cuMotion pause/replan | n/a | n/a | plan-time only, no runtime pause |

Nothing delivers **both** a full stop **and** a same-goal continue. The only two
levers that touch a running goal are the slider (continues the same goal, but
cannot fully stop) and the cancel (fully stops, but is terminal).

## Q4 — Failure behaviour of the dashboard-pause candidate

Because `pause` is rejected outright in headless, adopting it as the STOP would be
**actively dangerous** — the gate would believe it commanded a stop while the arm
kept moving. Per candidate failure case:

- **Pause commanded while no program runs (== the headless case):** returns
  "Failed to execute: pause"; the arm does not stop. A gate trusting this reply
  would fail *open*, not toward stop.
- **Gate process dies while "paused":** there is no pause to hold; the arm keeps
  moving. Only the existing **independent goal-cancel backstop** (fires on gate
  heartbeat loss — `watcher_core.py`) actually stops it.
- **Dashboard connection drops:** irrelevant — the pause never took effect; the
  goal-cancel backstop is a separate mechanism on the action interface, unaffected.
- **Another party presses play on the pendant:** N/A in headless — `play` is
  rejected too, and there is no paused program to resume.

Every one of these is why the design made the **goal-cancel backstop** — not any
dashboard verb — the mechanism that must deliver the stop, fail-toward-stop.

---

## Decision — change nothing in the stop path; stop-then-replan is the designed behaviour

Because no mechanism on this headless CB3 controller gives a true
freeze-and-continue, per the task's decision rule the **stop path is unchanged**:

- **STOP** stays the **goal-cancel backstop** (arm decelerates to a hold at its
  location). A resume after STOP is a **fresh plan-gate-execute**, not a
  continuation of the frozen trajectory — the safe, designed behaviour, not a
  regression.
- **SLOW** stays the **speed slider** (graded, and a raise above the floor
  continues the *same* trajectory — the one freeze-and-continue behaviour that does
  work, but only above the ~10 % floor, so it can never be a full stop).
- The independent goal-cancel backstop still fires if a stop is not confirmed
  within the deadline, if the gate dies, or if the camera-state heartbeat is lost
  (unchanged; `watcher_core.py`).

"Stop at its location from the planned moving state … the robot keeps moving when
the safe distance is good enough" is therefore delivered as: slider-SLOW while a
person approaches (same trajectory), goal-cancel STOP when they are close (arm
holds on its path), and a fresh plan on the next healthy CLEAR + dwell. The literal
"continue the exact frozen trajectory" is **not available on this controller** and
this repo does not fake it.

## Regression guard

`sim/assert_pause_investigation.py` reads the run's `RESULT` and PASSES while the
finding holds (dashboard pause not executed in headless, no freeze-and-continue,
goal unaffected). If a future driver/PolyScope/config change ever makes headless
`pause` execute, the asserter FAILS loudly with **RE-EVALUATE** so the owner
revisits this decision rather than silently keeping stop-then-replan when a true
freeze became available. Offline-tested in `tests/test_pause_investigation.py`.

## Implication for the live test

On the real headless CB3 the same is expected but must be re-checked with the owner
present (`LIVE_TEST.md`): confirm dashboard `pause` is still rejected. The "if the
real installation runs a loaded External Control URCap program rather than headless
injection, re-measure whether `pause`/`play` then give a clean freeze-and-continue"
case **was measured** (`PENDANT_PAUSE_FINDING.md`) and confirmed — it does — and is now
available as the opt-in `pendant_pause` strategy. For the cell as it runs today
(headless), the commissioned STOP is the goal-cancel backstop and a resume after STOP
is a fresh plan.
