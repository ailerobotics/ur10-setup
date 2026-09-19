# Can the gate STOP by pausing a PENDANT-LOADED program and continue the SAME motion? — measured on URSim CB3, NON-headless

This is the direct follow-up the captain asked for after `PAUSE_RESUME_FINDING.md`:
*"do the test where you load the program on the pendant and then try pausing it."*
`PAUSE_RESUME_FINDING.md` measured the cell as it runs today — **headless** — where
there is no dashboard-loaded program, so `pause` is rejected. The captain's candidate
is the OTHER way of running the driver: an External Control program **loaded and
playing on the (teach) pendant**, so the dashboard has a real program to pause.

**Answer: YES — on a pendant-LOADED program, dashboard `pause` gives a true
freeze-and-continue on this CB3.** The arm stops dead on its planned path, the applied
speed scaling falls to **0** (a stop the gate can *confirm*, unlike slider-zero's ~10%
floor), the running `FollowJointTrajectory` goal **survives**, and `play` continues the
**same** trajectory to its original goal (`error_code 0`) with **no replan**. This is
exactly *"stop at its location from the planned moving state … keep moving when the
distance is good enough."* So this change **adds pendant-pause as a selectable STOP
strategy** (`stop_strategy: pendant_pause`) for a cell that runs a pendant-loaded
program — while **headless-cancel stays the default**, because that is how this cell
runs today (`ur-bringup.sh`).

Repeatable, self-asserting, everything on `ROS_DOMAIN_ID=82`, URSim CB3 `3.15.8`
(matching the cell's PolyScope 3.15.8), real `ur_robot_driver`, on its own docker
network, no GPU:

- `sim/investigate_pause_ursim.sh MODE=pendant [ACTION=…]` — one scenario; brings up
  URSim with the External Control URCap installed and the External Control `.urp`
  loaded + played, then probes (`sim/_pause_probe.py --mode pendant`) and asserts
  (`sim/assert_pause_investigation.py --mode pendant`).
- `sim/run_pendant_pause_matrix.sh` — the whole durability + failure matrix, each row
  a fresh bring-up. Logs under `../data/human-gate-sim-v1/ursim-pendant-pause/`.
- `sim/run_pendant_gate_e2e.sh` — the REAL `gate_driver_node` (stop_strategy=pendant_pause)
  + independent backstop watcher, driven by scripted camera states, freezing and
  continuing the mover's trajectory. Logs under `../data/human-gate-sim-v1/ursim-pendant-gate/`.

## How the pendant program is set up (documented, repeatable)

The non-headless setup follows the Universal Robots client-library docker-URSim path
(`sim/ursim_pendant/README.md` for provenance):

1. URSim CB3 runs on a docker network whose **gateway is `192.168.56.1`** — the
   External Control URCap 1.0.5's compiled-in default host IP (`DEFAULT_IP`, port
   `50002`). So the loaded program connects back to the driver **with no installation
   edit**; the driver is launched with `reverse_ip:=192.168.56.1`.
2. The **External Control URCap** JAR (copied from the installed driver's
   `resources/externalcontrol-1.0.5.urcap`) is mounted at `/urcaps`; PolyScope installs
   it on boot (`polyscope.log`: *"URcap enabling external control registering!"*).
3. The **External Control program** `external_control.urp` (created in PolyScope
   3.15.8) plus `default.installation` / `default.variables` are mounted at
   `/ursim/programs` (`sim/ursim_pendant/programs/`).
4. The driver is launched **NON-headless** (`headless_mode:=false`).
5. An operator action — dashboard `load external_control.urp` then `play` — starts the
   program; the driver logs *"Robot connected to reverse interface. Ready to receive
   control commands."* and the `scaled_joint_trajectory_controller` goes active.

The four program files are unmodified from the Universal Robots client library
(BSD-3-Clause; `sim/ursim_pendant/README.md`). SHA-256 of the committed copies:
`external_control.urp` `8a3a67a2…`, `default.installation` `c5ed2b08…`,
`default.variables` `01d3a7bb…`, `wait_program.urp` `9ad43a7e…`.

## Q1 — pause_play: does the arm freeze on its path and continue the same goal? YES

Method: one long, slow, monotonic-pan trajectory on the `scaled_joint_trajectory_controller`;
once it is clearly moving, dashboard `pause`, hold, `play`, watching the dashboard
replies, joint motion, the applied scaling, the JTC state, and the one
`FollowJointTrajectory` goal held throughout. Measured (clean runs, 5 s and 60 s holds):

| observation | value |
|---|---|
| dashboard `pause` reply | **`success=true`, "Pausing program"** |
| arm froze on its path? | **YES** — pan drift during the hold `0.0` rad |
| time from `pause` to standstill | **~1.8 s**; stopping distance on pan **~0.006–0.007 rad** |
| applied speed scaling while paused | **0.0** (a CONFIRMABLE full stop) |
| `program_state` while paused | **PAUSED** (`program_running=false`) |
| `FollowJointTrajectory` goal during pause | **still ACTIVE** (not aborted) |
| `/joint_states` flowing during pause | **YES** — the reverse connection did NOT drop |
| JTC controller state during pause | **active** (not deactivated) |
| dashboard `play` reply | **`success=true`, "Starting program"** |
| play continued the SAME goal? | **YES** — the one goal handle ran to **`error_code 0`**, no new goal sent, no replan |
| state afterwards | PLAYING, `program_running=true`, JTC active, goal succeeded |

This is the exact literal behaviour the captain asked for and that headless could not
provide.

### Why scaling → 0 here matters for the gate

The gate confirms a STOP on `/speed_scaling_state_broadcaster/speed_scaling` (a stop is
"effected" when scaling ≤ ~0.05). A pendant `pause` drives scaling to **0**, so:
- the gate core's primary→backstop fall-through sees the stop CONFIRMED and does **not**
  escalate to the cancel backstop; and
- the independent watcher's "STOP demanded but the slider never took effect" rule does
  **not** trip.
Contrast slider-zero, which floors ~10% (`SLIDER_ZERO_FINDING.md`) and therefore is
*never* a confirmable full stop — the whole reason the goal-cancel backstop is the
headless STOP.

### Stopping time and the backstop-deadline interaction (found by the gate e2e)

A dashboard pause freezes the arm over **~1.8 s** (the robot's own pause deceleration),
far slower than the slider's ~50 ms. Both the gate core's STOP→BACKSTOP fall-through
(`backstop_deadline_s`, default 0.5 s) and the independent watcher's "STOP demanded but
scaling not ~0" rule (`stop_effect_deadline_s`, default 0.7 s) are tuned for the fast
slider — so on an actively-moving goal they would cancel the pause **before it freezes**,
defeating freeze-and-continue and turning every pendant STOP into a stop-then-replan.
`sim/run_pendant_gate_e2e.sh` surfaced this. The fix: a SINGLE knob,
`pendant_freeze_confirm_s` (default 2.5 s), sizes BOTH deadlines. In pendant mode the
gate node widens its own `backstop_deadline_s` to it, and the independent watcher —
given the same `stop_strategy: pendant_pause` — sizes its own effective stop-effect
deadline to `max(stop_effect_deadline_s, pendant_freeze_confirm_s)` from it. `params.yaml`
expresses the value once (a YAML anchor shared by both process sections), so the two can
no longer drift; earlier the watcher's `stop_effect_deadline_s` had to be raised by hand
(and could be forgotten, silently reverting pendant_pause to cancel-and-replan). Both
deadlines must EXCEED the measured freeze time, and the backstop still fires if the pause
has not effected a stop by then. As a runtime backstop against a half-applied config, the
watcher also cross-checks the strategy the gate ANNOUNCES on `/human_gate/decision`
against its own; on any mismatch (e.g. gate pendant_pause, watcher left on cancel) it
reports the misconfiguration loudly and repeatedly (log + `/human_gate/watcher_status`)
while STILL cancelling at its own deadline — the arm always ends stopped, never silently
mis-timed. **Consequence:** a pendant-pause STOP has a ~1.8 s stopping time, so the STOP
ring must be sized for it (report §3 Tr/Ts budget); re-measure the real freeze time and
re-derive the deadlines on the real arm (`LIVE_TEST.md`). This is a genuine trade: the
pause gives a smooth full stop + same-trajectory resume, at a slower stop than the
(never-fully-stopping) slider.

## Q2 — durability: 5 s / 60 s / 10 min holds

Each a fresh bring-up (`sim/run_pendant_pause_matrix.sh`, logs per row under
`../data/human-gate-sim-v1/ursim-pendant-pause/`). Same slow trajectory, paused in the
constant-velocity middle, held, then played:

| pause hold | froze? | scaling while paused | goal survived the hold? | on `play`, original goal | verdict |
|---|---|---|---|---|---|
| **5 s**   | yes (~1.8 s, ~0.007 rad) | 0.0 | yes (active) | **succeeded, code 0, no replan** | freeze-and-continue holds |
| **60 s**  | yes (~1.8 s) | 0.0 | yes (active) | **succeeded, code 0, no replan** | freeze-and-continue holds |
| **10 min (600 s)** | yes (~1.8 s) | 0.0 | yes (active) | **succeeded, code 0, no replan** | freeze-and-continue holds |

The pause holds **indefinitely**: while paused the applied scaling is 0, the
`scaled_joint_trajectory_controller` stays **active**, and the trajectory's clock is
frozen (scaling 0), so the `FollowJointTrajectory` goal-time tolerance never expires —
a 10-minute pause resumes the same goal to `code 0` exactly like a 5-second one.
`resume_max_pause_s` (default 120 s) is therefore a **policy** bound (a frozen plan
older than this is treated as stale → cancel + replan), NOT a controller limit; tune
it to the cell's tolerance for a stale plan, ≤ any real-arm limit measured later.

## Q3 — failure behaviour (each an isolated fresh bring-up)

Every one of the six cases the brief lists still ends the arm STOPPED. Four are direct
matrix measurements (`sim/run_pendant_pause_matrix.sh`); two are covered by the
strategy-independent goal-cancel backstop already proven in `sim/run_mock_e2e.sh` +
`tests/test_watcher_core.py` (the watcher is a SEPARATE process and is unchanged by the
pendant strategy). The strategy's own rules are unit-tested in `tests/test_stop_strategy.py`.

| failure case | measured / how | outcome — always ends stopped |
|---|---|---|
| **pause commanded when no program is running** | matrix `pause_no_program` | dashboard `pause` is **not honoured** (it timed out; the arm did NOT freeze and kept moving). Pause is a stop ONLY when a program is playing — so the gate must NOT trust a pause reply as a stop; the independent goal-cancel backstop is what actually stops the arm here (it fires because scaling never reaches ~0). |
| **someone plays the program while the gate still wants STOP** (a foreign play / the re-arm watchdog) | matrix `foreign_play` + `tests/test_stop_strategy.py` | a foreign `play` DID resume motion (~0.059 rad/s). The gate detects the False→True `program_running` edge and **re-pauses**, surfacing the conflict (`test_pendant_foreign_play_during_stop_is_repaused`); if it cannot hold the pause, the watcher's "STOP demanded but scaling not ~0" rule cancels. |
| **the external-control program is STOPPED rather than paused** | matrix `stop_instead` | dashboard `stop` is **terminal**: program → STOPPED, the FJT goal **aborts** (code −1), the JTC deactivates, the reverse connection is torn down. NOT a same-goal continue — recovery needs `resend_robot_program` + a fresh plan. This is why the strategy uses `pause`, never `stop`. |
| **gate process dies while paused** | `run_mock_e2e.sh` + `test_watcher_core.py` (strategy-independent) | the arm is already frozen (paused); the independent backstop watcher additionally cancels the goal on gate-heartbeat loss, so the arm stays stopped even though the primary path is dead. |
| **camera-state / dashboard connection drops while paused** | `pause_only` (proxy) + watcher heartbeat rule | a pause with no subsequent play stays frozen for the whole hold (measured drift `0.0` over 20 s, scaling 0). A dropped dashboard means no `play` can be issued → the arm stays frozen (fail-safe); a lost camera-state heartbeat trips the watcher → cancel. Either way: stopped. |
| **protective stop occurs while paused** | observed during bring-up (robot-side safety stop) | a CB3 protective stop holds the arm (the certified safety behaviour) and is terminal for the frozen goal: recovery requires `unlock protective stop` + `resend_robot_program` + a fresh plan (as `ur-rearm-watchdog.sh` already does). Ends stopped; a resume is a fresh plan. |

## Q4 — comparison with the headless finding (side by side)

| | HEADLESS (`PAUSE_RESUME_FINDING.md`, today's cell) | PENDANT-LOADED (this finding) |
|---|---|---|
| dashboard `pause` | **rejected** ("Failed to execute: pause") | **accepted** ("Pausing program") |
| loaded program | none (`loaded_program=""`, driver injects the script) | `external_control.urp` loaded + playing |
| arm on `pause` | does not stop — trajectory runs to completion | **freezes on its path in ~1.8 s** |
| applied scaling on `pause` | unchanged (still moving) | **0.0** |
| FJT goal | unaffected, succeeds on its own | **frozen, survives, continues on `play`** |
| `play` | rejected — nothing to resume | **resumes the SAME goal to `code 0`** |
| freeze-and-continue? | **NO** — STOP = goal-cancel backstop (stop-then-replan) | **YES** — STOP = pause, RESUME = play (same trajectory) |

So the two findings are consistent: *pause acts on a loaded PolyScope program*. The
cell has no loaded program headless (pause is a dangerous no-op there), but a
pendant-loaded program gives a real freeze-and-continue. The stop path therefore
**depends on how the driver is run**, which is why this is a **selectable strategy**,
not a change to the default.

## Decision — add pendant-pause as an opt-in STOP strategy; keep headless-cancel default

- **`stop_strategy: cancel` (DEFAULT, unchanged):** how this cell runs today (headless).
  STOP = slider→0 with the goal-cancel backstop delivering the real stop
  (stop-then-replan). Nothing in the shipped stop path changes.
- **`stop_strategy: pendant_pause` (NEW, opt-in):** for a cell that runs a pendant-loaded
  External Control program. STOP = dashboard **pause** (freeze on path); a healthy
  CLEAR+dwell RESUME = dashboard **play** (continue the same trajectory). In BOTH
  strategies the **slider stays the SLOW lever** and the **independent goal-cancel
  backstop stays the net** — it still fires if the freeze is not confirmed within the
  deadline, if the gate dies (watcher heartbeat loss), or if the camera-state heartbeat
  is lost. Extra rules the strategy enforces (unit-tested in
  `tests/test_stop_strategy.py`):
  - RESUME (play) happens **only** on the gate's own RUN — i.e. after a full healthy
    CLEAR held for the dwell — never on a mere SLOW retreat and never on invisibility;
  - RESUME is **refused once the pause has lasted longer than `resume_max_pause_s`**
    (the frozen plan is treated as stale): the strategy cancels instead, so the resume
    becomes a fresh plan;
  - a **foreign play** while the gate still wants STOP (someone plays the pendant, or
    the re-arm watchdog — see below) is **re-paused** and the conflict is surfaced.

This is a supplementary awareness layer, **NOT a certified safety function** (the CB3
pendant Force limit and any safeguard-input presence device remain the real stops).

## What running the cell pendant-loaded instead of headless would mean (owner decision)

Switching this cell from headless to a pendant-loaded program is an **operational**
change with real consequences — presented here as options, NOT changed by this task
(`ur-bringup.sh` is untouched, `stop_strategy` defaults to `cancel`):

- **Someone must start the program.** Headless self-injects on driver launch; a
  pendant-loaded program must be `load`+`play`ed (an operator on the pendant, or a
  startup script via the dashboard) before motion works.
- **The re-arm watchdog conflicts with pause.** `ur-rearm-watchdog.sh` re-arms whenever
  `robot_program_running` is false for >5 s — but a pendant-pause STOP *sets*
  `program_running=false`. In pendant mode the watchdog would `play` the program back
  and fight the gate's STOP. This is the exact "foreign play while STOP wanted" case the
  strategy re-pauses and surfaces, but it must not be left to that: **the watchdog must
  be made pause-aware (skip re-arm while the gate holds STOP) or disabled in pendant
  mode.** Owner decision.
- **The External Control host IP** on the real pendant must be set to the workstation's
  address on the robot network (`192.168.0.2` in this cell), not the sim default.
- **Recommendation:** keep the cell **headless (cancel strategy)** as the default and
  adopt `pendant_pause` only for a specifically-commissioned pendant-loaded cell, with
  the watchdog change above made first. The pendant-pause resume is a genuine UX/again
  improvement (no replan on every clear), but it trades away headless auto-recovery.

## Note on continuously-replanning (tracked-pick) motion — flagged, NOT built here

The captain also asked to think about the case where "the planning changes all the time
dynamically depending upon where the perception perceives the object and tracks it to
pick it up." That is a **separate design investigation** and is deliberately NOT part of
this change; a short orientation so the two are not conflated:

- Pendant-pause freeze-and-continue is a match for **a single committed trajectory**
  interrupted by a person: freeze it in place, then continue the *same* motion when
  clear. It preserves exactly the plan that was executing.
- A **continuously-replanned tracked pick** is different: the goal itself keeps changing
  as the target moves, so "continue the frozen trajectory" is usually the WRONG thing —
  the right resume is a *fresh* plan to the target's new pose. There, pendant-pause's own
  `resume_max_pause_s`→cancel-and-replan path (and the goal-cancel backstop) is the
  natural behaviour, and the freeze-and-continue benefit mostly disappears.
- So the two strategies are complementary: `pendant_pause` for freezing a fixed planned
  motion around people; `cancel` (stop-then-replan) for motion that is being replanned
  every cycle anyway. Which the tracked-pick pipeline wants is for that separate
  investigation to decide; this change only makes the freeze-and-continue lever exist and
  proves it, without changing the default.

## Regression guard

`sim/assert_pause_investigation.py --mode pendant` PASSES while pendant `pause_play`
delivers the freeze-and-continue and FAILS with **RE-EVALUATE** if a future
driver/PolyScope/config change stops delivering it (so the owner revisits the strategy
before shipping). Offline-tested in `tests/test_pause_investigation.py`; the strategy
logic itself in `tests/test_stop_strategy.py`.

## Implication for the live test

On the real CB3 this must be re-measured with the owner present before arming
`pendant_pause` — see `LIVE_TEST.md` ("Pendant-pause commissioning"): confirm `pause`
freezes the loaded program, the goal survives and continues on `play`, and re-measure
the real stopping time/distance and the safe maximum pause duration. Until then the
commissioned STOP stays the goal-cancel backstop (`cancel` strategy).
