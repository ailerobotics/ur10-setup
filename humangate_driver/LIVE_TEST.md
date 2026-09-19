# Owner-present live commissioning of the human-safety gate (Phase 4)

**STATUS: written, NOT run.** This is the real-arm test procedure for the
driver-backed gate. It is the analog of the repo's captain-execute notes
(`data/cell-clean-approach-v1/captain-execute-note.md`) and follows report §8.
It runs ONLY with the owner physically present, an e-stop in hand, and after the
sim proofs (mock + URSim) are green. Real-robot arming is a change this repo does
NOT yet contain (see below) — do not attempt to arm the real arm by editing
config alone.

## This is not a certified safety function

Before anything: this camera layer is a supplementary awareness aid, NOT a
certified safety function (report §0/§4). The real protection during this test is
(1) the CB3 teach-pendant **Force limit** set at the 100 N floor with the correct
Robotiq 2F-85 + tray payload, and (2) the owner's hand on the e-stop. If either is
not in place, do not run.

## What must be true first (prerequisites)

1. `humangate/` + `humangate_driver/` offline tests all green
   (`pytest` in each; see `humangate_driver/README.md`).
2. Sim proofs green and their logs committed: `sim/run_mock_e2e.sh`,
   `sim/run_ursim_e2e.sh`, and the pause investigation `sim/investigate_pause_ursim.sh`
   (logs under `data/human-gate-sim-v1/`).
3. The camera side (`cams/human_safety`) is live and publishing `/human_safety/state`
   with real detections, on the SAME domain the gate will use, and at least one
   camera has a trustworthy `base_link` pose (report §6). Confirm the state topic
   and its `schema_version` match `humangate/state_adapter.py` (EXPECTED_SCHEMA_VERSION).
4. The robot line is up: `ur10-driver` running, robot RUNNING/NORMAL with its
   program up, speed scaling > 0 (RUNBOOK §4/§5), dashboard reachable.
5. Pendant Force limit set at 100 N with the correct payload (report §11.4).
6. **One writer only.** Announce that the gate is taking the speed slider; no other
   session or person touches the pendant slider or sends the arm goals during the
   test (RUNBOOK §6b).

## Real-robot arming is a SEPARATE change (do this deliberately, not by config)

The arming interlock (`arming_interlock.py`) refuses the real robot by design: it
demands a non-0/42 domain AND a loopback/Docker-internal `robot_ip`, and refuses
192.168.0.1 and any physical-NIC address. Commissioning on the real arm therefore
requires a deliberate, reviewed code change to allow the real robot address under
an explicit, owner-gated flag — it must NOT be reachable by passing a launch
argument to today's code. Design and land that change (with its own review) before
this test. Until then, this procedure cannot point at the real arm.

## Procedure (owner present, e-stop in hand)

Everything at the cell's operating 5% trajectory scaling. Use a `loop_tags`-style
slow leg as the motion under test (`cams/cellmap/tools/loop_tags.py` /
`execute_plan.py --limits <measured joint limits>`), NOT a fast move.

1. **Bring up, disarmed dry-run first.** Start the gate against the live camera
   state with the driver-backed port but the arm NOT moving; confirm the gate
   tracks the person (its `/human_gate/decision` follows the camera zone) and that
   its recorded commands are what you expect, before any motion.
2. **Arm + start the slow leg.** With the owner clear of the cell, start the
   `loop_tags` leg; confirm the gate holds RUN and the slider is at the operating
   fraction, and the independent backstop watcher is running as its own process.
3. **Approach test.** Owner walks slowly from OUTSIDE the SLOW ring toward the arm
   and stops AT the STOP ring, then withdraws past the CLEAR boundary. Log the
   photon->stop latency.
4. **Kill-the-publisher test.** Mid-approach, stop the camera-state publisher;
   confirm the arm stops within the heartbeat timeout.

## Pass criteria (all must hold)

- (1) The arm visibly **slows** on entering SLOW.
- (2) The arm **stops and holds on its path** (not a protective stop, not a jerk)
  before the owner reaches the STOP ring, with the measured photon->stop latency
  logged.
- (3) The arm **stays stopped** for the full clear dwell and does NOT resume while
  the owner is still inside a ring or merely occluded.
- (4) The arm **resumes** only after CLEAR + dwell. On this cell's HEADLESS driver
  the commissioned STOP is the **goal-cancel backstop**, so a resume after STOP is a
  **fresh plan-gate-execute** (stop-then-replan), not a continuation of the frozen
  trajectory — the sim proved neither slider-zero nor dashboard pause gives a full
  freeze-and-continue on headless CB3 (`SLIDER_ZERO_FINDING.md`,
  `PAUSE_RESUME_FINDING.md`). The graded SLOW + same-trajectory resume via the slider
  still applies **above the floor** (a person approaching → slower same motion). Record
  the real arm's behaviour and confirm it matches; if it does not, re-run the finding.
- (5) Killing the state publisher mid-approach stops the arm within the heartbeat
  timeout.
- (6) A foreign slider write (have a second session nudge the pendant slider) makes
  the gate STOP and surface the conflict (sole-writer discipline).

## Measure and record on the real arm (report §8/§10 Phase 4)

- **Reaction time** (photon -> arm begins to react): the full latency chain Tr.
- **Stopping time and stopping distance at operating speed** (Ts, Ss).
- Whether **slider zero fully halts** or a floor applies on the real CB3 (compare
  to the URSim finding, `SLIDER_ZERO_FINDING.md`).
- The **command-to-effect delay** for the slider on the real driver.
- Whether **dashboard `pause`/`play` are still rejected** on the real headless arm
  (compare to `PAUSE_RESUME_FINDING.md`): call `/dashboard_client/pause` against a
  running slow leg and confirm it returns "Failed to execute" and the arm does not
  freeze. If the real installation instead runs a *loaded* External Control URCap
  program (not headless injection) and `pause`/`play` then give a clean
  freeze-and-continue on the scaled JTC, that is a material change — revisit the
  stop-path decision (re-run `sim/investigate_pause_ursim.sh`; its asserter will flag
  RE-EVALUATE) before commissioning pause as the STOP.

Feed these back into `humangate/gate_params.yaml`, re-deriving the provisional
zone radii / dwell / latency budget (report §3). Tune UP only if a measurement
demands it; never loosen below a measured-safe value.

## Pendant-pause commissioning (owner-present; written, NOT run)

This section is ONLY for a cell that will run `stop_strategy: pendant_pause` (a
pendant-LOADED External Control program, non-headless). The default `cancel` strategy
does NOT need it. It re-checks on the real CB3 what `PENDANT_PAUSE_FINDING.md` measured
on URSim. **Do this with the owner present, e-stop in hand, at 5 % trajectory scaling,
after the sim proofs are green.** It is NOT a certified safety function.

Before starting, decide the re-arm question (report + finding): a pendant-pause STOP
sets `program_running=false`, and `ur-rearm-watchdog.sh` re-arms on that after 5 s —
which would fight the gate's STOP. **Stop the `ur10-rearm` unit (or make it pause-aware)
before this test**, or the arm will resume under you.

What to LOAD: on the pendant, load the External Control program (the one whose
External Control installation host IP is set to the workstation's robot-network address
`192.168.0.2`, port `50002`, and that connects to the running driver). Confirm the
dashboard shows it loaded and `PLAYING`, and the driver logs "Robot connected to the
reverse interface".

What to PRESS / watch (arm the gate with `stop_strategy:=pendant_pause`, start a slow
`loop_tags` leg):
1. Walk from outside the SLOW ring toward the arm to the STOP ring. Watch: the arm
   **slows** (slider) then **freezes on its path** (dashboard pause) — `program_state`
   PAUSED, applied scaling 0, the trajectory goal NOT aborted. Log photon→standstill
   time and the stopping distance.
2. Stay close for a while (test 5 s, then ~60 s), then withdraw past the CLEAR
   boundary and stay out for the full clear dwell. Watch: the arm **resumes the SAME
   trajectory** on `play` and the original goal completes with no replan.
3. Force a **foreign play**: while still inside a ring (gate wants STOP), have a second
   session press `play` on the pendant. The gate must **re-pause** within a cycle and
   surface the conflict; the arm must not stay moving.
4. Force a **long pause**: stay close past `resume_max_pause_s`. On the next CLEAR+dwell
   the gate must **refuse the same-trajectory resume** and instead cancel + require a
   fresh plan (the frozen plan is treated as stale).

Pass criteria (all): pause freezes the arm on its path (scaling 0, goal not aborted);
play continues the SAME goal with no replan within the measured-safe pause duration;
a foreign play is re-paused; a pause older than `resume_max_pause_s` becomes a
cancel+replan; every fault (kill the state publisher, drop the dashboard, protective
stop) still ends the arm STOPPED (the goal-cancel backstop remains the net).

Measure and record on the real arm: whether `pause` freezes and `play` continues the
same goal (or a floor/abort appears), the real stopping time/distance, and the safe
maximum pause duration; feed the safe max into `resume_max_pause_s` (tune DOWN only).

Abort: any resume while a person is inside a ring; any pause that does NOT freeze the
arm; any protective stop; any case where a foreign play is NOT re-paused. Abort action:
e-stop, or the `loop_tags` STOP file; the gate leaves the slider at zero on shutdown.

## Abort conditions (any -> abort, investigate before continuing)

- Any protective stop / pendant popup.
- Any resume while the owner is inside a ring or occluded.
- Any latency exceeding the Tr the STOP ring was sized for.
- Any foreign-writer or external-control-program-drop event.
- Abort action: e-stop, or touch the STOP file the `loop_tags` leg watches; the arm
  holds. Then power down the gate (it leaves the slider at zero on shutdown).
