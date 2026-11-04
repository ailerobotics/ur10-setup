# Contact guard — early-failure detection for the real fork pick-and-place

`fork_demo/contact_guard/` is a supplementary **early-failure** layer for the
real UR10 CB3 fork pick-and-place. It watches the running motion, trips on a
contact fault *before* a fork tine or a dock is damaged, cancels through the
existing cancel-all path, optionally backs off, and HOLDS. It is built and
proven **offline and on URSim only** — nothing here commands the real arm, and
the cell is powered off.

> **This is NOT a certified safety function.** The real stops remain the
> teach-pendant safety Force limit and the person at the e-stop. This layer adds
> an *earlier, smarter* trip on the existing signals; it is a provisional guard
> built from one teach record (`teach-002`, 2026-10-22).

## The CB3 reality this is built for

This CB3 has **no wrist force/torque sensor**. The wrench on
`/force_torque_sensor_broadcaster/wrench` is estimated from joint currents and,
*in motion*, swings to **~427 N / ~1471 Nm** at a wrist-flip configuration on
clean, contact-free motion (the 200 N watchdog false-tripped and had to be
raised to 400–600 N during the session). Meanwhile the parts break far below
that: a dock channel wall at **~15–100 N**, a printed fork tine at
**~140–350 N** (sibling scout `data/ur10-pick-failure-modes-s1/report.md`). So:

- **In motion, the estimated force can only *corroborate*** — it is never
  sufficient to trip and never a single global newton number.
- **The trustworthy in-motion signals are the tracking error and an
  actual-progress stall**: a real contact physically stops the arm, so the
  commanded trajectory runs away from the measured joints (tracking rises) and
  the measured progress freezes (stall). A force-estimate artefact leaves motion
  unimpeded; a benign loaded/coarse-waypoint lag keeps advancing.
- **If the arm HALTS at the contact pose and captures a fresh wrench baseline
  there, the pose bias cancels** and a baseline-relative wrist-torque delta
  resolves **~3–5 N of tip force** (the 380 mm fork is a lever:
  tip force = torque ÷ 0.38 m). This is the single biggest lever the software
  has, so every contact segment is run as **stop-and-probe**.

## Signals

| signal | what | role | source |
|---|---|---|---|
| `force_mag`, `f_ins`, `f_lat` | fork-frame force magnitude, insertion-axis (fork +z), lateral | corroboration (band) + probe delta | wrench → fork tip |
| `torque_mag`, `t_pry` | fork-tip torque magnitude, prying (lateral) torque | corroboration + **first-class in probe** | wrench → fork tip |
| `eff_max` | max abs joint effort (current-estimate) | corroboration (band) | `/joint_states.effort` |
| `tcp_vel` | TCP speed | band / stall support | `/tcp_pose_broadcaster/pose` |
| `track_err` | commanded-vs-actual joint error | **trip (rising) + stall** | plan vs `/joint_states` |
| actual progress | measured joints projected on the plan | **stall discriminator** | plan + `/joint_states` |

The wrench is published in `tool0_controller`; it is shifted to the fork tip
(`M_fork = M_tool0 − offset × F`, `offset = [0, −0.038, 0.427]`) so a tip
contact reads as a wrist torque. Force components are unchanged (the fork TCP is
a pure translation of tool0).

## Detection model (families and trips)

Independent evidence families: **F** (force/effort band), **P** (baseline-relative
probe delta), **T** (tracking rising), **K** (actual-progress stall),
**L** (sustained carry load step). Trips, each with persistence:

- **PROBE** — during a stop-and-probe creep, a baseline-relative |F| or wrist
  torque delta past the probe floors (first-class; resolves ~3–5 N tip force).
- **STALL** — commanded progress advances while the measured arm is frozen
  (K; tracking T co-fires). The primary, reliable in-motion trip.
- **EARLY** — force out of band **and** tracking *rising* together (a hard
  contact building resistance, caught before a full stall).
- **LOAD_STEP** — a sustained force-band excursion across advancing progress on
  a carry leg (an added load / tray slide, distinct from a pose-locked estimate
  bump that clears with the pose).
- **STALE** — no fresh sample within `stale_s` → fail toward stop.

Never trips on **force alone**, on a benign plateaued tracking lag, or when the
trajectory is not executing. `execute_plan.py`'s own absolute `--max-track-rad`
backstop (0.15 rad) is a **separate, unchanged** layer.

## Leg-aware signal envelopes

`build_envelopes.py` reads the teach-002 bag + metadata, segments the continuous
record into the 12 plan legs by dwell window, and builds, **per leg and per
normalised-progress bin**, a baseline + tolerance band for every bag-observable
signal. Band half-width is `max(k·robust_sigma, absolute_floor, observed_range)`
with a small interpolation pad, so the pose-dependent force-estimate drift
(the 427 N wrist-flip artefact is clean motion) sits **inside** the band by
construction while a real contact step still leaves it. `track_err` needs a
commanded trajectory, so it is **not** banded on a freedrive teach and is a
runtime-only signal.

The committed bands are `envelopes/teach-002.yaml` (small, human-diffable). The
bag itself is **never** copied into the repo.

## Measured results

### Clean record — zero false trips

`cli.py dry-run` replays the detector over the whole clean teach-002 record:

```
TOTAL: 0 trip(s) over 11 legs; band margin 0.91 (headroom to the band edge).
```

Zero false trips, with **0.91** of the band half-width still free at the
closest approach. On a freedrive teach only the force family is observable, and
force alone never trips, so a clean record cannot false-trip by construction —
the margin shows the force band also *contains* the record with room to spare.

### Injected faults — all six detected (5 % speed)

| fault | leg | trigger | travel past contact | latency | \|F\| at trip | below 200 N by |
|---|---|---|---|---|---|---|
| tine hits dock post / front rail (insert) | insert_forks | STALL | 9.9 mm | 460 ms | 8 N | 192 N |
| lift with a tine under a post | lift | STALL | 8.0 mm | 450 ms | 17 N | 183 N |
| tray snag on extract | lift_from_table | STALL | 55 mm | 460 ms | 18 N | 182 N |
| set-down onto a misaligned dock | lower_on_table | STALL | 11.5 mm | 460 ms | 13 N | 187 N |
| collision during carry | carry_to_table | STALL | 42 mm | 460 ms | 11 N | 189 N |
| tray sliding on the forks (carry) | carry_to_table | LOAD_STEP | ~1020 mm | ~11.2 s | 70 N | 130 N |

The five motion-impeding faults trip in **< 0.5 s** and **< 55 mm** of intrusion,
far below both the old 200 N watchdog and the 100 N pendant floor. **Tray-slide
is the blind spot**: a load shift on a *free* carry does not impede motion, so it
is caught only as a sustained load step (latency depends on where in the carry
it happens, and can be large). The reliable guarantee that the tray is on the
forks is the **post-LIFT verification**, not the carry monitor.

### Stop-and-probe — detectable tip force per leg

A gentle **6 N** tip touch during a stop-and-probe creep trips **PROBE in ~30 ms**
and reports the inferred tip force within ~1 N — **94 N below** the 100 N pendant
floor. The noise-limited resolution is **~3.9 N** of tip force on every leg
(the ~1.5 Nm torque floor ÷ 0.38 m lever), which is **below the dock channel
wall damage threshold (15–100 N)**. Contrast the *in-motion* band resolution:

| contact leg | contact travel | in-motion detectable tip force | stop-probe tip force | segments (≤15 mm) | added dwell time |
|---|---|---|---|---|---|
| insert_forks | 639 mm | ~149 N | **3.9 N** | 43 | +17.2 s |
| lift | 222 mm | ~130 N | **3.9 N** | 15 | +6.0 s |
| insert_slot | 376 mm | ~140 N | **3.9 N** | 26 | +10.4 s |
| lower_on_table | 723 mm | ~135 N | **3.9 N** | 49 | +19.6 s |
| withdraw | 20 mm | ~87 N | **3.9 N** | 2 | +0.8 s |
| lift_from_table | 4495 mm* | ~317 N | **3.9 N** | 300 | +120 s |

\* the teach `lift_from_table` leg bundled a long transit move; the true contact
portion (the pick-off the table) is short. **Cycle-time cost:** one
`probe_dwell_s` (0.40 s) halt per segment. Stop-and-probe is the only mode that
sees contact *below the parts' failure loads* on this sensorless arm — the extra
dwell is the price of seeing a 3.9 N touch instead of a 150 N one.

### Named near-misses — none trip

The session's real near-misses are replayed as negatives and **do not trip**:
the 427 N and 318 N force-estimate artefacts (force only, tracking plateaued, no
stall), the 8.64 deg loaded-lift lag (tracking rises then plateaus, arm keeps
advancing), and the External Control drop (not executing → stand down).

## Reaction policy

On a trip the live node **cancels the running goal** through the existing
cancel-all path (`action_msgs/CancelGoal`, the same path `force_abort.py` uses).
Then, on a **contact** leg, it plans a short **back-off along that leg's own axis**
(default **15 mm at 2 %**) — but only if the back-off target stays inside the
workspace box and joint limits; otherwise it **HOLDS**. Transit legs and a STALE
trip always HOLD. It **never retries automatically** and prints a one-screen
verdict (leg, signal vs band, travel at trip). Parameters live in `params.yaml`
under the project's **tune-up-never-down** convention (a yaml may only make
detection tighter than the baked-in floor; `params.validate()` rejects a
loosening value).

## Pre-contact gate

`gate.py` evaluates, before any contact leg (fail toward stop on any missing
value): fork TCP + payload read back from the robot (fork TCP z ≥ 0.30, |y| ≥
0.01; payload in the unloaded/loaded band), speed scaling ≤ 10 %, RUNNING +
NORMAL, External Control running, live `/joint_states` (reuses
`check_joint_feed_live.py`), ≥ 20 GB free disk, an **operator-typed pendant Force
limit** (it cannot be read on CB3 — the gate records what was typed and flags it
if it is not the scout-recommended **100 N** with the correct payload), and an
**external alignment veto**. The gate exposes a JSON verdict
(`contact_guard.gate_verdict.v1`); a separate alignment lane (not implemented
here) vetoes it through the documented `contact_guard.align_veto.v1` interface
(`veto.py`) — a missing, malformed or stale verdict vetoes.

## Post-action verification

`verify.py`: after **LIFT**, confirm the tray is on the forks — a TCP height
rise (≥ 15 mm, the reliable kinematic signal) plus a wrench step consistent with
the loaded band; because the wrench step is **marginal** on this sensorless arm
(a full tray ~24 N vs ±9–19 N same-pose repeatability), a marginal step is
accepted on the TCP rise and flagged, but a *conclusive* wrench drop refuses to
continue. After **LOWER**, confirm the load left the forks (residual below
20 N) before WITHDRAW. A mismatch refuses to continue.

## Incremental contact legs

`segmenting.py` cuts each contact leg into ≤ 15 mm TCP segments, each densified
to ≥ 12 points (the 132-point lesson: dense waypoints kept the real tracking
excursions from 12.6 deg down to 3.6 deg), and each run as DWELL → fresh at-pose
baseline → CREEP with the between-sample check → CHECK before the next. Transit
legs keep their existing path.

## Integration

The guard is wired into the real toolchain behind an opt-in flag that is **on by
default for contact legs**:

- `step_replay.py` launches `contact_guard/node.py` alongside `force_abort.py`
  for each contact segment; `--no-contact-guard` opts out.
- `real_fork_pick.py` adds the pre-contact gate + the guard to its (plan-only)
  hand-off; `--no-contact-guard` opts out.
- Both accept `--guard-dry-run <bag>`: run the detector over a recorded bag and
  print per-leg verdicts (offline, no motion). The teach-002 dry-run above is
  that output.

## Run it

```bash
# offline fault + negatives report (pure python, no bag, no ROS)
python3 -m fork_demo.contact_guard.cli inject

# guard dry-run over the teach-002 bag (needs the mcap reader; zero false trips)
~/venvs/contact-guard/bin/python -m fork_demo.contact_guard.cli dry-run

# (re)build the committed envelopes from the bag
~/venvs/contact-guard/bin/python -m fork_demo.contact_guard.build_envelopes

# runtime mock end-to-end proof (no ROS, self-asserting)
python3 fork_demo/contact_guard/sim/run_mock_e2e.py

# offline unit tests
python3 -m pytest fork_demo/contact_guard/tests/ -q

# URSim CB3 hardware-in-loop proof (segments, cancel, back-off) — captain/CI,
# needs the universalrobots/ursim_cb3 docker image; task-only ROS_DOMAIN_ID=86
fork_demo/contact_guard/sim/run_ursim_e2e.sh
```

The mcap bag is read in a venv (`~/venvs/contact-guard`, `pip install mcap
mcap-ros2-support`); it is never copied into the repo.

## Staged live-test protocol (captain's stage-wise commanding)

Every physical stage below is run **only** after firstmate relays the literal
go token `CAPTAIN GO: <STAGE>` (the exact stage name from the table). That token
is the sole authorisation for a physical stage and must appear in **no other
context** — not in code, not in a default, not as an example. Each stage runs
with a person at the e-stop, the pre-contact gate passing, and the contact guard
armed on every contact leg.

| `<STAGE>` | what moves | go criteria | no-go / abort |
|---|---|---|---|
| `AIR_INSERT` | insert path 150 mm **in front** of the dock (no contact) | gate passes; guard arms; clean band dry-run | any trip; tracking > band |
| `INSERT_NO_TRAY` | insert into the dock with the **tray removed** | air-insert clean; dock aligned | PROBE/STALL trip; tine contacts a channel wall |
| `INSERT_WITH_TRAY` | insert under the real tray | insert-no-tray clean; tray seated | PROBE trip (tip > 3.9 N); stall |
| `LIFT_10MM` | lift 10 mm | insert clean; post-insert verify | prying torque (PROBE); no TCP rise |
| `LIFT_FULL` | full lift (~23 mm off the posts) | lift-10 mm clean; **post-LIFT verify: tray on forks** | tine-under-post PROBE/STALL; verify mismatch |
| `EXTRACT` | extract the lifted tray | lift verified | snag (STALL); tray shift (LOAD_STEP) |
| `CARRY` | carry across the cell | extract clean; load steady | collision (STALL); tray slide (LOAD_STEP) |
| `LOWER` | set the tray onto the far dock | carry clean; dock aligned (alignment veto OK) | early/lateral load (STALL); misaligned dock |
| `WITHDRAW` | withdraw the forks | **post-LOWER verify: load released** | residual load; snag |

A trip at any stage cancels the goal, HOLDS, and prints the verdict; the
operator inspects the forks/dock and relays a **fresh** go token to resume —
the guard never retries on its own.

## Files

- `params.py` / `params.yaml` — parameters (tune-up-never-down).
- `signals.py` — fork-frame transform, derived signals, progress projection.
- `envelope.py` — leg/progress band model, builder, serialisation.
- `bagsource.py` — read the teach/replay mcap + metadata into leg time series (lazy mcap).
- `build_envelopes.py` — build the committed `envelopes/teach-002.yaml`.
- `detector.py` — the runtime detector (families, trips).
- `faults.py` — offline fault injection + the named negatives + the report.
- `reaction.py` — cancel / back-off / HOLD + one-screen verdict.
- `segmenting.py` — stop-and-probe segment plan.
- `gate.py` — pre-contact gate (pure).
- `veto.py` — external alignment-veto interface.
- `verify.py` — post-LIFT / post-LOWER verification.
- `runner.py` — the facade the node / mock / integration drive.
- `node.py` — the ROS live watcher (lazy rclpy).
- `cli.py` — `dry-run` / `inject` / `build`.
- `dryrun.py` — the bag-replay proof core.
- `sim/` — `run_mock_e2e.py` (in-session proof), `run_ursim_e2e.sh` + helpers.
- `tests/` — offline unit tests + the bag-replay test.

Decision record and the full numbers: `data/ur10-pick-contact-safety-v1/`
(under `~/firstmate`, not the repo).
