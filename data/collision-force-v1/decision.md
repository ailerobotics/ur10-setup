# Collision-force reduction — decision + captain bring-up note (collision-force-v1)

**Goal:** make the arm give up sooner when it meets something unexpected (a
person, or the shelf robot). Prepared plan-only; **no arm motion was commanded.**

## TL;DR (this is also the PR body)

- **Repo-settable change:** the ROS-side software contact-force backstop drops
  from **300 N → 200 N** (newtons of estimated external-force deviation), now
  living in one documented place, `cumotion/collision_force_limits.yaml`, and
  consumed by `loop_tags.py`.
- **Reasoning (two sentences):** 200 N stays clear of the worst clean-motion
  force deviation ever measured on this CB3 (151.8 N, nothing touching the arm),
  so it will not nuisance-trip, while cancelling on gross contact at about a third
  less push than the old 300 N. The real, sensitive, safety-rated collision-force
  protective stop is the **pendant Safety Force limit**, which on this CB3 cannot
  be set from software at all and must be set by the captain on the teach pendant.

## Where the threshold actually lives (the most useful part)

This arm is a **UR10 CB3, PolyScope 3.15.8, with NO wrist force/torque sensor.**
Contact force is *estimated* from joint currents. Two independent mechanisms can
stop the arm on contact, and they live in completely different places:

| # | Mechanism | Units | Set where | Repo-settable? | Safety-rated? |
|---|-----------|-------|-----------|----------------|----------------|
| 1 | **Robot protective stop — the real collision-force threshold** | N | Teach pendant: `Installation → Safety → Safety Limits → Force` | **NO** | Yes |
| 2 | ROS-side software contact-force backstop (cancels the running trajectory) | N | `cumotion/collision_force_limits.yaml` → `loop_tags.py` | **YES** | No |
| — | Payload (mass + CoG) — desensitises #1 if wrong | kg / m | Pendant `Installation → General → Payload` (or ROS `set_payload`) | Gray area¹ | feeds #1 |
| — | Tracking-error guard (the *effective* ROS-side contact detector on CB3) | rad | `execute_plan.py` / `loop_tags.py` `--max-track-rad` | YES (already tuned²) | No |

¹ Payload can technically be pushed over ROS (`set_payload` service / the URDF
`gpio payload` interface), but this repo does not command it and setting it wrong
is dangerous — treat it as a captain pendant/installation action.
² `--max-track-rad` is already tuned right at its nuisance floor (0.05 rad was
tried and false-tripped a clean cycle; it sits at 0.08 rad in the loop). This
change does **not** touch it — lowering it would trip clean work.

**Bottom line:** the thing the captain is asking to lower — the force at which the
robot itself trips a protective stop — is mechanism #1, and it is **pendant-only.**
The repository can only lower mechanism #2, a coarse backstop. Both changes are
below; #2 is done here, #1 is the captain's to make.

## Evidence for the current value and units

- **Units = newtons.** The guard compares `WrenchStamped` force deviation (N) and
  UR's safety Force limit is in N.
- **Current repo value = 300 N**, the hard-coded `--max-force-n` default in
  `loop_tags.py`, described there as a "GROSS contact backstop only".
- **Measured, not guessed** (`loop_tags.py:197-209`, full clean cycle at 5%,
  2026-08-28): clean-motion force deviation ran **27.8–151.8 N with nothing
  touching the arm**; a **150 N** backstop **false-tripped** a clean cycle. So any
  backstop must sit comfortably above ~152 N, which is why the CB3 force signal
  cannot be a *sensitive* contact detector and the old value was a coarse 300 N.
- **CB3 Force-limit floor = 100 N** (`loop_tags.py:55-57`: the safety Force limit
  "on CB3 cannot be set lower than 100 N and is the reason a light push does not
  stop the arm by itself"). The **live pendant value is not knowable from this
  repo** — the captain must read it on the pendant.
- **No wrist F/T sensor / force from joint currents** (`torque-report.html`,
  `loop_tags.py:46-52`); `zero_ftsensor` is e-Series only.

## The repository change: 300 N → 200 N

Tied to what the arm does in this cell:

- **Gripper:** Robotiq 2F-85 (~0.9–1.0 kg + coupling), recently swapped in for the
  old pneumatic gripper (`cumotion/ur10_gripper.xrdf`,
  `cumotion/robotiq_limits_proposed.yaml`).
- **Trays:** light cardboard/yogurt trays (`data/cell-clean-approach-v1`,
  `approach_demo.py`), well within the UR10's 10 kg payload.
- **Speeds:** motion runs at **5% velocity and acceleration scaling**
  (`SPEED = 0.05`), pendant slider ≤ 30% on top — low momentum, small dynamic
  forces.

200 N is **1.32×** the worst measured clean peak (151.8 N); the old 300 N was
~2×. Lowering to 200 N means the ROS backstop cancels the trajectory on gross
contact at ~200 N instead of ~300 N, while keeping ~48 N of headroom over the
clean envelope so it does not nuisance-trip. It is a **secondary net**, not
sensitive people-protection — that is the pendant limit below.

## What the captain must set on the pendant (mechanism #1 — cannot be done here)

The right value can only be dialled in empirically, with the captain present and
the e-stop in hand. **Start conservative (most protective) and raise only if it
nuisance-trips:**

1. **Set the correct payload first** (`Installation → General → Payload`): the
   Robotiq 2F-85 mass + coupling + the heaviest tray the arm carries. A payload
   left too high (e.g. an old value) makes the robot expect more torque and is a
   very likely reason the arm currently "pushes harder than you want before it
   trips" — fix this before touching the Force limit.
2. **Set the Force safety limit** (`Installation → Safety → Safety Limits →
   Force`, Normal mode) to the **CB3 floor of 100 N** to begin with. This is the
   earliest protective stop the CB3 allows. Because the cell runs at 5% scaling
   with light trays, 100 N should hold without nuisance trips.
3. **Run the normal slow cell motions** and watch for protective stops during
   clean work. If it trips with nothing in the way, raise the Force limit in small
   steps (e.g. 100 → 120 → 150 N) until clean cycles run quiet — no higher.
4. Record the value you settled on in `pendant_safety_force.current_value_n` in
   `cumotion/collision_force_limits.yaml` so the next session knows it.

(Values recorded in the YAML under `pendant_safety_force` are FOR THE RECORD only —
no code reads them.)

## Captain bring-up note

**What changes when this lands:** only `loop_tags.py`'s force backstop default
drops from 300 N to 200 N (from the new config file). Nothing else moves —
planning, execution paths, joint limits, maps, containers and the tracking-error
guards are all unchanged. Deploy is the usual copy to `~/ur10`; **this branch does
not deploy anything.**

**First run to watch:** the `home → tag0 → tag1` loop (`loop_tags.py`). Confirm a
full clean cycle completes without the message `external force deviation NNN N
exceeded 200.0 N`. Peak clean force deviation is printed each leg (`peak force dev
… N`) — it should stay well under 200 N (measured peaks were ≤ 151.8 N).

**Genuine protective stop vs. ROS-side nuisance trip — how to tell them apart:**
- A **genuine robot protective stop** (mechanism #1) is announced on the **teach
  pendant** (a safety popup), the robot leaves RUNNING/NORMAL, `robot_program_running`
  goes false, and every controller goes inactive. Recover per RUNBOOK §4/§5.
- A **ROS-side backstop trip** (mechanism #2) prints `external force deviation NNN
  N exceeded 200.0 N — contact detected, stopped` from `loop_tags.py`, cancels the
  current trajectory, and **the arm just holds position** — the robot stays
  RUNNING/NORMAL and the program stays up. No pendant popup.

**One-step revert (if 200 N starts tripping clean work):**
- For a single run, override on the command line: `--max-force-n 300`
  (or `--max-force-n 0` to disable the ROS backstop entirely).
- Permanently: edit the one line `ros_contact_force_backstop_n:` in
  `cumotion/collision_force_limits.yaml` back to `300.0`. That is the only place
  the value lives.
- To undo the pendant change, set the Force safety limit back to its previous
  value on the pendant (mechanism #1 is independent of this repo).

## Files

- `cumotion/collision_force_limits.yaml` — the single source of truth (value + docs).
- `cams/cellmap/tools/collision_force.py` — pure loader (no ROS).
- `cams/cellmap/tools/loop_tags.py` — `--max-force-n` default now sourced from the config.
- `cams/cellmap/tools/test_collision_force.py` — offline tests (value, sane range,
  no silent override, pendant documented).
