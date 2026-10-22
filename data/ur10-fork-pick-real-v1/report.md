# Real UR10 forklift tray pick-and-place — demo + recording (ur10-fork-pick-real-v1)

**Date:** 2026-10-22 · **Scope:** a safety-bounded, scripted forklift demo on the
**real** UR10 that picks the BRAVO Erdnussflips tray off the shelf with the fork
gripper and places it on the table on the other side of the shelf, recorded on all
three cameras + `/joint_states`, every contact motion gated by the captain.

This report is the task index. It records (1) the hardware-gate state and the
read-only robot reads, (2) the safety-bounded demo built from the Isaac plan, and
(3) the recording slots to be filled at the two captain-gated executions.

---

## 1 · Hardware gate (brief step 1) — current state: NOT YET CLEARED

The brief forbids any contact plan/motion until firstmate relays the captain's
confirmation that **(a)** the physical fork gripper is mounted, **(b)** the dock
sits under the real Erdnussflips tray, and **(c)** the fork TCP is set on the
robot. Status as of 2026-10-22 ~12:53Z:

| Gate item | State | Evidence |
|---|---|---|
| (a) Fork gripper mounted | **Likely YES (visual)** | Sibling scout `ur10-cams-check-v1`: wrist D435i (018322070619) colour still shows two fork tines on the flange, pointed at the shelf, BRAVO tray in view. Not a robot read. |
| (b) Dock under the tray | **UNCONFIRMED** | The tray sits in a white carrier in the wrist view; whether that carrier is the fork dock/pallet is not confirmed — needs captain eyes-on. |
| (c) Fork TCP set | **NO** | Read from the robot (below): TCP offset is **z = 0.1495 m only**, payload **1.000 kg** — this is *not* the fork TCP. |
| Captain confirmation relayed | **PENDING** | Firstmate is still asking the captain. |
| Arm motion-ready | **NO** | Dashboard `POWER_OFF / ROBOT_EMERGENCY_STOP` (then `CONFIRM_SAFETY`) — captain appears to be working on the cell. |

### Read-only robot reads (brief step 1: "read the current TCP, payload and safety limits")

Read with `fork_demo/read_tcp_payload.py` (passive: reads the controller's primary
broadcast on :30011 and an opt-in short RTDE output on :30004; sends no command):

```
robot_mode (primary): CONFIRM_SAFETY
TCP OFFSET (tool0 -> TCP): x=+0.00000 y=+0.00000 z=+0.14950  rx=ry=rz=0   (|t|=0.1495 m)
payload mass: 1.000 kg   cog: (0.0, 0.0, 0.1495)
```

- **The configured TCP is NOT the fork TCP.** The forklift kit needs
  `tcp_in_tool0 = (x 0.000, y −0.038, z 0.427)` and a payload of the gripper
  (0.82 kg) plus the carried tray/product (up to the 4.42 kg erdnussflips load).
  The current `z = 0.1495 m` / `1.0 kg` is a leftover placeholder tool — it must
  be replaced on the pendant before any Cartesian fork motion is meaningful.
- **Safety limits:** the safety-rated protective-stop **Force limit is teach-pendant
  only on this CB3** (`Installation → Safety → Safety Limits → Force`, N) and
  cannot be read or written from the PC/repo (`AGENTS.md`;
  `cumotion/collision_force_limits.yaml`). This task leaves the UR safety
  configuration untouched and does not read it from ROS. The one repo-settable
  contact threshold is the ROS-side gross-contact backstop
  `ros_contact_force_backstop_n = 200 N`, which the demo's force watchdog uses.

**Conclusion:** the gate is **not cleared** — at minimum the fork TCP is not set,
the dock is unconfirmed, and the captain's confirmation is pending (and the arm is
e-stopped). No contact plan or motion has been produced. The groundwork below is
all no-motion and is ready to run the moment firstmate relays the clearance.

---

## 2 · The safety-bounded demo (built now, no motion)

`fork_demo/real_fork_pick.py` picks the tray and places it on the far-side table,
re-anchored on the **real, measured** tray-front and table-drop poses (never the
twin's numbers). It takes the Isaac prototype only as the SHAPE of the motion:

> Reference: `tray_perception configs/forklift_demo/erdnussflips_pick.yaml`
> (sim mp4 `http://100.100.47.65:8787/v/mission/forklift-demo-002/walkthrough.mp4`).
> Relative offsets taken from it: standoff 0.12 m, insert 0.28 m, lift 0.035 m
> (tray rises ~23 mm off the dock posts), extract 0.40 m, withdraw 0.32 m.

Waypoints (base_link TCP = fork-tip centreline), red = contact, blue = transit —
see `fork_pick_path.png`:

```
HOME → PRE_APPROACH → INSERT_START → INSERTED → LIFTED → EXTRACTED
     → CARRY_OVER → SET_DOWN → LOWERED → WITHDRAWN → RETREAT
```

- **Contact legs** (INSERT_START→INSERTED, INSERTED→LIFTED, LIFTED→EXTRACTED,
  SET_DOWN→LOWERED→WITHDRAWN) are straight Cartesian moves at **5 %** scaling.
- **Transit legs** (HOME/approach, the carry across to the table, the final
  retreat) run at **10 %** and are planned collision-aware (cuMotion + the cleaned
  cell map) — not analytic straight lines — because the carry crosses the cell.

### Poses MUST be measured (brief step 2)

The two anchors are supplied in a measured poses yaml (`--poses`; schema +
provenance fields in `fork_demo/real_cell_poses.example.yaml`). **How each will be
obtained, with uncertainty, is recorded there and echoed in §3** at measurement
time — by the wrist camera (hand-eye on `/wrist/color_upright`, against the tray
front / mapped tags) or a manual freedrive teach at reduced speed. Until then the
script runs `--dry-run` on clearly-labelled PROVISIONAL placeholders and the
execute path **refuses** placeholder anchors.

### Safety thresholds (brief step 3) — visible in `SafetyLimits`

| Threshold | Value | Where enforced |
|---|---|---|
| Speed scaling, every leg | ≤ **10 %** | per-leg `speed_frac`; `execute_plan.py` caps 5 % |
| Speed scaling, contact legs | ≤ **5 %** | contact legs tagged 5 %; insert/extract |
| Acceleration scaling | ≤ **5 %** | `accel_frac`; cuMotion plan scaling |
| Force-deviation abort | **200 N** | `fork_demo/force_abort.py` watchdog, cancels the goal (from `collision_force_limits.yaml`; coarse CB3 net) |
| Pendant safety Force limit | untouched | teach-pendant only; the real safety-rated stop |
| Workspace box (shelf uprights / neighbours) | base_link AABB | every waypoint checked; execution refuses on breach |
| Halt-on-tracking-error | **0.15 rad** | `execute_plan.py --max-track-rad` |
| Per-joint step guard | 20 ° | between resolved IK waypoints |

The execute path is multiply gated (measured anchors, `--i-am-at-the-estop`, fork
TCP file, workspace box, interactive `EXECUTE`, RUNNING/NORMAL + External Control +
motion controller, LIVE `/joint_states`, seeded-IK inside `measured_joint_limits.yaml`)
and then **hands off** to `execute_plan.py --exact` with the force watchdog running
alongside — it never sends an unreviewed inline trajectory.

Offline tests: `fork_demo/tests/test_real_fork_pick.py` (10 tests — waypoint shape,
straight-insert/pure-lift geometry, speed caps, the 150 mm front-shift never
increasing reach, workspace box, provisional-anchor refusal, force monitor +
stale fail-toward-stop).

---

## 3 · Captain-gated executions (brief step 4) — SLOTS, not yet run

Each runs only after firstmate relays the explicit go with the captain at the
e-stop. Record with `fork_demo/record_fork_demo.sh <run>` (all three cameras +
`/joint_states` + TCP + speed scaling into `data/ur10-fork-pick-real-v1/<run>/`;
the TABLE D415 is the side-by-side angle vs the Isaac mp4).

| Run | Command | Status | Artifacts |
|---|---|---|---|
| **1. Front dry run** (150 mm in front, no contact) | `real_fork_pick.py --front-offset 0.15 --poses <measured> --i-am-at-the-estop --tcp <fork_tcp>` | **not run** (gate) | `front-dry-run-001/` |
| **2. Real pick-and-place** | `real_fork_pick.py --front-offset 0.0 --poses <measured> --i-am-at-the-estop --tcp <fork_tcp>` | **not run** (gate) | `real-pick-001/` |

Both executions are blocked on §1. The side-by-side video itself is a later task
(out of scope); this dir holds the real-run recordings for it.

---

## Files

- `fork_demo/real_fork_pick.py` — the demo (dry-run default; gated front-dry-run + real pick).
- `fork_demo/force_abort.py` — independent force-deviation abort watchdog (+ offline tests).
- `fork_demo/read_tcp_payload.py` — read-only TCP offset + payload reader (§1 reads).
- `fork_demo/record_fork_demo.sh` — three-camera + telemetry recorder for a run.
- `fork_demo/real_cell_poses.example.yaml` — measured-anchor schema + provenance.
- `fork_demo/tests/test_real_fork_pick.py` — offline geometry + safety tests.
- `fork_pick_path.png` — dry-run path plot (provisional anchors).
