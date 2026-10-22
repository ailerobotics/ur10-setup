# fork_demo — scripted forklift tray-pick demo for the real UR10 (plan-only)

Prepared under task **ur10-fork-demo-readiness-v1** (2026-10-22). This is the
minimal, conservative, scripted (non-perception) demo for picking a supermarket
tray with the passive **forklift gripper S** (two forks 120 mm apart, load rides
on friction). It is **safe by default**: it plans and plots, and only executes
behind a stack of safety gates.

See `../data/ur10-fork-demo-readiness-v1/report.md` for the full rig-readiness
verdict and why the demo is **NOT recordable yet** (as of 2026-10-22: External
Control STOPPED, motion controllers inactive, `/joint_states` frozen, wrist camera
not streaming, and no physical fork gripper / pallet / TCP).

## Real pick-and-place (task ur10-fork-pick-real-v1)

`real_fork_pick.py` is the **real-cell** forklift demo: pick the BRAVO
Erdnussflips tray off the shelf and place it on the table on the other side,
re-anchored on the **real, measured** tray-front + table-drop poses (never the
twin's numbers), with the Isaac `erdnussflips_pick.yaml` used only as the SHAPE.
Safe by default (`--dry-run`); the gated execute path needs measured `--poses`,
`--i-am-at-the-estop`, a fork TCP file, the workspace box, and RUNNING/NORMAL +
live feed, then hands off to `execute_plan.py --exact` with `force_abort.py`
watching. All safety thresholds live in `SafetyLimits` (speed ≤10 %/leg, ≤5 %
contact; accel ≤5 %; 200 N force backstop; workspace box; tracking-error halt).

```bash
python3 fork_demo/real_fork_pick.py                       # dry-run (provisional anchors)
python3 fork_demo/read_tcp_payload.py [--rtde]            # read the robot's configured TCP + payload
python3 -I fork_demo/tests/test_real_fork_pick.py         # offline tests
fork_demo/record_fork_demo.sh front-dry-run-001           # record 3 cams + telemetry for a run
```

Measured-pose schema + provenance: `real_cell_poses.example.yaml`. Full status,
robot reads and recording slots: `../data/ur10-fork-pick-real-v1/report.md`.

## Files

- `fork_pick_demo.py` — the demo. **`--dry-run` is the default** and touches no
  ROS: it prints the base_link TCP waypoints for the full cycle
  (HOME → APPROACH → INSERT → LIFT → EXTRACT → CARRY → PLACE) and saves a 3D path
  plot. Fork geometry is read from the kit's `forklift_config.json`
  (`~/datasets/shelf-twin/grippers/forklift_S/…zip`); conservative fallbacks are
  baked in so the dry run works without the kit.
- `check_joint_feed_live.py` — a reusable jitter gate that distinguishes a LIVE
  encoder feed from the frozen republished RTDE packet (RUNBOOK §10). CLI: exit 0
  = live, exit 1 = frozen.

## Usage

```bash
# dry run (default): prints the plan + saves data/ur10-fork-demo-readiness-v1/fork_demo_path.png
python3 fork_demo/fork_pick_demo.py

# override the provisional cell layout (base_link metres)
python3 fork_demo/fork_pick_demo.py --shelf-front-x 0.98 --pick-slot-z 0.31 --place-y 0.55

# is the joint feed actually live?  (read-only)
source /opt/ros/jazzy/setup.bash
python3 fork_demo/check_joint_feed_live.py
```

## Execute path (refuses unless every gate passes)

```bash
python3 fork_demo/fork_pick_demo.py --i-am-at-the-estop --tcp <fork_tcp.yaml>
```

Gates, in order (any failure aborts before motion):

1. `--i-am-at-the-estop` flag present (asserts a person is physically at the e-stop);
2. a real fork **TCP file** supplied with `--tcp` (none exists yet);
3. interactive confirmation — type `EXECUTE`;
4. robot **RUNNING / NORMAL** (dashboard);
5. External Control **running** AND a **motion controller active**;
6. `/joint_states` verified **LIVE** (jitter gate; rejects today's frozen feed);
7. every waypoint's **seeded-IK** solution inside the **measured joint windows**
   (`cumotion/measured_joint_limits.yaml`).

After the gates it prints the validated, joint-limited plan and **stops** — the
actual motion dispatch is a deliberate, separate hand-off to the established
executor (`execute_plan.py --exact --limits cumotion/measured_joint_limits.yaml`)
under the pendant speed slider, so the trajectory gets a human review before it is
ever sent. This script was **never executed** in the readiness task.

## Before this can be a real demo

Physical prerequisites (not in place — report.md §4): a **mounted physical fork
gripper**, a **pallet/dock** under the tray, a measured **fork TCP + payload**
(gripper 0.82 kg + tray ≤ 2.45 kg), a **free-space test** against the cleaned cell
map, a **person at the e-stop**, and an **overview camera** for the recording. The
waypoint positions here are **provisional** and must be confirmed against the real
shelf and the Bravo Erdnussflips tray.
