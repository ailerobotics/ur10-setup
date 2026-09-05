# Captain execute note — AprilTag 1 → shelf approach, WITHIN measured+declared joint limits

> ## ✅ EXECUTABLE — re-validated inside the measured joint limits (2026-09-05)
>
> This note delivers the **executable** plan and the exact captain command for
> the tag 1 → shelf standoff demo. The plan is
> **`cams/cellmap/plans/tag1-shelf-measlim.yaml`**, re-planned by cuMotion and
> validated so that **every waypoint stays inside the measured joint windows**
> (`cumotion/measured_joint_limits.yaml`) — 5 hand-swept joints + 3
> captain-declared extensions (disclosed below). It supersedes, for execution,
> the collision-evidence-only plan `tag1-shelf-cumotion.yaml`, which drove
> wrist_1 to 355.1° and MUST NOT be run.
>
> **Physical execution is CAPTAIN-GATED.** No agent has moved the arm. Confirm
> the prerequisites and the safety checklist below, then run the single command
> yourself with the e-stop in hand.

**Status: DRY-RUN VALIDATED, plan-only. Nothing has moved.** An agent prepared
and verified the plan and its guards without ever commanding arm motion.

## What the plan is

A collision-aware cuMotion plan from a hover **above AprilTag 1** to a **standoff
in front of the gondola shelf**, against the cleaned map
`cell-2026-09-04-cleaned.nvblx`, tool orientation held as parked (no reorient, no
grasp — the gripper is not active). It stops **short of contact**.

| | base_link (m) | |
|---|---|---|
| start (hover above tag 1) | tool0 (−0.506, −1.058, 0.25) | tag 1 centre is (−0.506, −1.058, −0.057) |
| goal (shelf standoff) | tool0 (−0.10, 0.80, 0.95) | 0.33 m short of the nearest carton (y≈1.13) |

Validated numbers (`cams/cellmap/plans/tag1-shelf-measlim.yaml`): cuMotion
`isaac_ros_cumotion`, **276 points, 27.4 s at 0.05 scaling**, tool path 2.715 m
(1.34× straight line, 74.7 cm max deviation), max joint delta **161.9° (< 170°
guard)**. Clearances swept over ALL collision spheres + 0.04 m buffer on the FULL
collision model: **+13.7 cm to the shelf/carton volume, +4.8 cm to the table**
(`plan-measlim-clearance-checks.md`, render `plan-measlim-vs-cumotion.png`).
Every waypoint passes the joint-window guard against
`cumotion/measured_joint_limits.yaml` (tightest margin 1.0° = the declared slack);
`execute_plan.py --limits` dry-check passes.

`start_joints`: `[1.2971, −3.0399, −0.2456, 5.9301, −1.1607, −0.5524]` rad.

## Joint-limit provenance — plain-English disclosure

The limits file `cumotion/measured_joint_limits.yaml` has **5 hand-swept
(measured)** joints and **3 captain-declared** extensions. A *declared* window is
NOT a measurement: the captain widened it by decision to cover the travel this
validated demo needs, taking responsibility for it; the raw hand-sweep extremes
stay on record in the file (`measured:`), and each override is recorded under
`declared:` (source = captain-declared, NOT hand-swept, 2026-09-04/05).

| joint | hand-swept raw (deg) | window applied (deg) | declared? | why |
|---|---|---|---|---|
| shoulder_pan | −65.3 … 90.9 | −60.3 … 85.9 | measured | 5° margin |
| shoulder_lift | −173.8 … −97.2 | **−175.2 … −11.2** | **declared (both ends)** | sweep never reached the demo swing (−174.2 … −12.2); +1° slack |
| elbow | −52.8 … 114.2 | **−62.7** … 109.2 | **declared (floor)** | demo floor −61.7 below the sweep floor −52.8; ceiling kept measured; +1° slack |
| wrist_1 | 157.8 … 353.2 | 162.8 … 348.2 | measured (untouched) | the in-limits plan keeps wrist_1 ≤ 339.77°, inside the measured window — NOT extended |
| wrist_2 | −81.0 … 120.4 | **−83.4** … 115.4 | **declared (floor)** | demo floor −82.4 below the sweep floor −81.0; ceiling kept measured; +1° slack |
| wrist_3 | −88.8 … 169.7 | −83.8 … 164.7 | measured | 5° margin |

Note: the earlier evidence plan needed wrist_1 to **355.1°, 1.9° PAST the
hand-swept physical bind (353.2°)**. The captain accepted extending wrist_1 only
if still required — it was **not required**: this re-plan routes wrist_1 entirely
inside its measured window, so **no joint here is declared past a measured
physical hard-stop.**

## Prerequisites

1. **Cleaned-map planner up, on the FULL collision model.** For `--exact`
   execution the planner is not re-queried (saved waypoints are sent with
   reconstructed timing), so this matters for RE-VALIDATION / re-planning, not for
   the `--exact` run itself. When you do re-plan, start it from a LIVE copy
   (`~/ur10` after merge) or a task worktree — never a returned scratch worktree —
   on the measured limited model with FULL world collision:
   ```bash
   systemctl --user stop ur10-cumotion; docker stop ur10-cumotion   # the unit stop alone can orphan the container
   ~/ur10/cumotion/start_cumotion_map.sh cell-2026-09-04-cleaned.nvblx \
       urdf:=/cfg/ur10_measured.urdf xrdf:=/cfg/ur10_measured.xrdf
   # wait for: "Robot description loaded successfully" + "MotionPlan action server initialized"
   ```
   **Never start it on `ur10_pneumatic.urdf` / `.xrdf`** (those drop shoulder_link
   + upper_arm_link from world collision). `ur10_measured.xrdf` keeps the full
   set (`--keep-world-collision`); confirm `collision.geometry ==
   self_collision.geometry == ur10e_collision_spheres`.
2. **Live, jittering `/joint_states`.** Verify the feed is LIVE by encoder jitter
   (~4–9 mdeg at rest), never by rate — a dropped RTDE link republishes the last
   packet at a perfect 125 Hz. If it reads 0.000 mdeg (frozen), recover the
   driver before anything else (`systemctl --user restart ur10-driver`, one writer
   only — RUNBOOK §6b — then re-check jitter). `execute_plan.py` will also abort
   if the live start is off, but do not rely on a frozen feed.
3. **The arm must be AT the validated start.** `execute_plan.py` ABORTS if the
   live pose is > 0.57° off `start_joints`. Jog there on the pendant first.
4. Robot RUNNING/NORMAL, external-control program up (RUNBOOK §4), **one writer
   only** (RUNBOOK §6b).
5. **Restart the wrist-cam tray detector** so it runs during the move (captain
   request). Inside the `shelfbot_ros` container:
   ```bash
   docker exec shelfbot_ros bash -lc 'source /opt/ros/jazzy/setup.bash && \
     /opt/yolo_venv/bin/python3 /root/tray_wrist_probe.py --secs 0 \
     --model /root/weights/tray_v5_best.pt --classes tray,broken_tray \
     --base-tf=-0.776771,-0.154955,0.824915,-0.682359,0.197102,-0.105725,0.695960 \
     --json /root/tray_probe.json --annot /root/tray_annot.png'
   ```
6. **The executed yaml must be the committed, validated one.**
   `git status --short cams/cellmap/plans/tag1-shelf-measlim.yaml` must print
   nothing. `plan_test.py` writes `plans/<label>.yaml` unconditionally, so never
   re-run under this label before executing.

## Safety checklist — do not skip a line (from cams/approach/README.md)

- [ ] E-stop in hand, thumb on it, for the whole motion.
- [ ] Pendant speed slider ≤ 30 % (independent cap on top of the 0.05 code cap).
- [ ] Workspace between the arm and the gondola visibly clear of people and objects.
- [ ] Wrist camera USB cable has ≥ 0.7 m slack routed along the arm.
- [ ] The camera is NOT in the arm's collision model — eyeball the arm path for
      the camera body's clearance too.
- [ ] Nobody else is commanding the arm (one writer — RUNBOOK §6b).
- [ ] `/joint_states` is LIVE by jitter, not frozen (Prerequisite 2).
- [ ] The plan passed the joint-window guard against
      `cumotion/measured_joint_limits.yaml` and both clearance sweeps.
- [ ] The executed yaml is the committed, unmodified one (Prerequisite 6).

## Execution — the single command

Run from the repo root (`~/ur10` after this merges):

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/execute_plan.py \
    cams/cellmap/plans/tag1-shelf-measlim.yaml \
    --exact --limits cumotion/measured_joint_limits.yaml
```

`--exact` executes the committed validated waypoints with reconstructed uniform
timing (peak velocity 5.0 % of the joint limits — verified). `execute_plan.py`
keeps its own guards, all of which abort before any motion: live start must match
the plan's start, the robot must be RUNNING/NORMAL with its program up, and
**every waypoint must lie inside `--limits`** (mandatory, no bypass). During
motion a tracking excursion > 0.15 rad (8.6°) cancels the goal. There is **no**
runnable command for any other plan in this branch.

## Notes

- This demonstrates collision-aware routing (tag 1 → a safe standoff). The
  per-carton pick approach is the camera-driven `cams/approach/approach_demo.py`
  model (typed-`yes` gate per leg).
- Re-measure or re-declare the limits whenever the line is re-routed or the arm
  is re-parked — see RUNBOOK §10 "Measuring / declaring real joint limits".
- Rollback: delete/rename `cell-2026-09-04-cleaned.nvblx` and the loaders fall
  back to the 2026-09-03 scan.
