# Captain execute note — AprilTag 1 → shelf approach, cleaned map (2026-09-04)

**Status: DRY-RUN VALIDATED, plan-only. Nothing has moved. Physical execution is
yours, at the pendant.** This note hands you the ready-to-run command and the
safety checklist. An agent prepared and verified it without ever commanding arm
motion.

## What the plan is

A collision-aware cuMotion plan from a hover **above AprilTag 1** to a **standoff
in front of the gondola shelf** where the 4 cartons are, against the cleaned map
`cell-2026-09-04-cleaned.nvblx`. Tool orientation held as parked (no reorient, no
grasp — the gripper is not active). It stops **short of contact**.

| | base_link (m) | |
|---|---|---|
| start (hover above tag 1) | tool0 (−0.506, −1.058, 0.25) | tag 1 centre is (−0.506, −1.058, −0.057) |
| goal (shelf standoff) | tool0 (−0.10, 0.80, 0.95) | 0.33 m short of the nearest carton (y≈1.13) |

Validated numbers (`cams/cellmap/plans/tag1-shelf-cumotion.yaml`): cuMotion
`isaac_ros_cumotion`, 313 points, **31.2 s at 0.05 scaling**, tool path 2.575 m
(1.27× straight line, 58 cm max deviation — it arcs up over the table and curves
through the open cell), max joint delta 161.9° (< 170° guard). Clearances swept
over ALL collision spheres + 0.04 m buffer: **+13.7 cm to the shelf/carton volume,
+4.8 cm to the table.** Cross-checked against live wrist-cam detections: the map's
shelf front (y≈1.13) matches where the cartons actually are (depth-measured
y≈1.13–1.32) — the plan approaches a real shelf, not a map artefact.

## Prerequisites

1. Cleaned-map planner up (loads the newest `cell-*.nvblx` = the cleaned map):
   ```bash
   systemctl --user stop ur10-cumotion            # if the default planner is running
   ~/ur10/cumotion/start_cumotion_map.sh cell-2026-09-04-cleaned.nvblx
   # wait for: "cell map loaded … starting cuMotion (read_esdf_world=true)"
   ```
2. **The arm must be AT the validated start** (hover above tag 1). `execute_plan.py`
   ABORTS if the live pose is > 0.57° off the plan's start. Bring the arm there
   first (jog on the pendant, or a separate validated leg to the start_joints
   `[1.2971, −3.0399, −0.2456, 5.9301, −1.1607, −0.5524]` rad). Do NOT skip this —
   the plan is only collision-checked from that start.
3. Robot RUNNING/NORMAL, external-control program up (RUNBOOK §4), one writer only
   (RUNBOOK §6b).

## Safety checklist — do not skip a line (from cams/approach/README.md)

- [ ] E-stop in hand, thumb on it, for the whole motion.
- [ ] Pendant speed slider ≤ 30 % (independent cap on top of the 0.05 code cap).
- [ ] Workspace between the arm and the gondola visibly clear of people and objects.
- [ ] Wrist camera USB cable has ≥ 0.7 m slack routed along the arm.
- [ ] The camera is NOT in the arm's collision model — eyeball the arm path for
      the camera body's clearance too.
- [ ] Nobody else is commanding the arm.

## Run it (dry-run FIRST, then execute)

```bash
source /opt/ros/jazzy/setup.bash

# 1 — DRY RUN: re-validate the plan against the live arm + cleaned map, no motion
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 \
  ~/ur10/cams/cellmap/tools/plan_test.py --label tag1-shelf-cumotion \
  --pipeline isaac_ros_cumotion --start-pose=-0.506,-1.058,0.25 \
  --goal-pose=-0.10,0.80,0.95 --pose-goal
#   expect: "OK: 313 points … 1.27x straight line" and clearances as above.

# 2 — arm the stack (RUNBOOK §3/§6): program running, slider nonzero
ros2 service call /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction "{speed_slider_fraction: 0.3}"

# 3 — EXECUTE the VALIDATED waypoints at 0.05 scaling (guards abort before motion:
#     live start must match, program running, waypoints inside pneumatic windows,
#     tracking-error cancel during motion). --exact runs the approved path itself.
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 \
  ~/ur10/cams/cellmap/tools/execute_plan.py \
  ~/ur10/cams/cellmap/plans/tag1-shelf-cumotion.yaml \
  --limits ~/ur10/cumotion/pneumatic_limits.yaml --exact
```

`execute_plan.py` refuses to move on any guard failure (start mismatch, path
drift, window violation, program not running) and cancels the goal on a tracking
excursion > 0.15 rad. Abort at any time with the e-stop or Ctrl-C (it only ever
sends the one already-validated trajectory).

## Notes

- This demonstrates the collision-aware routing (tag 1 → a safe standoff). The
  actual per-carton pick approach is the camera-driven `cams/approach/approach_demo.py`
  model (typed-`yes` gate per leg); full per-carton YOLO confirmation lands during
  that approach as the wrist reaches a shelf-facing pose (the eye-level parked view
  boxes shelf boards, not cartons — the recorded viewpoint limitation).
- Rollback: delete/rename `cell-2026-09-04-cleaned.nvblx` and the loaders fall back
  to the 2026-09-03 scan.
