# cams/approach — eye-in-hand cardboard approach demo (v1)

The wrist-mounted D435i finds a cardboard target on the gondola shelf, the
arm plans a pure-translation approach that stops well short of it, holds,
and returns exactly to where it started. One motion per leg, a typed
confirmation before each, plan-only by default.

Built and bench-verified 2026-08-18 **without ever moving the arm**: all
evidence below comes from live detection, live TF, and live MoveIt
*plan-only* requests. The first `--execute` run belongs to the captain.

## Pieces

| File | Runs where | Does |
|---|---|---|
| `wrist_rig.launch.py` | host, domain 0 | D435i streams + IMU + `tool0->camera_link` mount TF |
| `target_publisher.py` | `shelfbot_ros` container | trained YOLO -> median-depth back-projection -> `/approach/target_camera` |
| `approach_demo.py` | host | DETECT → COMPUTE → CONFIRM → APPROACH → HOLD → CONFIRM → RETREAT |
| `tools/imu_gravity_check.py` | host | rotation sanity check of the mount TF against gravity |
| `bench-annotated.png` | — | live detection frame from the bench run |

## Design decisions (the "why")

**Pure translation, no reorienting toward the box.** v1 keeps the tool
orientation exactly as parked and only translates. Reorienting would (a) swing
the strapped-on camera and its hand-routed USB cable through unpredictable
arcs, (b) invalidate the rough extrinsics mid-move (the target was measured
through this very transform), and (c) buy nothing — the demo's point is "the
arm went where the camera said", which translation alone proves. The tool
translates so the **camera** ends `--standoff` short of the target along the
camera→target ray; since orientation is held, camera and tool move by the
same vector.

**Standoff 0.30 m default, hard floor 0.25 m, larger is always allowed.**
The extrinsics are a documented guess (below): translation is uncertain at
the ±5 cm level and the target itself is a bbox-median estimate. 0.25 m is
an order of magnitude above the combined error, and also keeps the camera
(which the arm's collision model does NOT contain) clear of everything.
`--standoff` below the floor is refused, never clamped. Note the UR10
envelope: for the current park + shelf geometry, goals get *unreachable*
below ~0.85 m standoff (see "reach" below) — deep targets are approached by
*raising* the standoff, which is the safe direction.

**Speed/accel scaling hard-capped at 0.05 in code.** First contact between
a camera-derived pose and real motion; 5 % of rated speed keeps any surprise
slow enough to e-stop comfortably. `--speed` can only lower it. The pendant
slider multiplies on top (the scaled controller respects it), so the captain
holds a second, independent brake.

**Plan → printed summary → typed `yes` → execute.** Planning is decoupled
from execution end to end (MoveIt `plan_only`, then the repo's proven
`scaled_joint_trajectory_controller` FollowJointTrajectory path from
`motion_test.py`). The full plan — target, approach point, travel distance,
per-joint deltas, duration — is printed *before* anything can move, so the
human confirms what the arm will actually do, not what they hope it does.
Anything but the literal word `yes` aborts. `--dry-run` is the default;
real motion requires an explicit `--execute`.

**Seeded IK + joint-space goal instead of a pose goal.** Bench finding:
pose-goal planning let MoveIt's goal sampler pick a *distant* IK
configuration — OMPL returned a formally valid plan with a +405°
shoulder_lift rewind for a 0.59 m translation; Pilz LIN failed outright on
per-sample KDL IK. The demo therefore calls `/compute_ik` seeded with the
current joints, REFUSES any solution whose largest joint delta exceeds 170°,
and plans to the joint goal with Pilz **PTP** (deterministic joint-space
interpolation; default pipeline). The same 170° guard re-checks every final
trajectory regardless of planner.

**Workspace whitelist (REFUSE, never clamp), base_link frame:**

```
TARGET_BOX    x: -0.80 .. 1.00   y: 1.40 .. 2.40   z: 0.55 .. 1.70
APPROACH_BOX  x: -0.60 .. 0.80   y: -0.10 .. 1.35  z: 0.80 .. 1.55
```

Derivation (measured live 2026-08-18, parked shelf-facing pose): tool0 at
`(0.065, 0.248, 1.156)`, camera looking along base_link +Y; the nearest
cardboard front face (blue yogurt tray, lower board) measured at
`(-0.022, 1.516, 0.855)`. TARGET_BOX is the shelf volume (front faces from
y=1.40 back through the rack). APPROACH_BOX stops the tool/camera ≥ 0.15 m
short of that nearest measured face (y ≤ 1.35), stays above everything below
the lower pick surface (z ≥ 0.80, board top ~0.83) and under the top-shelf
overhang (z ≤ 1.55). The y-gap between the boxes (1.35 / 1.40) means no pose
can ever satisfy both. Both the target and BOTH endpoints of the approach
(tool0 goal, camera destination) must pass, else the run is refused.
Re-derive these numbers if the gondola or the arm pedestal moves.

**Abort conditions** (safe stop, nothing commanded): dynamic TF older than
1 s; `/joint_states` stale; target spread > 3 cm; target moved > 5 cm
between plan and confirm; IK failure or rewound configuration; planning
failure; empty trajectory; goal rejected by the controller (RUNBOOK §4 —
let `ur10-rearm` fix it); no execution feedback for 3 s; execution overrun.

## The camera mount transform (rough by design)

Guess published by `wrist_rig.launch.py` as `tool0 -> camera_link` (the
RealSense's own calibrated TF tree continues to the optical frames):

- **Rotation**: camera forward = tool0 +Z (out of the flange); camera up
  initially guessed as tool0 +X, then **refined against the IMU gravity
  vector**: with the arm parked, measured "up" in the IMU frame sat within
  0.3° of image-up while the prediction from the raw guess was 28.6° off —
  exactly the angle tool0 +X leans off vertical in the park pose. The camera
  is strapped level with the *world*, i.e. rolled **+28.6°** about its
  optical axis w.r.t. tool0 (now the `cam_roll_deg` default).
  **Residual after refinement: 4.2°** (`tools/imu_gravity_check.py`,
  2026-08-18, 911 samples, |g| = 8.94 m/s² — the D435i accel is
  factory-uncalibrated, hence not exactly 9.81).
- **Translation**: `(0.05, 0.00, 0.015)` m in tool0 — strap offset toward
  tool0 +X plus lens standing slightly proud of the flange. A pure guess
  (gravity carries no lever-arm information), uncertain at ±5 cm; the
  standoff floor absorbs it.

## Live detector findings (2026-08-18)

The trained weights (`tray_best_vm.pt`, imgsz 960, conf 0.25) were built on
near-overhead tray views. From the eye-level wrist viewpoint they **never
produce `tray`/`Grabable`** — even at conf 0.05 — but they DO box the blue
cardboard yogurt tray as **`Obstacle`** (0.39–0.63). All other `Obstacle`
hits were shelf structure whose bboxes are truncated at the image border.
Workable filter (implemented, auditable via the class-count log line):
`tray`/`broken_tray` accepted unconditionally; `Obstacle` accepted only with
a fully-interior bbox covering < 40 % of the image. `--strict-classes`
disables the fallback. The big cardboard box on the top shelf is cut off at
the frame top → filtered out (correctly); the white milk display box never
fires at all.

Target validity (a moving arm must never yield a target): 5 consecutive
detections ≤ 2 s apart with 3-D spread < 3 cm, **and** every gyro sample in
the last 1 s below **0.03 rad/s** (~1.7 °/s; D435i gyro noise at rest is
~0.01 rad/s, hand-jogging is >0.1), **and** IMU data fresh (< 0.5 s). No
IMU ⇒ no targets, by design (`--no-imu` is bench-only and prints a banner).

## Captain's run procedure (in order)

Safety checklist first — do not skip a line:

- [ ] E-stop in hand, thumb on it, for both legs.
- [ ] Pendant speed slider ≤ 30 % (independent cap on top of the 0.05 code cap).
- [ ] Workspace between arm and gondola visibly clear of people and objects.
- [ ] Camera USB cable has ≥ 0.7 m of slack routed along the arm — the
      approach translates the wrist up to ~0.6 m toward the shelf.
- [ ] The camera is NOT in the arm's collision model: eyeball the straight
      line tool0 → goal for clearance, including the camera's own body.
- [ ] Nobody else is commanding the arm (RUNBOOK §6b: one writer at a time).

Then, four terminals (or backgrounded), in this order:

```bash
# 1 — wrist camera + IMU + mount TF (host, domain 0)
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 launch ~/ur10/cams/approach/wrist_rig.launch.py

# 2 — sanity-check the mount rotation (arm parked; expect ~4-5 deg)
ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  python3 ~/ur10/cams/approach/tools/imu_gravity_check.py

# 3 — detector bridge (container; weights + venv live there)
docker start shelfbot_ros
docker cp ~/ur10/cams/approach/target_publisher.py shelfbot_ros:/root/
docker exec -e ROS_DOMAIN_ID=0 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 shelfbot_ros \
  bash -c "source /opt/ros/jazzy/setup.bash && source /ros_ws/install_container/setup.bash \
           && /opt/yolo_venv/bin/python3 /root/target_publisher.py"
# watch for 'VALID=True' log lines; /detect_new/annotated shows the live view

# 4 — dry-run FIRST, every session, and read the plan summary
source /opt/ros/jazzy/setup.bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 ~/ur10/cams/approach/approach_demo.py \
  --standoff 0.90        # 0.90 verified reachable for the current shelf scene

# 5 — arm the stack (RUNBOOK §3/§6): program running, slider nonzero
ros2 service call /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction "{speed_slider_fraction: 0.3}"

# 6 — the real thing: same command + --execute, type `yes` at each gate
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 ~/ur10/cams/approach/approach_demo.py \
  --standoff 0.90 --execute
```

Abort at any gate by typing anything other than `yes` (or Ctrl-C — the demo
only ever sends the already-confirmed single trajectory, nothing else).

## Bench evidence (2026-08-18, arm never moved)

- Stable target, real shelf scene, base_link frame:
  `(-0.022, +1.516, +0.855)` — blue yogurt tray front face; window spread
  0.1–0.4 cm across repeated runs.
- Full `--dry-run --standoff 0.90` against live MoveIt: both legs planned
  clean — IK max joint delta 66.3°, Pilz PTP, 116 points / 11.5 s per leg
  at 0.05 scaling, retreat mirrors approach exactly (full summaries in the
  PR).
- `bench-annotated.png`: live annotated detection frame.
- Reach finding: with the tray 1.29 m from the camera, standoffs < ~0.85 m
  put tool0 beyond the UR10's envelope (first symptom was OMPL's "unable to
  sample goal"; now caught early by the reach check with a clear message).

## Known limitations (v1)

- **Extrinsics are a documented guess** (±5 cm translation, ~4° rotation
  residual). Fine at ≥ 0.25 m standoff; NOT fine for grasping. Hand-eye
  calibration replaces this.
- **The camera is not in the arm's collision model.** MoveIt would happily
  plan the camera body through an obstacle. Poses must keep generous
  clearance; approach only through visibly free space.
- **The planning scene is empty** (RUNBOOK 8b): MoveIt knows self-collisions
  only — the whitelist boxes and the human eye are the shelf-collision
  guard. (cuMotion knows just the ground plane.)
- **Detector is viewpoint-sensitive**: cardboard reads as `Obstacle` from
  eye level and confidence hovers near 0.25, so valid targets can take tens
  of seconds to accumulate. Retraining with wrist-viewpoint images fixes
  this properly.
- **`world->PerceptionCamera` must not exist in this mode** — it is the
  static table-rig TF and fights the wrist TF chain. `wrist_rig.launch.py`
  only publishes it behind `lookat_tf:=true`; check for orphaned
  `rig_tf_main` processes from older launches (`pgrep -af rig_tf_main`).

## IMU: used now / future work

Used in v1 (deliberately nothing more): gravity-vector rotation sanity check
(above) and gyro-stillness gating of target validity.

Future ideas, explicitly NOT built: accel-based bump/collision detection
during approach (e-stop trigger); vibration signature to detect a loose
strap; gyro-integrated orientation drift canary for the mount guess between
sessions; gravity-check automation at every park to catch a bumped camera;
complementary filter to timestamp-align detections with arm motion instead
of requiring full stillness.
