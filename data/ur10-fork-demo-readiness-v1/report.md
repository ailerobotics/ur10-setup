# Real UR10 + fork-gripper demo readiness (ur10-fork-demo-readiness-v1)

**Date:** 2026-10-22 · **Scope:** read-only health + readiness check of the physical
UR10 cell on `aileworkstation`, to answer whether a scripted (non-perception) forklift
tray-pick demo can be recorded on the **real** robot and shelf **now**.

**No arm motion, mode change, brake release, or program/motion command was issued.**
Every check below is read-only (dashboard queries, ROS topic/service reads, socket
inspection, camera topic subscription). The `ur10-*` services, Blender and whispr were
not stopped or restarted, and `/tmp/aile-gpu.lock` was not taken.

---

## VERDICT: NOT READY to record a real fork demo now

The robot is **powered and safe** (RUNNING / NORMAL, no e-stop, no protective stop) and
MoveIt/cuMotion are up, but the cell is **not in a motion-ready or demo-recordable state**,
for three independent reasons, each of which is a hard blocker on its own:

1. **No motion authority.** The External Control program is **STOPPED / "No program
   loaded"** and **every motion controller is inactive** — the arm cannot be commanded to
   move (classic RUNBOOK §4 "External Control script dead" state). Recovery is the
   `resend_robot_program` / controller re-activation in RUNBOOK §4 — *not done here*
   (out of this task's read-only scope).
2. **Telemetry is frozen, not live.** `/joint_states` streams a perfect 125 Hz but is a
   **stale republished packet** (byte-identical over 245 samples / 4 s, constant nonzero
   velocity that never integrates), with a **23.8 MB unread backlog** on the robot's
   `:30002` socket — the driver's receive side has stalled. The displayed joint pose
   **cannot be trusted** until the driver is restarted (RUNBOOK §10 "two things that will
   fool you"). Fix = `systemctl --user restart ur10-driver` — *not done here* (task
   forbids restarting `ur10-*`).
3. **No camera view of the rig.** The wrist D435i is logging repeated *"Incomplete video
   frame / Frame Corrupted"* USB faults and is **not delivering any color/depth frames**
   right now, so **no still could be captured** to confirm what is on the flange or
   whether the shelf/tray are in the workspace. No separate fixed/rig camera is running at
   all (only `/wrist/*` topics exist; both fixed D415s are presence-only/uncalibrated per
   the project cell notes).

Beyond the live-state blockers, the fork demo also has **physical/setup prerequisites that
are not in place** (no evidence of a mounted fork gripper, no pallet/dock, no fork TCP) —
see §4. The forklift kit at `~/datasets/shelf-twin/grippers/forklift_S/` is an **Isaac Sim
package** (USD assets + a sim driver), not a physical mount.

---

## 1 · Is the robot healthy and reachable? (each check + evidence)

| Check | Result | Evidence |
|---|---|---|
| Network link | **OK** | `ping -I enp5s0 192.168.0.1` → 3/3, 0% loss, ~0.15 ms |
| Dashboard reachable | **OK** | `nc 192.168.0.1 29999` responds |
| Robot mode | **RUNNING** (powered, brakes released) | dashboard `robotmode` → `RUNNING` |
| Safety mode | **NORMAL** (no e-stop, no protective stop) | dashboard `safetystatus` → `NORMAL` |
| Program state | **STOPPED / no program loaded** ❌ | dashboard `programState` → `STOPPED null`; `get loaded program` → `No program loaded` |
| External Control running | **false** ❌ | `/io_and_status_controller/robot_program_running` → `data: false` |
| ROS driver service | active | `systemctl --user is-active ur10-driver` → active |
| MoveIt | **up** | `ur10-moveit` active; nodes `/move_group`, `/move_group/moveit`; actions `/move_action`, `/sequence_move_group` present |
| cuMotion planner | action advertised, **unit inactive** | `/cumotion/move_group` in `ros2 action list`; `ur10-cumotion` service inactive (would need starting for collision-aware planning) |
| `/joint_states` rate | 125 Hz **but FROZEN** ❌ | 245 msgs / 4 s, **1 unique** position vector (0.0000 mrad range), constant vel[0] = −0.1243 rad/s that never integrates; live would show 4–9 mdeg jitter (RUNBOOK §10) |
| RTDE receive health | **stalled** ❌ | `:30002` socket Recv-Q = **23,823,071 bytes** unread (driver not consuming); `:30001`/`:30004` Recv-Q = 0 |
| Motion controllers | **all inactive** ❌ | `scaled_joint_trajectory_controller`, `joint_trajectory_controller`, `passthrough_trajectory_controller`, `forward_position_controller` → `inactive` |
| State broadcasters | active | `joint_state_broadcaster`, `speed_scaling_state_broadcaster`, `tcp_pose_broadcaster`, `force_torque_sensor_broadcaster`, `io_and_status_controller` → `active` |
| Wrist camera service | active but **no frames** ❌ | `ur10-wrist-cam` active; all `/wrist/*` image topics have a publisher but `ros2 topic hz` → "does not appear to be published yet"; logs show repeated "Frame Corrupted" USB notifications |

**Health summary:** hardware is alive and electrically safe, and the planning stack (MoveIt)
is loaded. But the robot is *not commandable* (no External Control, all motion controllers
inactive), its *telemetry is stale* (frozen RTDE feed), and its *eyes are down* (wrist cam
not streaming). Each is recoverable with the documented runbook step, none of which is in
this read-only task's scope.

## 2 · What is physically on the flange now?

**Could not be determined from a live image.** The wrist camera — the only camera
streaming to this cell — is **not delivering frames** (USB frame-corruption fault, see §1),
so the requested wrist/rig stills could **not** be captured. The target directory
`/home/manishsaini/firstmate/data/isaac-report/rig-readiness/` was created but contains no
stills; see `grab_stills.md` there for the attempts and the QoS/stall evidence.

What *is* known: the forklift gripper kit (`~/datasets/shelf-twin/grippers/forklift_S/`) is
an **Isaac Sim asset package** (`forklift_gripper_S.usd` + a sim driver `forklift_pick_place.py`),
i.e. a *simulation* model being exercised on the shelf twin — **not** a physical gripper
that has been built or mounted. There is no evidence in this read-only check that a physical
fork gripper is on the flange. Determining the real flange tool requires either a working
wrist/overview camera or a captain eyes-on confirmation.

## 3 · Is the shelf in the workspace, and is a Bravo Erdnussflips tray present?

**Could not be confirmed visually** — no camera frames (§2). This question needs a working
camera or captain eyes-on. The scanned cell map (`cumotion/maps/cell-*.nvblx`) places a
shelf front ~1.5 m ahead in `base_link`, but that is a *snapshot* and does not prove the
physical shelf/tray are positioned for a pick *today*.

## 4 · What is missing before a real fork demo can be recorded

**Live-state blockers (recoverable with runbook steps, out of this task's scope):**

- **Re-arm External Control** so the arm is commandable — RUNBOOK §4
  (`resend_robot_program`, then re-activate `scaled_joint_trajectory_controller`), or start
  the `ur10-rearm` watchdog.
- **Restart the driver to clear the frozen RTDE feed** — `systemctl --user restart
  ur10-driver` (RUNBOOK §10); then re-verify `/joint_states` shows live 4–9 mdeg jitter
  before trusting any pose.
- **Recover the wrist camera stream** (USB frame-corruption fault) so the operator can see
  the flange/shelf/tray and record the demo — see `cams/README.md` "Traps".

**Physical / setup prerequisites (not present):**

- **A physical fork gripper mounted on the flange.** Only an Isaac Sim USD model exists; a
  real two-fork passive gripper (forks 120 mm apart) must be fabricated and bolted to the
  flange.
- **A pallet or dock under the tray.** The passive fork pick relies on an S-pallet
  (closed pockets 44×24 mm) or an S-dock (open U-channel). The load rides on friction;
  without a pocket/dock the forks cannot engage.
- **A fork TCP / tool offset.** The forks extend **0.427 m along the tool Z** with their
  centreline **38 mm below** the tool Y axis (`forklift_config.json`: `tine_z1=0.427`,
  `fork_center_y=-0.038`). A TCP at the fork tips (and a correct payload: gripper 0.82 kg
  + tray up to 2.45 kg, well within UR10's 10 kg) must be set on the pendant/driver before
  any Cartesian motion is meaningful. None is configured.
- **A free-space (collision) test / verified cell map.** MoveIt's default scene is empty
  (RUNBOOK §8b); only cuMotion with the cleaned cell map knows the shelf/table. The demo
  waypoints must be validated against `cumotion/measured_joint_limits.yaml` and a free-space
  dry run before execution.
- **A person at the e-stop** for the duration of any real motion (non-negotiable; the
  pendant Safety Force limit + a safeguard device are the real stops).
- **Camera placement for recording** the demo (an external/overview camera; the wrist cam
  is both faulted and the wrong viewpoint for a demo recording).

## 5 · Minimal demo plan (prepared, NOT executed)

A scripted, conservative tray-pick sequence is provided as `fork_demo/fork_pick_demo.py`
(+ `fork_demo/README.md`). It is **plan-only by default**:

- **`--dry-run` (default):** computes and prints the base-frame TCP waypoints for the full
  cycle — **HOME → APPROACH → INSERT → LIFT → EXTRACT → CARRY → PLACE** — derived from the
  fork geometry in `forklift_config.json` and a parametrised base-frame cell layout, and
  saves a 3D path plot. Touches no ROS, commands no motion.
- **Execute path is multiply guarded** and refuses to run unless ALL hold: the explicit
  **`--i-am-at-the-estop`** flag is passed, an interactive **type-`EXECUTE`** confirmation
  is given, the robot is RUNNING/NORMAL with External Control running and a motion
  controller active, the **`/joint_states` feed is verified LIVE** (jitter gate, rejects the
  frozen feed seen today), a **fork TCP file** is supplied, and every waypoint's seeded-IK
  joint solution passes the **`measured_joint_limits.yaml`** window guard. Speeds are capped
  conservatively. **It was not executed in this task** (`--i-am-at-the-estop` was never
  passed); only `--dry-run` was validated.

The joint-space, seeded-IK + per-joint-delta-guard approach follows this project's
guidance (`AGENTS.md`; `cams/approach/approach_demo.py`) to avoid MoveIt's distant-IK
rewinds, and every executed plan is expected to additionally pass
`execute_plan.py --limits cumotion/measured_joint_limits.yaml`.

---

## How to reproduce these checks (read-only)

```bash
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ping -I enp5s0 -c3 192.168.0.1
printf 'robotmode\nsafetystatus\nprogramState\nget loaded program\nquit\n' | nc -w5 192.168.0.1 29999
ros2 topic echo --once /io_and_status_controller/robot_program_running
ss -tn | grep '192.168.0.1:3000'          # watch :30002 Recv-Q
python3 fork_demo/check_joint_feed_live.py  # jitter gate: live vs frozen
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
ros2 topic hz /wrist/color/image_raw        # (currently: no frames)
```
