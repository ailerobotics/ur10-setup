# UR10 Runbook — recurring commands

Every command you'll run again and again, in the order you usually need them.
(Also rendered on `report.html`.)

Robot: `192.168.0.1` (CB3) · PC: `192.168.0.2` on `enp5s0` · ROS 2 Jazzy

> Every `ros2` command needs `source /opt/ros/jazzy/setup.bash` first.

## 1 · Is the robot reachable?

```bash
# ALWAYS bind the interface — plain ping false-positives via wifi
ping -I enp5s0 -c3 192.168.0.1

# Robot state via dashboard (no ROS needed)
printf 'robotmode\nsafetymode\nprogramState\nquit\n' | nc -w5 192.168.0.1 29999

# All UR ports in one look
~/ur10/ur-detect.sh
```

## 2 · Driver service (systemd user unit)

```bash
# start (after PC reboot — the unit is transient and does not autostart)
systemd-run --user --unit=ur10-driver -p Restart=on-failure -p RestartSec=5 ~/ur10/ur-bringup.sh

systemctl --user status  ur10-driver     # health
systemctl --user restart ur10-driver     # ALWAYS after rebooting the robot controller
systemctl --user stop    ur10-driver
journalctl --user -u ur10-driver -f      # live logs
```

## 3 · Health checklist (run after any (re)connect)

```bash
ping -I enp5s0 -c2 192.168.0.1                       # 1. link
ss -tn | grep ':50001'                               # 2. command channel — must show ESTAB
ros2 topic hz /joint_states                          # 3. ~125 Hz
ros2 topic echo --once /io_and_status_controller/robot_program_running   # 4. data: true
ros2 topic echo --once /speed_scaling_state_broadcaster/speed_scaling    # 5. > 0
```

## 4 · The #1 fix — robot reconnected / motion ignored / MoveIt Execute rejected

Cable replugged, robot rebooted, **teach button held (hand-guiding kills the program!)**,
or pendant popped "check IP/connection"? The external-control script on the robot is dead —
symptoms: `robot_program_running: false`, trajectory controller `inactive`, MoveIt Execute
fails instantly with "Goal was rejected by server".

**Automated since 2026-08-14:** the `ur10-rearm` watchdog detects this and re-arms within
~10 s (only when robot is RUNNING+NORMAL; resend causes no motion). Start it after a PC reboot:

```bash
systemd-run --user --unit=ur10-rearm -p Restart=on-failure -p RestartSec=10 ~/ur10/ur-rearm-watchdog.sh
journalctl --user -u ur10-rearm -f     # watch it work
```

Manual version (what the watchdog runs):

```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"
```

If controllers went inactive too:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [scaled_joint_trajectory_controller, friction_model_controller], strictness: 1}"
```

Robot controller itself was rebooted? The driver never survives that:

```bash
systemctl --user restart ur10-driver
```

## 5 · Wedged CB3 controller (dashboard ignores everything)

Symptoms: `running` → false, `play` → "Failed to execute", speed pinned at 0.
Reboot the controller from the pendant, then init remotely:

```bash
printf 'power on\nquit\n'      | nc 192.168.0.1 29999   # wait ~6 s
printf 'brake release\nquit\n' | nc 192.168.0.1 29999   # ~15 s to RUNNING
```

## 6 · Motion test

```bash
# VISIBLE test — wrist_1 tilts the whole wrist block ~8.6°; use this to see it move
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/motion_test_visible.py

# minimal tests — wrist_3 = flange rotation, near-INVISIBLE without a tool mounted
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/motion_test.py    # 0.05 rad
source /opt/ros/jazzy/setup.bash && python3 ~/ur10/nudge_wrist3.py   # 0.10 rad
```

Speed slider must be nonzero first:

```bash
ros2 service call /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction "{speed_slider_fraction: 0.3}"
```

## 6b · Goal rejected / test hangs although everything "looks fine"

Usually means **another operator** (person or agent session) touched the robot between
your checks: a `resend_robot_program` from elsewhere drops the reverse connection for a
moment → the watchdog deactivates motion controllers; a foreign slider write can leave
scaling at 0 (goal accepted but frozen). Rule: **one writer at a time** — announce
take/release of the arm. Recovery is §4; if scaling stays 0 after re-arming,
`systemctl --user restart ur10-driver` (also clears any frozen goal).

## 7 · Controllers

```bash
# ros2controlcli is not installed — use the service:
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
```

## 7b · Hardware-in-the-loop (Isaac twin) — bridge in `~/workspaces/ur10-hil`

The physical arm executes the shelfbot mission's motions; the Isaac cell keeps
the shelf, products and gripper virtually. Full bring-up + safety gates:
`~/workspaces/ur10-hil/RUNBOOK.md`. The recurring bits:

```bash
cd ~/workspaces/ur10-hil && source /opt/ros/jazzy/setup.bash && source install/setup.bash
tools/run_hil.sh            # mirror + relay(DRY RUN) + RSP, tmux `hil`
tools/run_hil.sh --live     # relay forwards trajectories to the REAL arm
tools/run_hil.sh down

# end-to-end check (arm nudges wrist_3 ±0.05 rad through the mission action):
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 tools/hil_motion_test.py
```

Verified 2026-08-14: relay round-trip PASS; Isaac twin tracks the real arm
< 2 mrad on all six joints.

## 8 · Live RViz view (watch the arm mirror physical motion)

Official UR view config (copied from `ur_description`) into `rviz/live_view.rviz`.
Runs on the workstation display as its own unit — read-only, safe alongside anything.

```bash
systemd-run --user --unit=ur10-rviz -E DISPLAY=:0 \
  bash -c 'source /opt/ros/jazzy/setup.bash && exec rviz2 -d $HOME/ur10/rviz/live_view.rviz'

systemctl --user stop ur10-rviz          # close it
```

To move the arm by hand: **hold the black teach button** on the back of the CB3 pendant
and push. RViz mirrors it live (telemetry is RTDE, independent of the control program).
If hand-guiding killed ROS motion afterwards → §4.

## 8b · MoveIt — interactive planning in RViz (drag TCP, IK, planner switch)

Uses our patched launch + self-healing starter (`moveit/start_moveit.sh`) — the stock
`ur_moveit_config 3.8.0` launch breaks the RViz MotionPlanning display on Jazzy two ways:
params pre-declared on the RViz node crash `loadRobotModel` (empty RViz, no robot), and
starting bare leaves the display without an IK solver (robot but **no drag ball**). The
starter launches bare, injects the solver post-init, and then needs **one Reset click**.

```bash
systemd-run --user --unit=ur10-moveit -E DISPLAY=:0 ~/ur10/moveit/start_moveit.sh
# wait for the robot to appear, then click "Reset" (bottom-left in RViz) ONCE
# → the orange goal arm + TCP drag ball appear (marker EE:goal_tool0)

systemctl --user stop ur10-moveit        # close it
journalctl --user -u ur10-moveit -f      # logs
# if the ball is missing later: re-run the two `ros2 param set /rviz2_moveit
# robot_description_kinematics...` lines from start_moveit.sh, then Reset again
```

Using it: drag the ball/arrows on the TCP (orange arm = goal state) → **Plan** →
check the preview → **Execute**. Planner switch: Context tab → OMPL / Pilz / CHOMP.
Keep "Velocity Scaling" ≤ 0.1 for interactive use.

**SAFETY: the planning scene is empty** — MoveIt knows the robot's self-collisions
but NOT the table, walls, or anything mounted around the arm. It will happily plan
through them. Watch every Execute; e-stop in reach. The pendant speed slider also
caps execution globally.

## 8c · NVIDIA cuMotion (cuRobo) planner in the same RViz

GPU planner (RTX 3060) running **inside the Isaac shelfbot container** (the host has no
CUDA toolkit, so the deb is uninstallable natively); MoveIt reaches it through the
host-built plugin via action `cumotion/move_group`. In RViz: **Context tab → pipeline
dropdown → `isaac_ros_cumotion`** → Plan/Execute as usual.

```bash
# GPU planner (container; auto-restarts):
systemd-run --user --unit=ur10-cumotion -p Restart=on-failure -p RestartSec=15 ~/ur10/cumotion/start_cumotion.sh
journalctl --user -u ur10-cumotion -f          # "MotionPlan action server initialized"
ros2 action list | grep cumotion               # must show /cumotion/move_group on the host

systemctl --user stop ur10-cumotion            # stop (docker container dies with it)
```

Pieces (all in `cumotion/` + `~/workspaces/cumotion_src`):
- `cumotion/ur10_calibrated.urdf` — dumped from /robot_description (has OUR calibration)
- `cumotion/ur10.xrdf` — ur10e XRDF adapted: same link names, collision buffers 0.01→0.02 m
  (spheres were authored for the e-Series' slightly different link lengths)
- `cumotion/start_cumotion.sh` — container runner (UDPv4 DDS, --net=host, --gpus)
- host plugin: `~/workspaces/cumotion_src` — `colcon build --packages-select
  isaac_ros_cumotion_robot_description isaac_ros_cumotion_moveit` (heavy CUDA pkgs
  COLCON_IGNOREd; the plugin is pure MoveIt C++). start_moveit.sh sources this overlay.
- cuMotion plans with a **ground plane at base height** (add_ground_plane:=true) — it's the
  only planner that knows the table exists. OMPL/Pilz/CHOMP still don't.

## 9 · One-time / rare

```bash
# Re-extract kinematics calibration from the arm
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.0.1 target_filename:=$HOME/ur10/ur10_calibration.yaml

# Reinstall driver stack (keeps the Isaac-repo pin)
sudo bash ~/ur10/install-ur-driver.sh

# PC network profile (already persistent; for reference)
nmcli connection add type ethernet con-name ur10 ifname enp5s0 \
  ipv4.method manual ipv4.addresses "192.168.0.2/24,169.254.101.1/16" \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 10
```

## 10 · Cell map: scan, re-scan, use

Static 3-D obstacle map of the cell in `base_link`, built from the **wrist**
D435i while the captain freedrives the arm, consumed by cuMotion as an ESDF.
Rationale, design decisions and the full procedure with pose/sweep recipes:
`cams/cellmap/README.md`. This section is the recurring commands only.

Frame chain — no `map`/`odom`, no SLAM; the camera pose is pure kinematics:

```
base_link → tool0 → camera_link → camera_color_optical_frame
   /joint_states   ↑ cams/cellmap/calib/tool0_to_camera_<date>.yaml   (RealSense static TF)
```

### Use an existing map (the common case)

```bash
# the map planner replaces the default one — SAME container name, so stop it first
systemctl --user stop ur10-cumotion; docker stop ur10-cumotion   # the unit stop alone can orphan the container
~/ur10/cumotion/start_cumotion_map.sh          # newest cumotion/maps/cell-*.nvblx
```

Starts nvblox (map server, no camera) → `load_map` → cuMotion with
`read_esdf_world:=true` and `cumotion/ur10_map.xrdf` (0.04 m world buffer on
forearm/wrists/tool). cuMotion is started **only after the loader exits 0**:
`read_esdf_world:=true` against a missing ESDF service blocks forever, and
against an empty nvblox it answers `World update failed` to every request —
a planner that looks alive and refuses everything.

```bash
# back to the default (no map) planner — nothing here changes start_cumotion.sh
systemctl --user stop ur10-cumotion; docker stop ur10-cumotion
systemd-run --user --unit=ur10-cumotion -p Restart=on-failure -p RestartSec=15 ~/ur10/cumotion/start_cumotion.sh
```

### Clean an existing map in place (no re-scan)

To make a map usable for planning without re-scanning — e.g. drop the
robot base/pedestal (the fixed base is not masked during a scan, so it stands in
the map as a phantom obstacle where every plan starts) and isolated speckle:

```bash
# writes a NEW cleaned .nvblx; the original is left untouched for rollback
python3 cams/cellmap/tools/nvblx_clean.py \
    ~/ur10/cumotion/maps/cell-2026-09-03.nvblx \
    ~/ur10/cumotion/maps/cell-2026-09-04-cleaned.nvblx \
    --remove-box=-0.60,0.20,-0.32,0.50,-0.65,-0.10 \
    --denoise --min-blocks 4 --self-check
```

A `.nvblx` is a SQLite DB; the tool zeroes the TSDF weight of voxels whose centre
is inside a base_link box (voxel-precise) and drops small isolated blocks — no
ROS, no rig (colocated `test_nvblx_clean.py`). cuMotion recomputes the ESDF it
plans against from the TSDF, so editing the TSDF is what the planner sees. The
tool refuses an output that already exists (`--force` replaces a previous cleaned
map; the source file is never the target). `--denoise` prints one audit line per
removed component (base_link centre, extent, block count): unobserved space reads
as FREE to the planner, so confirm each one is mid-air speckle and not a real
small object (a carton spans 1–3 blocks at 0.16 m), and protect anything real
with `--keep-box=x0,x1,y0,y1,z0,z1`. Audit of the shipped cleaned map:
`data/cell-clean-approach-v1/nvblx-clean-audit.txt`. The
cleaned file becomes operational automatically (loaders take the newest
`cell-*.nvblx`). ALWAYS re-verify before trusting it:

```bash
# load into the planner (must say "cell map loaded … starting cuMotion", no "World update failed")
~/ur10/cumotion/start_cumotion_map.sh cell-2026-09-04-cleaned.nvblx
# regenerate the mesh from the loaded map, then check tags/table/arm
docker exec ur10-cumotion bash -lc 'source /opt/ros/jazzy/setup.bash && \
    python3 /cellmap/tools/map_ctl.py ply /tmp/cleaned.ply'
docker cp ur10-cumotion:/tmp/cleaned.ply ~/ur10/cumotion/maps/cell-2026-09-04-cleaned.ply
python3 cams/cellmap/tools/verify_map.py ~/ur10/cumotion/maps/cell-2026-09-04-cleaned.ply \
    --tag tag0=0.1157,-1.0578,-0.0565 --tag tag1=-0.5057,-1.0575,-0.0572
# table + both tags must still read ~11 mm; arm/base absent. Descriptor: cumotion/maps/cell-2026-09-04-cleaned.md
```

Cleaning is NOT a substitute for re-scanning when geometry actually moved — it
only removes what is already in the map. Re-scan per the rules below.

### Reduced joint windows (only if the pneumatic line / wire strap is on the arm)

Whether the line is on the arm, and whether the captain-confirmed shoulder_pan
±120° still stands, is the captain's call. When it is on, every plan that will
be executed must be checked against per-joint windows in the
`reference_pose_rad` + `window_deg` schema of `cumotion/pneumatic_limits.yaml`
(`plan_test.py --joint-limits`, `execute_plan.py --limits`; both refuse to start
on any other schema). PREFERRED source: the MEASURED hand-sweep limits file
`cumotion/measured_joint_limits.yaml` (next section, "Measuring / declaring
real joint limits"). Only if that file is unusable, regenerate the proposal
windows around the LIVE park pose:

```bash
# reads live /joint_states (no motion); writes a NEW timestamped windows file and
# REFUSES to overwrite an existing one (a measured file is never replaced).
# --out/--xrdf-out MUST be given explicitly (fresh or /tmp path): the generator's
# DEFAULTS are the repo files cumotion/ur10_pneumatic.urdf/.xrdf, which
# run_live.sh (cams/cellmap/live.launch.py) loads as the live-avoidance
# planner's robot description — never overwrite them. Do NOT start the map
# planner on the generated XRDF either unless it was made with
# --keep-world-collision: by default it drops shoulder_link/upper_arm_link
# from world collision (a pedestal workaround, obsolete on a cleaned map). The
# map planner stays on ur10_map.xrdf, or on the measured limited model of the
# next section for anything that will be executed.
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 ~/ur10/cumotion/make_limited_urdf.py \
    --out /tmp/ur10_pneumatic.urdf --xrdf-out /tmp/ur10_pneumatic.xrdf \
    --write-limits ~/ur10/cumotion/pneumatic_limits_live-$(date +%Y%m%d-%H%M).yaml
```

`run_live.sh` itself defaults to those same `ur10_pneumatic.*` files (the
2026-08-28 pedestal workaround with shoulder/upper_arm excluded from world
collision). On the cleaned map that exclusion is no longer needed; pointing the
live launch at the full model (`urdf:=/cfg/ur10_calibrated.urdf
xrdf:=/cfg/ur10_map.xrdf`) is a captain decision and is not changed here.

Then re-validate on a FRESH label with `--joint-limits <that file>` and require
both `--check-box` sweeps ≥ 0 (`cams/cellmap/README.md` §5), and execute only
with `execute_plan.py --exact --limits <that file>`. `--frozen` against
`pneumatic_limits.yaml` centres the windows on its 2026-08-28 reference pose —
never use that for execution. (The measured file is the opposite case: its
reference pose is absolute, so it MUST be used with `--frozen` — next section.)

### Measuring / declaring real joint limits (joint-limits-freedrive-v1)

The authoritative limits file is `cumotion/measured_joint_limits.yaml`: per-joint
windows from a captain **freedrive hand-sweep**, same `reference_pose_rad` +
`window_deg` schema the guards read. Two kinds of window live in it, and the
provenance of each is explicit in the file:

- **measured** — the hand-swept extremes shrunk by `margin_deg` (default 5°),
  recorded under `measured:`. This is the real reachable travel.
- **declared** — a window the captain widened *by decision* to cover a validated
  demo's required travel that the sweep did not reach (recorded under `declared:`,
  `source: captain-declared (NOT hand-swept)`). A declaration only ever WIDENS
  (union with the current window), so it never narrows a joint; the raw measured
  extremes stay on record. Declaring a window PAST a hand-measured physical bind
  is a captain call and a safety red flag — disclose it in the execute note.

**Re-measure / re-declare whenever the line is re-routed or the arm is
re-parked** (the reachable travel changes). Tools (all host, domain 0, plan-only —
none command motion):

```bash
# full sweep: captain hand-drives each joint to its physical extreme in freedrive
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/freedrive.py on   # 4 Hz deadman; leave running
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 cams/cellmap/tools/record_joint_limits.py \
    --out cumotion/measured_joint_limits.yaml           # Ctrl-C writes; jitter-gated (refuses a frozen feed)
# targeted re-sweep of only some joints (extends, never narrows):
#   record_joint_limits.py --merge --joints shoulder_lift,wrist_1 --out <file>
# captain-declared widening (no rig, no sweep) to cover a known demo travel:
#   record_joint_limits.py --declare elbow=-61.7,-14.1 --reason "…" --out <file>

# limited model for the CLEANED map (keeps full world collision), explicit outputs:
python3 cumotion/make_limited_urdf.py --frozen --keep-world-collision \
    --limits cumotion/measured_joint_limits.yaml \
    --out cumotion/ur10_measured.urdf --xrdf-out cumotion/ur10_measured.xrdf
```

The offline test `cams/cellmap/tools/test_record_joint_limits.py` covers the
recorder/declare/merge maths and round-trips through both consumers (no rig). The
executable plan and the definitive captain command are in
`data/cell-clean-approach-v1/captain-execute-note.md`.

### When to re-scan

- Anything the arm can reach was moved, added or removed. The map is a
  **snapshot**; unobserved space is treated as **free**, so a new obstacle is
  invisible to the planner exactly as it was before the map existed.
- The arm was re-calibrated, or the camera was unstrapped/restrapped — the
  mount transform changes and the whole map shifts with it. Re-run the
  hand-eye calibration first, then re-scan.
- Never needed for the tray rig or the approach demo: both are unchanged and
  do not read the map.

### Resuming after a part-finished session (as of 2026-08-27)

Hand-eye calibration is **done and validated**:
`cams/cellmap/calib/tool0_to_camera_2026-08-27.yaml`, published automatically by
`wrist_rig.launch.py` (`calib:=auto`). It does not need repeating unless the
camera is unstrapped or the arm re-calibrated. To pick up at the scan:

```bash
# 1. robot back on the wire (see §1-§4), then freedrive on
# 2. camera rig, with the D435i pinned by serial (a second RealSense is connected)
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=0 ros2 launch cams/approach/wrist_rig.launch.py \
    camera_params:=cams/cellmap/d435i_wrist_serial.yaml
#    confirm: "[wrist_rig] tool0->camera_link from CALIBRATION ..."  (or tf2_echo tool0 camera_link)
# 3. straight to the scan
cams/cellmap/run_scan.sh
DISPLAY=:0 XAUTHORITY=$(ls -t /tmp/xauth_* | head -1) cams/cellmap/run_rviz.sh
cams/cellmap/scan_session.sh cell-$(date +%F)
```

### Hand-eye recalibration (captain hands-on, ~20 min)

Needed whenever the camera is unstrapped/remounted or the arm re-calibrated —
the mount transform changes and the whole map shifts with it. **Recalibrate
before re-scanning, never after.**

```bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 0. robot on the wire (§1-§4). If the pendant shows IDLE the brakes are on:
printf 'power on\nquit\n'      | nc -w5 192.168.0.1 29999
printf 'brake release\nquit\n' | nc -w5 192.168.0.1 29999   # ~15 s to RUNNING
systemctl --user restart ur10-driver        # MANDATORY: a mode change drops RTDE

# 1. freedrive (leave running; Ctrl-C releases and restores the trajectory ctrl)
python3 cams/cellmap/tools/freedrive.py on

# 2. one capture per tag — see cams/cellmap/README.md for the full command
python3 -u cams/cellmap/handeye_calibrate.py --tag-id 0 ... --target 18
python3 -u cams/cellmap/handeye_calibrate.py --tag-id 1 ... --target 18

# 3. combine (refuses if the two tags disagree) + validate
python3 cams/cellmap/tools/combine_tags.py calib/...-tag0.yaml calib/...-tag1.yaml
python3 cams/cellmap/validate_tag1.py --optical wrist_color_upright_optical_frame ...
```

**Two things that will fool you, both seen live on 2026-09-03:**

* `ros2 topic echo --once /io_and_status_controller/robot_mode` (and
  `safety_mode`) is **latched-only** — 0 messages in 8 s of live subscription.
  It replays a stale sample to every new subscriber and read `POWER_OFF` /
  `ROBOT_EMERGENCY_STOP` while the robot was `IDLE` / `NORMAL`. **Use the
  dashboard** (`printf 'robotmode\nsafetystatus\nquit\n' | nc -w5 192.168.0.1 29999`).
* `/joint_states` at a perfect 125 Hz is **not** proof the feed is live. A
  dropped RTDE link republishes the last packet forever: 0.000 mdeg range with
  a true pose completely different from the one displayed. Check **jitter**
  (live = 4-9 mdeg); the fix is `systemctl --user restart ur10-driver`.

This CB3 (PolyScope 3.15.8) stays in mode 7 `RUNNING` during ROS freedrive —
mode 6 `BACKDRIVE` is the pendant's own hand-guide state and is *not* expected
here. There is no Local/Remote toggle to flip either; the dashboard rejects
`is in remote control` as an unknown command. The only proof freedrive engaged
is that the arm moves when pushed.

Calibrate against the stream the map consumes: `/wrist/color_upright/*`
(the wrist D435i is mounted upside down). Every result records its stream in a
`source:` block — quote the stream whenever you quote the numbers.

### Re-scan (captain, ~15 min incl. calibration check)

```bash
cams/cellmap/run_scan.sh                       # segmenter + nvblox; refuses without a calib yaml
cams/cellmap/run_rviz.sh                       # RViz WITH the nvblox plugin (host RViz cannot decode nvblox_msgs/Mesh)
cams/cellmap/scan_session.sh cell-$(date +%F)  # prints the sweep recipe, then Enter → save_map + .ply
```

**What good looks like:** surfaces stay put while the arm moves; table at
z ≈ 0 and shelf front at y ≈ 1.5 (`cams/cellmap/tools/map_ctl.py esdf --probe x,y`);
the arm itself absent (robot segmenter). **Smear** that follows the arm =
calibration or timestamp error — stop and do not save, re-run the hand-eye
calibration.

Maps live in `cumotion/maps/` when under ~50 MB (committed), otherwise in
`~/ur10/cumotion/maps/` on the host (see `cumotion/maps/README.md`).

### Known limits

Snapshot only; coverage = what the wrist camera saw from the reach envelope
(walls beyond reach are out of scope); min range ~17 cm, so surfaces closer
than that to the lens are not mapped; error budget ≈ calibration residual +
2 cm voxels + ~1 %-of-range depth noise, which the 0.04 m buffer absorbs but
thin objects may defeat. Ground plane and the approach demo's whitelist boxes
stay on as independent guards.
