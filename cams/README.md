# cams — RealSense rig for the shelfbot cell

Real cameras take over the exact topic contract the Isaac twin publishes, so
the shelfbot perception stack (container `shelfbot_ros`, ROS domain **42**)
needs zero code changes. This directory is the host-side camera bring-up:
launch, RViz view, bench tools, and the placement decision report.

Status 2026-08-17 (first bring-up day): one D435i (serial `018322070619`,
fw 5.12.8.200) streaming; contract topics live at ~28 Hz; the trained
`tray_perception` detector verified live on GPU in the container. The camera
is still loose (temporary table spot) — world poses stay approximate until it
is mounted and calibrated. Placement + mounting decisions:
[placement-report.html](placement-report.html).

## Contract produced

| Topic | Type / encoding | Note |
|---|---|---|
| `/camera/rgb` | Image `rgb8` 1280x720@30 | YOLO input |
| `/camera/depth/image` | Image `32FC1` metres, aligned to RGB | `depth_image_proc convert_metric` from the wrapper's 16UC1 mm |
| `/camera/depth/camera_info` | CameraInfo | aligned-depth intrinsics |
| TF `world -> PerceptionCamera` | static | RGB **optical** frame; look-at from `cam_pos`/`cam_aim` until hand-eye calibration replaces it |

## Install (done 2026-08-17, Ubuntu 24.04 / ROS Jazzy / kernel 7.0)

```
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-depth-image-proc
```

ROS-pinned RSUSB build — **no dkms** (would not build on kernel 7.0). Check
`apt policy` first: the NVIDIA Isaac apt repo on this box has outranked
packages.ros.org before (see `/etc/apt/preferences.d/99-ur-from-ros2.pref`);
for these packages it did not compete.

**udev rules are NOT shipped** by the vendored `ros-jazzy-librealsense2` deb
(validated: `dpkg -L` has none) — without them the camera enumerates but
cannot stream (no write access on the USB node). Fix: install Intel's
official rules file for the matching librealsense version:

```
curl -fsSLO https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.58.1/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Run

```
# camera rig (domain 42 + UDPv4 so the container sees it, HIL-bridge pattern)
ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 launch ~/ur10/cams/rig.launch.py cam_pos:=-1.2,-0.6,0.85 cam_aim:=0.3,-0.9,0.8

# RViz view (RGB, detection mask, trained-model overlay, pointcloud, TF)
ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 rviz2 -d ~/ur10/cams/rig.rviz

# pipeline tray detector (inside shelfbot_ros; generic open-vocab model for now)
docker exec shelfbot_ros bash -c "source /opt/ros/jazzy/setup.bash && source /ros_ws/install_container/setup.bash \
  && cd /root/weights && /opt/yolo_venv/bin/python3 -m shelfbot.perception.tray_pose_node \
  --ros-args -p use_sim_time:=false -p model:=/root/weights/yolov8s-worldv2.pt -p region_radius_m:=10.0"

# trained tray_perception model, live annotated view on /detect_new/annotated (bench harness)
docker exec shelfbot_ros bash -c "source /opt/ros/jazzy/setup.bash \
  && /opt/yolo_venv/bin/python3 /root/tray_live_loop.py /root/weights/tray_best_vm.pt"
```

`tools/`: `grab_frame.py` (one RGB+depth frame + stats), `grab_bbox.py` (wait
for a detection, save annotated frame), `tray_best_live.py` (one-shot custom
model on a live frame), `tray_live_loop.py` (continuous annotator publishing
`/detect_new/annotated`; the container copy is what runs).

`approach/`: the eye-in-hand demo — wrist D435i on the tool flange, domain 0,
IMU on, detector bridge + plan-confirm-execute approach script. Own README.
In that mode the world->PerceptionCamera look-at TF below MUST NOT be
published (it is gated off in `approach/wrist_rig.launch.py`).

## Multi-camera rig (v2, 2026-08-31)

The rig grew from one camera to three: the wrist **D435i** (`018322070619`,
repositioned on the flange, **mounted upside down** — next section) plus two
static **D415**s — SHELF (high/frontal) and `025522060953` (TABLE, workspace
side view). The SHELF unit is whichever second D415 is plugged in:
`024122061936` on 2026-08-31, `021422061772` since 2026-09-03 (the 08-31 unit
is no longer connected; `rs-enumerate-devices -s` is the authority, and
`params/shelf_d415.yaml` must carry the connected serial, quoted). The two
D415s are interchangeable, so the SHELF/TABLE serial assignment is provisional
until mounted (Step 2); swap the two params YAMLs if reversed.

```
# All three on domain 0 (local RViz), each pinned by a quoted params YAML:
ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 launch cams/rig_multicam.launch.py \
    cams:=wrist,shelf,table color_profile:=848x480x30 depth_profile:=848x480x30

# Bring cameras up incrementally as they are plugged in / mounted:
ros2 launch cams/rig_multicam.launch.py cams:=wrist

# RViz: 3D context (UR10 model + TF + pointclouds after Step-2 extrinsics)
ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 DISPLAY=:0 \
  rviz2 -d cams/rig_multicam.rviz

# Captain-facing "are the cameras live?" view: all 3 colour feeds in ONE
# window, side by side, labelled, live. Prefer this over RViz for eyeballing
# feeds (see the RViz-tabbing trap below).
ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 DISPLAY=:0 \
  python3 cams/tools/rig_feed_montage.py
```

```
# Are the feeds LIVE (not latched)? Bounded check: frames in a window, Hz, and
# the age of the newest header stamp against the wall clock. Exit 0 = all live.
python3 cams/tools/rig_feed_check.py                     # the three raw colour feeds
python3 cams/tools/rig_feed_check.py --topic /wrist/color_upright/image_raw
```

Per-camera topics: `/<role>/color/image_raw`, `/<role>/aligned_depth/image`
(metres) + `/image_mm` (raw), `/<role>/points`. Each camera runs its own node
in namespace `/wrist|/shelf|/table` pinned by `cams/params/<role>_*.yaml`.
Launch args beyond `cams:=`: `upright:=true` (default; the rot180 twin for
cameras flagged upside down, below), `reset:=true` (SDK `hardware_reset()` of
the listed cameras before streaming — the software reset, see the 09-03
traps), `imu:=true` (wrist accel+gyro), `color_profile` / `depth_profile`.
Extrinsics are NOT set here — that is Step 2 (AprilTag world-alignment for the
D415s; hand-eye re-calibration for the repositioned wrist, whose
`cams/cellmap/calib/tool0_to_camera_2026-08-27.yaml` is now STALE).

## Wrist camera is mounted upside down (2026-09-03)

The wrist D435i sits inverted in its flange bracket, so the driver's raw colour
image is upside down for humans. realsense2_camera cannot rotate, so
`rig_multicam.launch.py` runs `tools/image_rot180.py` beside the wrist driver
and publishes an **upright twin** of the colour and aligned-depth streams —
image **and** a matching CameraInfo **and** its own optical frame, i.e. a
complete, consistent camera model rather than a display hack:

| raw (driver, untouched) | upright twin (rot180) |
|---|---|
| `/wrist/color/image_raw` + `camera_info`, frame `wrist_color_optical_frame` | `/wrist/color_upright/image_raw` + `camera_info`, frame `wrist_color_upright_optical_frame` |
| `/wrist/aligned_depth/image` (m) + `/image_mm` + `camera_info` | `/wrist/aligned_depth_upright/image` (m) + `camera_info` (mm: `cellmap/tools/depth_to_mm.py`) |
| `/wrist/points`, `/wrist/imu` | — (3D data has no "up"; use TF) |

Camera model of the rotation (pixel-centre convention): `u' = (W-1)-u`,
`v' = (H-1)-v`; fx/fy unchanged; `cx' = (W-1)-cx`, `cy' = (H-1)-cy`; radial
distortion unchanged, tangential p1/p2 negated; P gets the same cx/cy shift.
Verified live on 848x480: cx 430.45 -> 416.55, cy 230.17 -> 248.83. The node
also broadcasts the static TF `wrist_color_optical_frame ->
wrist_color_upright_optical_frame` = Rz(180 deg), quaternion (0,0,1,0), so
both frames live in one tree.

**What calibration and perception must know.** The upright twin is the stream
the hand-eye calibration was solved against and the stream the cell map
consumes (2026-09-03, `cams/cellmap/README.md` "Recalibration for the
UPSIDE-DOWN mount"): the extrinsic on file is `tool0 ->
wrist_color_upright_optical_frame`. An extrinsic is only meaningful together
with the stream it was solved on — solving on raw and applying to upright (or
vice versa) is a silent 180 deg roll about the optical axis that no residual
reveals. Never pair an image from one twin with the CameraInfo or frame of the
other; the raw topics are left exactly as the driver publishes them so nothing
that already consumes them changes. AprilTag detection itself is
rotation-invariant (36h11 decodes in any orientation and the corner order
follows the tag), so tags work on either stream. The montage and
`rig_multicam.rviz` show the upright twin for the wrist. The IMU stays in the
driver's frame — the old `cam_roll_deg` / gravity-residual assumptions in
`approach/` predate the inverted mount and are stale.

## Traps hit on 2026-09-03 (rig v2 bring-up + flip)

- **`Color Sensor Failure! RS4XX: RGB modules inconsistency - 2 found`** on a
  D415 at enumeration = the kernel could not probe that unit's RGB UVC
  interface: `journalctl -k | grep '4-1:1.3'` shows `Failed to query ... -71`
  (EPROTO) and `probe with driver uvcvideo failed`. It is the USB **link**, not
  the SDK: host port `4-1` did this to `024122061936` on 09-01 and to
  `025522060953` on 09-03 while the same units were fine elsewhere. The
  software reset (`ros2 launch cams/rig_multicam.launch.py cams:=table
  reset:=true`) re-enumerates the device; on this port it came back with the
  same EPROTO errors and then **dropped off the bus entirely** (`The device has
  been disconnected!`, `/dev/video6: No such file`). After that only a physical
  replug helps — into a different port/cable. The node keeps retrying
  (`NOT found. Will Try again`) so it picks the camera up when replugged.
- **Best-effort image drops under subscriber load**: 848x480 rgb8 at 30 Hz is
  ~36 MB/s per subscriber over UDPv4 and every fragment must land in a
  212 KB default socket buffer (`net.core.rmem_default`); `/proc/net/snmp`
  `Udp: RcvbufErrors` grows ~500/s with the montage + RViz + flip nodes up,
  and per-subscriber rates read 20-29 Hz while `ros2 topic hz` alone reads
  ~27 Hz. Frames stay fresh (stamp age < 0.1 s) — it is loss, not latency.
  Raising `rmem_default`/`rmem_max` is a sudo sysctl (captain); fewer
  subscribers or a smaller profile also help.

## Traps hit on 2026-08-17 (each cost real time)

- **`serial_no` degrades to a double** through every launch path — CLI nested
  quotes, python dict str, even `ParameterValue(value_type=str)`: launch_ros
  writes the params YAML unquoted and rcl types `018322070619` as a number,
  which the node refuses. With one camera, leave `serial` empty (first
  device). For the multi-camera rig, pin serials via a hand-written, properly
  quoted params YAML — not launch arguments.
- **Orphaned camera node holds the device**: killing a `ros2 launch` can leave
  `realsense2_camera_node` alive; the next launch then starves. Check
  `pgrep -af realsense2_camera_node` before relaunching.
- **Container venv torchvision ABI**: `/opt/yolo_venv` had the PyPI
  `torchvision 0.24.0` against the container's `torch 2.9.0+cu130` →
  `operator torchvision::nms does not exist` at first YOLO load. Fix per
  `shelfbot-ws` SETUP notes: reinstall `torchvision==0.24.0` from the
  `download.pytorch.org/whl/cu130` index (`--no-deps --force-reinstall`).
  The sim never hit this: canonical restock runs use the depth-plane
  detector, so YOLO was never loaded.
- **Stale `ros2` CLI daemon** can report a live topic as unpublished after
  containers/nodes restart; `ros2 daemon stop` and retry before debugging.

## Traps hit on 2026-08-31 (multi-camera rig)

- **Infrared streams starve a shared USB bus**: realsense2_camera opens the two
  IR streams by default at the depth module's native profile (1280x720x30 Y8 on
  the D415) — streams we never use (we need only colour + aligned depth +
  pointcloud). With both D415s on ONE USB3 controller (they enumerated on bus
  004 here; the D435i was alone on bus 006), the extra IR payload triggered
  continuous `Incomplete video frame detected` and dropped colour to ~25 Hz.
  Fix: `enable_infra1:=false enable_infra2:=false` (baked into
  rig_multicam.launch.py) — all three then hold ~30 Hz, zero incomplete frames.
- **All cameras share one TF frame tree unless prefixed**: the realsense frame
  ids (`camera_link`, `camera_color_optical_frame`, ...) come from the
  `camera_name` PARAMETER, not the node namespace — so three nodes all publish
  the SAME frames, a collision that scrambles RViz's TF view and makes every
  pointcloud claim one frame. Set `camera_name:=<role>` per camera (done in the
  launch) → `wrist_/shelf_/table_*` frames. Note `camera_name` sets the TF
  prefix only; topic names come from node namespace+name.
- **RViz tabs multiple Image panels behind Displays**: with three Image
  displays and no saved dock-state blob, RViz2 stacks all three image panels as
  tabs behind the Displays panel on load — the feeds publish fine but are
  hidden, and there is no headless way to un-tab them (no wmctrl/xdotool here,
  and hand-writing the QMainWindow-state blob is impractical). For a reliable
  captain-facing feed check use `cams/tools/rig_feed_montage.py` (one OpenCV
  window, three colour feeds tiled + labelled + live-dot). Also: a killed rig
  bring-up can leave a stale RViz **in a docker container** (cell-map ran
  `ur10-cellmap-rviz-*`) on :0 showing a dead view — `docker ps | grep rviz`
  and stop it, don't just hunt host `rviz2` PIDs.

## Next (from the placement report)

Mount MAIN (tension pole ~(−0.9, −0.1), cam at (−0.70, −0.25, 2.2), 50° down)
and SHELF (ceiling anchor (0.30, −0.90), cam at 2.6 m, 57°); wrist D435i
bracket into the gripper CAD; then scripted hand-eye calibration + AprilTag
drift canary. Wiring the trained `tray_perception` model into
`tray_pose_node` needs a small shelfbot change (guard the world-model
`set_classes` call, filter to the `tray` class, pass `imgsz=960`).
