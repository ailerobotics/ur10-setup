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

## Next (from the placement report)

Mount MAIN (tension pole ~(−0.9, −0.1), cam at (−0.70, −0.25, 2.2), 50° down)
and SHELF (ceiling anchor (0.30, −0.90), cam at 2.6 m, 57°); wrist D435i
bracket into the gripper CAD; then scripted hand-eye calibration + AprilTag
drift canary. Wiring the trained `tray_perception` model into
`tray_pose_node` needs a small shelfbot change (guard the world-model
`set_classes` call, filter to the `tray` class, pass `imgsz=960`).
