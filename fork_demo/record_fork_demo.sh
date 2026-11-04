#!/usr/bin/env bash
# REPLAY-demo recorder (task ur10-fork-pick-real-v1) -- DISK-LIGHT.
#
# For the two captain-gated REPLAY runs (front dry run, real pick-and-place).
# Records all three cameras as COMPRESSED H.264 mp4 at 15 fps (via
# record_cameras_mp4.py -> bundled ffmpeg) -- NEVER raw image frames into a
# rosbag -- plus a SMALL telemetry bag (/joint_states /tf + fork TCP pose +
# wrench + speed). Everything into the firstmate data dir, NOT the worktree.
# The TABLE D415 is the side-by-side angle vs the Isaac mp4.
#
# Refuses to start below 20 GB free; the mp4 recorder stops below 5 GB.
# Records telemetry only; commands NO motion. Run alongside the gated replay.
#
# Usage:  fork_demo/record_fork_demo.sh <run-name>      e.g. real-pick-001
set -eo pipefail

RUN="${1:?usage: record_fork_demo.sh <run-name>}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="/home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/$RUN"
mkdir -p "$OUT"

MIN_FREE_START_GB=20
FREE_GB=$(df -BG --output=avail "$OUT" | tail -1 | tr -dc '0-9')
if [ "${FREE_GB:-0}" -lt "$MIN_FREE_START_GB" ]; then
  echo "[replay-rec] REFUSING to start: only ${FREE_GB} GB free (< ${MIN_FREE_START_GB} GB)."
  exit 1
fi

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

BAG_TOPICS=(/joint_states /tf /tf_static /tcp_pose_broadcaster/pose
            /force_torque_sensor_broadcaster/wrench
            /speed_scaling_state_broadcaster/speed_scaling
            /io_and_status_controller/robot_program_running)

CAM_PID=""; BAG_PID=""; MP4_PID=""
cleanup() {
  echo "[replay-rec] tearing down ..."
  [ -n "$MP4_PID" ] && kill -INT "$MP4_PID" 2>/dev/null || true
  sleep 3
  [ -n "$BAG_PID" ] && kill -INT "$BAG_PID" 2>/dev/null || true
  [ -n "$CAM_PID" ] && kill "$CAM_PID" 2>/dev/null || true
  pkill -f 'realsense2_camera.*(shelf|table)' 2>/dev/null || true
  echo "[replay-rec] done. artifacts under $OUT"
}
trap cleanup EXIT INT TERM

echo "[replay-rec] bringing up shelf + table D415s (wrist already up) ..."
ros2 launch "$REPO/cams/rig_multicam.launch.py" cams:=shelf,table >"$OUT/cams_launch.log" 2>&1 &
CAM_PID=$!
sleep 8

echo "[replay-rec] small telemetry bag -> $OUT/bag"
ros2 bag record -o "$OUT/bag" "${BAG_TOPICS[@]}" >"$OUT/bag_record.log" 2>&1 &
BAG_PID=$!

echo "[replay-rec] compressed camera mp4s -> $OUT/{wrist,shelf,table}.mp4 @15fps"
python3 "$REPO/fork_demo/record_cameras_mp4.py" --out-dir "$OUT" \
  --cameras wrist,shelf,table --fps 15 >"$OUT/mp4_record.log" 2>&1 &
MP4_PID=$!

echo "[replay-rec] RECORDING. Run the gated replay now; Ctrl-C when complete."
wait "$MP4_PID"
