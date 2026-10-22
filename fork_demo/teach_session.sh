#!/usr/bin/env bash
# Freedrive-TEACH session recorder (task ur10-fork-pick-real-v1).
#
# Brings up the shelf + table D415s (wrist D435i already up as ur10-wrist-cam),
# records a rosbag of all three cameras + joint/TCP/wrench/speed telemetry, and
# runs teach_record.py (which, with --i-will-teach-now, drives the freedrive-safe
# controller state + enables driver freedrive). Everything lands in
# data/ur10-fork-pick-real-v1/<run>/. On exit it stops the bag, stops the
# recorder (which disables freedrive + restores the trajectory controller), and
# kills the orphaned D415 nodes by PID.
#
# Usage:  fork_demo/teach_session.sh <run-name> [--teach]
#   <run-name>   e.g. teach-001
#   --teach      pass --i-will-teach-now --restore-traj to teach_record.py
#                (driver freedrive). Omit it if the captain uses the PENDANT
#                freedrive button instead (record-only).
set -euo pipefail

RUN="${1:?usage: teach_session.sh <run-name> [--teach]}"
TEACH_FLAGS=""
[ "${2:-}" = "--teach" ] && TEACH_FLAGS="--i-will-teach-now --restore-traj"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/data/ur10-fork-pick-real-v1/$RUN"
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

TOPICS=(
  /wrist/color/image_raw /wrist/color_upright/image /wrist/aligned_depth/image
  /shelf/color/image_raw /table/color/image_raw
  /joint_states /tcp_pose_broadcaster/pose
  /force_torque_sensor_broadcaster/wrench
  /speed_scaling_state_broadcaster/speed_scaling
  /io_and_status_controller/robot_program_running /tf /tf_static
)

CAM_PID=""; BAG_PID=""; REC_PID=""
cleanup() {
  echo "[teach-session] tearing down ..."
  [ -n "$REC_PID" ] && kill -INT "$REC_PID" 2>/dev/null || true
  sleep 2
  [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "$CAM_PID" ] && kill "$CAM_PID" 2>/dev/null || true
  pkill -f 'realsense2_camera.*(shelf|table)' 2>/dev/null || true
  echo "[teach-session] done. artifacts under $OUT"
}
trap cleanup EXIT INT TERM

echo "[teach-session] bringing up shelf + table D415s ..."
ros2 launch "$REPO/cams/rig_multicam.launch.py" cams:=shelf,table >"$OUT/cams_launch.log" 2>&1 &
CAM_PID=$!
sleep 8

echo "[teach-session] recording rosbag -> $OUT/bag"
ros2 bag record -o "$OUT/bag" "${TOPICS[@]}" >"$OUT/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 2

echo "[teach-session] starting teach recorder (flags: ${TEACH_FLAGS:-record-only}) ..."
python3 "$REPO/fork_demo/teach_record.py" --out-dir "$OUT" $TEACH_FLAGS &
REC_PID=$!

echo "[teach-session] RECORDING. Captain: hand-guide the arm through the sequence."
echo "[teach-session] Ctrl-C when the teach is complete."
wait "$REC_PID"
