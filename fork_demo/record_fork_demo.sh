#!/usr/bin/env bash
# Record all three cameras + /joint_states for a fork-demo run (step 4).
#
# Brings up the shelf + table D415s (the wrist D435i is already up as the
# ur10-wrist-cam service), records a rosbag of the colour streams + wrist depth +
# the joint/TCP/speed telemetry into data/ur10-fork-pick-real-v1/<run>/, and on
# exit tears the D415 launch down AND kills its orphaned child nodes by PID (the
# known orphan-reparenting trap, cams/README.md). The TABLE view is the intended
# side-by-side recording angle (vs the Isaac mp4).
#
# READ-ONLY w.r.t. the arm: records telemetry, commands no motion. Run this in
# one terminal, then run the gated real_fork_pick.py hand-off in another with a
# person at the e-stop.
#
# Usage:  fork_demo/record_fork_demo.sh <run-name>      e.g. front-dry-run-001
set -euo pipefail

RUN="${1:?usage: record_fork_demo.sh <run-name>}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/data/ur10-fork-pick-real-v1/$RUN"
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

TOPICS=(
  /wrist/color/image_raw
  /wrist/color_upright/image
  /wrist/aligned_depth/image
  /shelf/color/image_raw
  /table/color/image_raw
  /joint_states
  /tcp_pose_broadcaster/pose
  /speed_scaling_state_broadcaster/speed_scaling
  /io_and_status_controller/robot_program_running
)

CAM_PID=""
BAG_PID=""
cleanup() {
  echo "[record] tearing down ..."
  [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "$CAM_PID" ] && kill "$CAM_PID" 2>/dev/null || true
  # orphan trap: kill any shelf/table realsense child nodes, leave /wrist alone
  pkill -f 'realsense2_camera.*(shelf|table)' 2>/dev/null || true
  sleep 1
  echo "[record] done. bag + stills under $OUT"
}
trap cleanup EXIT INT TERM

echo "[record] bringing up shelf + table D415s ..."
ros2 launch "$REPO/cams/rig_multicam.launch.py" cams:=shelf,table >"$OUT/cams_launch.log" 2>&1 &
CAM_PID=$!
sleep 8   # let the D415 streams come up

echo "[record] recording rosbag -> $OUT/bag"
ros2 bag record -o "$OUT/bag" "${TOPICS[@]}" >"$OUT/bag_record.log" 2>&1 &
BAG_PID=$!

echo "[record] RECORDING. Run the gated real_fork_pick.py hand-off now."
echo "[record] press Ctrl-C when the run is complete to stop + tear down."
wait "$BAG_PID"
