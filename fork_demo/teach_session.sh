#!/usr/bin/env bash
# Freedrive-TEACH session recorder -- DISK-LIGHT, NO CAMERAS (task ur10-fork-pick-real-v1).
#
# Captain's word (2026-10-22): "skip recording camera data in the teach run."
# So the teach records ONLY telemetry, into the firstmate data dir (NOT the
# worktree): teach_record.py's full-rate JSONL (joints + computed fork TCP +
# wrench + speed + dwell markers) plus a SMALL rosbag of /joint_states + /tf +
# /tf_static (no image topics -> a few MB/min, not the 51 GB a 3-camera raw bag
# cost). Camera video is recorded ONLY for the later replay demo, compressed
# (see fork_demo/record_cameras_mp4.py), never raw frames into a bag.
#
# Free-space guard: refuses to start below 20 GB free; teach_record.py stops the
# session cleanly if free space falls below 5 GB.
#
# Usage:  fork_demo/teach_session.sh <run-name> [--teach]
#   --teach  drive the freedrive-safe controller state + enable driver freedrive
#            (omit for record-only if the captain uses the pendant freedrive button)
set -eo pipefail

RUN="${1:?usage: teach_session.sh <run-name> [--teach]}"
TEACH_FLAGS=""
[ "${2:-}" = "--teach" ] && TEACH_FLAGS="--i-will-teach-now --restore-traj"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="/home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/$RUN"
mkdir -p "$OUT"

MIN_FREE_START_GB=20
FREE_GB=$(df -BG --output=avail "$OUT" | tail -1 | tr -dc '0-9')
if [ "${FREE_GB:-0}" -lt "$MIN_FREE_START_GB" ]; then
  echo "[teach-session] REFUSING to start: only ${FREE_GB} GB free (< ${MIN_FREE_START_GB} GB)."
  exit 1
fi
echo "[teach-session] ${FREE_GB} GB free -- ok to start."

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# NO image topics -- telemetry only.
BAG_TOPICS=(/joint_states /tf /tf_static
            /tcp_pose_broadcaster/pose
            /force_torque_sensor_broadcaster/wrench
            /speed_scaling_state_broadcaster/speed_scaling
            /io_and_status_controller/robot_program_running)

BAG_PID=""; REC_PID=""
cleanup() {
  echo "[teach-session] tearing down ..."
  [ -n "$REC_PID" ] && kill -INT "$REC_PID" 2>/dev/null || true
  sleep 2
  [ -n "$BAG_PID" ] && kill -INT "$BAG_PID" 2>/dev/null || true
  echo "[teach-session] done. artifacts under $OUT"
}
trap cleanup EXIT INT TERM

echo "[teach-session] recording small telemetry bag -> $OUT/bag"
ros2 bag record -o "$OUT/bag" "${BAG_TOPICS[@]}" >"$OUT/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 2

echo "[teach-session] starting teach recorder (flags: ${TEACH_FLAGS:-record-only}) ..."
python3 "$REPO/fork_demo/teach_record.py" --out-dir "$OUT" $TEACH_FLAGS &
REC_PID=$!

echo "[teach-session] RECORDING (NO cameras). Captain: hand-guide the arm."
echo "[teach-session] Ctrl-C when the teach is complete."
wait "$REC_PID"
