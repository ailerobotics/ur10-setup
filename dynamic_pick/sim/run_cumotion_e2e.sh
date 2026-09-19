#!/usr/bin/env bash
# Dynamic-pick end-to-end proof (c): the segmented re-plan/preempt controller
# planning COLLISION-AWARE legs through the task's OWN cuMotion container (nvblox
# ESDF), on the ISOLATED task domain 83, executed on the mock arm, gated by the
# merged human-safety gate (report §1 candidate C1, §7, §10 Phase A; task item 4).
#
# This is the proof that the segmented loop works with the REAL planner the cell
# uses — cuMotion with read_esdf_world=true — not just the analytic stand-in, and
# it measures cuMotion's solve latency + the goal-preemption latency + the
# achievable re-target rate at the 5 % cap.
#
# GPU DISCIPLINE (task hard rule): the GPU is shared + small (RTX 3060 12 GB, voice
# holds ~2.2 GB). This whole run holds the FLEET GPU LOCK (flock) so it never
# contends with the sibling planner tasks, records nvidia-smi before + peak, and
# bounds its own runtime well under the 30-min cap. It starts its OWN cuMotion
# container (fm-...-cumotion, domain 83) and its OWN mock driver, and stops/removes
# only what it started. It NEVER runs the cell's start_cumotion*.sh (domain 0),
# never touches ur10-cumotion / shelfbot_ros.
export ROS_DOMAIN_ID=83
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$PKG/.." && pwd)"
GATE="$REPO/humangate_driver"
LOGDIR="${LOGDIR:-$REPO/data/human-dynamic-pick-sim-v1/cumotion}"
mkdir -p "$LOGDIR" ~/.cache
CUCONT="fm-human-dynamic-pick-sim-v1-cumotion"
# Default to the DESPECKLED cell map: the standard cell-*.nvblx carries one stray
# weight-1 mid-air voxel that blocked planning from the park region (AGENTS.md);
# the despeckled variant removes exactly that voxel. Both are read-only host maps
# mounted into the task's own container; this never touches the live planner.
DESPECK="$HOME/ur10/cumotion/maps/parkpose-fix-cell-2026-09-04-despeck.nvblx"
MAP="${MAP:-$([ -f "$DESPECK" ] && echo "$DESPECK")}"

# ---- acquire the fleet GPU lock for the whole GPU run ---- #
exec {LOCKFD}>~/.cache/fleet-gpu.lock
echo "[dp-cumotion] acquiring fleet GPU lock..."
if ! flock -w 3600 "$LOCKFD"; then
  echo "[dp-cumotion] could not get the GPU lock in time — do non-GPU work and retry"; exit 4
fi
echo "[dp-cumotion] GPU lock held."
nvidia-smi --query-gpu=memory.total,memory.used,memory.free --format=csv | tee "$LOGDIR/gpu_before.txt"

PIDS=(); DRIVER_PID=""; SMI_PID=""
cleanup() {
  echo "[dp-cumotion] cleanup (own PIDs + own cuMotion container only)"
  [ -n "$SMI_PID" ] && kill "$SMI_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  [ -n "$DRIVER_PID" ] && kill -INT -"$DRIVER_PID" 2>/dev/null && sleep 3
  sleep 2
  [ -n "$DRIVER_PID" ] && kill -9 -"$DRIVER_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  docker rm -f "$CUCONT" >/dev/null 2>&1 && echo "[dp-cumotion] removed $CUCONT"
  # lock released when the script exits (LOCKFD closes)
}
trap cleanup EXIT INT TERM

# ---- peak GPU sampler in the background ---- #
( while true; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; sleep 2; done ) \
  > "$LOGDIR/gpu_samples.txt" 2>/dev/null &
SMI_PID=$!

echo "[dp-cumotion] starting the task cuMotion container (domain 83)"
setsid bash "$HERE/start_cumotion_task.sh" $MAP > "$LOGDIR/cumotion.log" 2>&1 &
# note: start_cumotion_task.sh execs docker run; its process group is the docker CLI

echo "[dp-cumotion] waiting for /cumotion/move_group (nvblox load + cuMotion, up to ~180 s)..."
ok=0
for i in $(seq 1 90); do
  ros2 action list 2>/dev/null | grep -q "/cumotion/move_group" && ok=1 && break
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[dp-cumotion] FAILED: cuMotion move_group never came up"; tail -30 "$LOGDIR/cumotion.log"; exit 1
fi
echo "[dp-cumotion] cuMotion up."

echo "[dp-cumotion] launching mock arm (domain 83) to execute the planned legs"
setsid ros2 launch ur_robot_driver ur10.launch.py \
    use_mock_hardware:=true launch_rviz:=false headless_mode:=true \
    robot_ip:=127.0.0.1 > "$LOGDIR/driver.log" 2>&1 &
DRIVER_PID=$!
for i in $(seq 1 40); do
  ros2 action list 2>/dev/null | grep -q "scaled_joint_trajectory_controller/follow_joint_trajectory" \
    && ros2 topic list 2>/dev/null | grep -q "^/joint_states$" && break
  sleep 1
done
sleep 2   # let the mock arm settle at its home before the bootstrap move

echo "[dp-cumotion] sim bootstrap: parking the arm at the measured reference (cuMotion branch)"
python3 "$HERE/prepose.py" --ros-args \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" > "$LOGDIR/prepose.log" 2>&1
cat "$LOGDIR/prepose.log"

echo "[dp-cumotion] starting gate + watcher (sole motion authority)"
python3 "$GATE/gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$GATE/backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[dp-cumotion] starting human states + moving target + controller (planner=cumotion)"
python3 "$HERE/human_state_publisher.py" --ros-args -p scenario:=dynamic_pick_cumotion \
    > "$LOGDIR/human.log" 2>&1 &
HUMAN_PID=$!; PIDS+=("$HUMAN_PID")
python3 "$HERE/moving_target_publisher.py" --ros-args -p scenario:=dynamic_pick_cumotion \
    > "$LOGDIR/target.log" 2>&1 &
PIDS+=($!)
python3 "$PKG/segmented_pick_node.py" --ros-args \
    -p planner:=cumotion -p cumotion_action:=/cumotion/move_group \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" \
    -p plan_time_s:=5.0 -p log:="$LOGDIR/pick.jsonl" > "$LOGDIR/pick.log" 2>&1 &
PIDS+=($!)

echo "[dp-cumotion] running scenario (~58 s)..."
for i in $(seq 1 70); do kill -0 "$HUMAN_PID" 2>/dev/null || break; sleep 1; done
sleep 3

echo "[dp-cumotion] peak GPU during run:"
sort -n "$LOGDIR/gpu_samples.txt" 2>/dev/null | tail -1 | tee "$LOGDIR/gpu_peak.txt"

echo "[dp-cumotion] analysing"
python3 "$HERE/assert_dynamic_pick.py" --mode cumotion \
    --pick "$LOGDIR/pick.jsonl" --decisions "$LOGDIR/decisions.jsonl" \
    | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[dp-cumotion] logs in $LOGDIR (peak GPU MiB: $(cat "$LOGDIR/gpu_peak.txt" 2>/dev/null))"
exit $RC
