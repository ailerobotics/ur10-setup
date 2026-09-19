#!/usr/bin/env bash
# Dynamic-pick end-to-end proof (a): the segmented re-plan/preempt controller
# driving the driver's MOCK HARDWARE, gated by the merged human-safety gate
# (report §7, §10 Phase A; task brief item 4). No Docker, no GPU, no real robot.
#
# Brings up our OWN ur_robot_driver (use_mock_hardware:=true) on the ISOLATED task
# domain 83, replays a scripted moving pick target AND scripted human-safety states
# (the REAL camera wire format — no cameras opened), runs the merged gate +
# independent watcher (from humangate_driver, unchanged) as the sole motion
# authority, and runs the dynamic-pick controller with the GPU-free analytic
# planner. In mock the speed slider is inert, so SLOW/STOP are effected by the
# gate's goal-cancel backstop (holds the arm) — the mock-appropriate proof.
#
# Everything this script starts is stopped BY PID on exit (never by name; the
# driver by process group so no child node leaks). It starts its OWN driver and
# never touches ur10-driver, the robot containers, or anything it did not start.
export ROS_DOMAIN_ID=83
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$PKG/.." && pwd)"
GATE="$REPO/humangate_driver"
LOGDIR="${LOGDIR:-$REPO/data/human-dynamic-pick-sim-v1/mock}"
mkdir -p "$LOGDIR"
: > "$LOGDIR/driver.log"

PIDS=(); DRIVER_PID=""
cleanup() {
  echo "[dp-mock] cleanup (by PID; driver by process group)"
  [ -n "$DRIVER_PID" ] && kill -INT -"$DRIVER_PID" 2>/dev/null && sleep 3
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  [ -n "$DRIVER_PID" ] && kill -9 -"$DRIVER_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

echo "[dp-mock] launching ur_robot_driver (use_mock_hardware:=true) on domain 83"
setsid ros2 launch ur_robot_driver ur10.launch.py \
    use_mock_hardware:=true launch_rviz:=false headless_mode:=true \
    robot_ip:=127.0.0.1 > "$LOGDIR/driver.log" 2>&1 &
DRIVER_PID=$!

echo "[dp-mock] waiting for /joint_states + FJT action..."
ok=0
for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -q "/joint_states" && \
     ros2 action list 2>/dev/null | grep -q "follow_joint_trajectory"; then ok=1; break; fi
  sleep 1
done
[ "$ok" = "1" ] || { echo "[dp-mock] FAILED: mock driver did not come up"; tail -20 "$LOGDIR/driver.log"; exit 1; }
echo "[dp-mock] driver up."

echo "[dp-mock] sim bootstrap: parking the arm at the in-window reference pose"
python3 "$HERE/prepose.py" --ros-args \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" > "$LOGDIR/prepose.log" 2>&1
cat "$LOGDIR/prepose.log"

echo "[dp-mock] starting gate + watcher (sole motion authority)"
python3 "$GATE/gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$GATE/backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[dp-mock] starting human states + moving target + controller (aligned clocks)"
python3 "$HERE/human_state_publisher.py" --ros-args -p scenario:=dynamic_pick_full \
    > "$LOGDIR/human.log" 2>&1 &
HUMAN_PID=$!; PIDS+=("$HUMAN_PID")
python3 "$HERE/moving_target_publisher.py" --ros-args -p scenario:=dynamic_pick_full \
    > "$LOGDIR/target.log" 2>&1 &
PIDS+=($!)
python3 "$PKG/segmented_pick_node.py" --ros-args \
    -p planner:=analytic \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" \
    -p log:="$LOGDIR/pick.jsonl" > "$LOGDIR/pick.log" 2>&1 &
PIDS+=($!)

echo "[dp-mock] running scenario (~58 s)..."
for i in $(seq 1 70); do kill -0 "$HUMAN_PID" 2>/dev/null || break; sleep 1; done
sleep 3   # let the controller flush its summary on the last legs

echo "[dp-mock] scenario done; analysing"
python3 "$HERE/assert_dynamic_pick.py" --mode mock \
    --pick "$LOGDIR/pick.jsonl" --decisions "$LOGDIR/decisions.jsonl" \
    | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[dp-mock] logs in $LOGDIR"
exit $RC
