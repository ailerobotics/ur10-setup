#!/usr/bin/env bash
# Dynamic-pick GRASP end-to-end proof: the segmented re-plan/preempt controller
# following a target THROUGH the near/grasp bands to a completed grasp + retreat
# (report §3), on the driver's MOCK HARDWARE, gated by the merged human-safety gate.
# SIMULATION ONLY, isolated task domain 83. No Docker, no GPU, no real robot.
#
# This is the companion to run_mock_e2e.sh (which measures follow-and-preempt in a
# never-reached ping-pong regime): here the target CONVERGES so the tool point
# enters NEAR then GRASP while the target keeps moving, the grasp completes and the
# controller plans the FIXED retreat leg. It proves the per-phase re-plan policy
# holds on the FRESH-leg path too (GRASP frozen; NEAR limited to the pre-cleared
# box), not only on preemption.
#
# Everything this script starts is stopped BY PID on exit (driver by process group).
export ROS_DOMAIN_ID=83
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$PKG/.." && pwd)"
GATE="$REPO/humangate_driver"
LOGDIR="${LOGDIR:-$REPO/data/human-dynamic-pick-sim-v1/grasp}"
RETREAT_XYZ="0.45,-0.25,0.50"
mkdir -p "$LOGDIR"
: > "$LOGDIR/driver.log"

PIDS=(); DRIVER_PID=""
cleanup() {
  echo "[dp-grasp] cleanup (by PID; driver by process group)"
  [ -n "$DRIVER_PID" ] && kill -INT -"$DRIVER_PID" 2>/dev/null && sleep 3
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  [ -n "$DRIVER_PID" ] && kill -9 -"$DRIVER_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

echo "[dp-grasp] launching ur_robot_driver (use_mock_hardware:=true) on domain 83"
setsid ros2 launch ur_robot_driver ur10.launch.py \
    use_mock_hardware:=true launch_rviz:=false headless_mode:=true \
    robot_ip:=127.0.0.1 > "$LOGDIR/driver.log" 2>&1 &
DRIVER_PID=$!

echo "[dp-grasp] waiting for /joint_states + FJT action..."
ok=0
for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -q "/joint_states" && \
     ros2 action list 2>/dev/null | grep -q "follow_joint_trajectory"; then ok=1; break; fi
  sleep 1
done
[ "$ok" = "1" ] || { echo "[dp-grasp] FAILED: mock driver did not come up"; tail -20 "$LOGDIR/driver.log"; exit 1; }
echo "[dp-grasp] driver up."

echo "[dp-grasp] sim bootstrap: parking the arm at the in-window reference pose"
python3 "$HERE/prepose.py" --ros-args \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" > "$LOGDIR/prepose.log" 2>&1
cat "$LOGDIR/prepose.log"

echo "[dp-grasp] starting gate + watcher (sole motion authority)"
python3 "$GATE/gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$GATE/backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[dp-grasp] starting human states + moving target + controller (aligned clocks)"
python3 "$HERE/human_state_publisher.py" --ros-args -p scenario:=dynamic_pick_grasp \
    > "$LOGDIR/human.log" 2>&1 &
HUMAN_PID=$!; PIDS+=("$HUMAN_PID")
python3 "$HERE/moving_target_publisher.py" --ros-args -p scenario:=dynamic_pick_grasp \
    > "$LOGDIR/target.log" 2>&1 &
PIDS+=($!)
python3 "$PKG/segmented_pick_node.py" --ros-args \
    -p planner:=analytic \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" \
    -p retreat_xyz:="$RETREAT_XYZ" -p grasp_settle_s:=3.0 \
    -p log:="$LOGDIR/pick.jsonl" > "$LOGDIR/pick.log" 2>&1 &
PIDS+=($!)

echo "[dp-grasp] running scenario (~46 s)..."
for i in $(seq 1 60); do kill -0 "$HUMAN_PID" 2>/dev/null || break; sleep 1; done
sleep 3   # let the controller flush its summary on the last legs

echo "[dp-grasp] scenario done; analysing"
python3 "$HERE/assert_grasp.py" --pick "$LOGDIR/pick.jsonl" --retreat "$RETREAT_XYZ" \
    | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[dp-grasp] logs in $LOGDIR"
exit $RC
