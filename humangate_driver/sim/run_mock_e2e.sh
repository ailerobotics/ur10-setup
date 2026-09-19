#!/usr/bin/env bash
# End-to-end proof (a): the gate driving the driver's MOCK HARDWARE (report §8,
# task brief item 4a). No Docker, no network, no real robot.
#
# Brings up our OWN ur_robot_driver in use_mock_hardware:=true on the isolated
# task domain 79, runs a harmless trajectory, replays scripted camera states, and
# lets the armed gate + independent backstop watcher supervise it. In mock the
# speed slider is inert (scaling stuck at 1.0), so a STOP cannot be effected by
# the slider and the gate MUST fall through to the BACKSTOP (cancel the goal,
# hold) — which is exactly the mock-appropriate behaviour to prove here.
#
# Everything this script starts is stopped by PID on exit (never by name). It
# starts its OWN driver; it never touches ur10-driver or any process it did not
# start.
export ROS_DOMAIN_ID=79
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LOGDIR="${LOGDIR:-$REPO/data/human-gate-sim-v1/mock}"
mkdir -p "$LOGDIR"
: > "$LOGDIR/driver.log"

PIDS=()
DRIVER_PID=""
cleanup() {
  echo "[run_mock_e2e] cleaning up (by PID; driver by process group so no child leaks)"
  # the driver launch is its own process group (setsid): signal the GROUP so its
  # child nodes are reaped too, not orphaned.
  [ -n "$DRIVER_PID" ] && kill -INT -"$DRIVER_PID" 2>/dev/null && sleep 3
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  [ -n "$DRIVER_PID" ] && kill -9 -"$DRIVER_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

echo "[run_mock_e2e] launching ur_robot_driver (use_mock_hardware:=true) on domain 79"
setsid ros2 launch ur_robot_driver ur10.launch.py \
    use_mock_hardware:=true launch_rviz:=false headless_mode:=true \
    robot_ip:=127.0.0.1 > "$LOGDIR/driver.log" 2>&1 &
DRIVER_PID=$!

echo "[run_mock_e2e] waiting for /joint_states + FJT action..."
ok=0
for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -q "/joint_states" && \
     ros2 action list 2>/dev/null | grep -q "follow_joint_trajectory"; then
    ok=1; break
  fi
  sleep 1
done
if [ "$ok" != "1" ]; then
  echo "[run_mock_e2e] FAILED: mock driver did not come up; see $LOGDIR/driver.log"
  tail -20 "$LOGDIR/driver.log"; exit 1
fi
echo "[run_mock_e2e] driver up."

echo "[run_mock_e2e] starting scripted camera-state publisher first (so the gate "
echo "               and watcher see a healthy heartbeat before any motion)"
python3 "$HERE/scripted_state_publisher.py" --ros-args \
    -p scenario:=full > "$LOGDIR/publisher.log" 2>&1 &
PUB_PID=$!
PIDS+=("$PUB_PID")
sleep 1

echo "[run_mock_e2e] starting armed gate + independent watcher"
python3 "$HERE/../gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$HERE/../backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:=127.0.0.1 \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)

echo "[run_mock_e2e] waiting for the gate to reach RUN (slider up) before moving..."
for i in $(seq 1 12); do
  if grep -q '"action": "RUN"' "$LOGDIR/decisions.jsonl" 2>/dev/null; then break; fi
  sleep 1
done

python3 "$HERE/trajectory_mover.py" --duration 40 --amp 0.5 \
    --log "$LOGDIR/mover.jsonl" --watch 80 > "$LOGDIR/mover.log" 2>&1 &
MOVER_PID=$!
PIDS+=("$MOVER_PID")

echo "[run_mock_e2e] running scenario (~46 s)..."
# wait for the publisher (scenario) to finish, up to a cap
for i in $(seq 1 60); do
  kill -0 "$PUB_PID" 2>/dev/null || break
  sleep 1
done
sleep 2

echo "[run_mock_e2e] scenario done; analysing"
python3 "$HERE/assert_e2e.py" --mode mock \
    --decisions "$LOGDIR/decisions.jsonl" \
    --mover "$LOGDIR/mover.jsonl" \
    --watcher "$LOGDIR/watcher.jsonl" | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
# keep a compact, committed transition log (the raw 20 Hz stream is gitignored)
python3 "$HERE/summarize_decisions.py" "$LOGDIR/decisions.jsonl" \
    "$LOGDIR/decisions_transitions.jsonl" >/dev/null 2>&1 || true
echo "[run_mock_e2e] logs in $LOGDIR"
exit $RC
