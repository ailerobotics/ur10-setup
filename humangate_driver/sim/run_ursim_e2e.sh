#!/usr/bin/env bash
# End-to-end proof (b): the gate driving the Universal Robots SIMULATOR (URSim)
# for the CB3 series in Docker, with the real ur_robot_driver connected to it
# (report §8, task brief item 4b). This is the rig that can show the SPEED-SCALING
# primary lever, which mock hardware cannot (mock scaling is inert).
#
# Isolation per the task brief:
#   * URSim runs on its OWN docker network (hg_ursim_net, 192.168.98.0/24), no GPU,
#     reachable only via its docker-internal IP 192.168.98.2 (no ports published to
#     any physical interface);
#   * the gate/watcher are told robot_ip=192.168.98.2 — a Docker-internal address
#     the arming interlock accepts, and which is NOT 192.168.0.1 nor on a physical
#     NIC, so this can never reach real hardware;
#   * everything runs on ROS_DOMAIN_ID=79.
# On exit it stops every process it started by PID (SIGINT to the driver launch so
# it tears down its own children) and REMOVES its own container + network. It
# never touches containers/processes it did not create.
#
# If the URSim image is unavailable (needs sudo/login), do the mock proof instead
# and stop here (task hard rule). This script assumes the image is already pulled.

export ROS_DOMAIN_ID=79
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LOGDIR="${LOGDIR:-$REPO/data/human-gate-sim-v1/ursim}"
mkdir -p "$LOGDIR"

NET=hg_ursim_net
SUBNET=192.168.98.0/24
URSIM_IP=192.168.98.2
CONT=hg_ursim
IMAGE=universalrobots/ursim_cb3:3.15.8

PIDS=()
LAUNCH_PID=""
cleanup() {
  echo "[run_ursim_e2e] cleanup (by PID; driver by process group; own container/network only)"
  # driver launch is its own process group (setsid): signal the GROUP so its child
  # nodes are reaped too, not orphaned (a leaked driver pollutes DDS discovery).
  [ -n "$LAUNCH_PID" ] && kill -INT -"$LAUNCH_PID" 2>/dev/null && sleep 4
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  [ -n "$LAUNCH_PID" ] && kill -9 -"$LAUNCH_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  docker rm -f "$CONT" >/dev/null 2>&1
  docker network rm "$NET" >/dev/null 2>&1
  echo "[run_ursim_e2e] removed container $CONT and network $NET"
}
trap cleanup EXIT INT TERM

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[run_ursim_e2e] BLOCKED: image $IMAGE not present. Pull it or run the mock proof."
  exit 3
fi

docker network inspect "$NET" >/dev/null 2>&1 || \
  docker network create --subnet="$SUBNET" "$NET" >/dev/null
docker rm -f "$CONT" >/dev/null 2>&1
echo "[run_ursim_e2e] starting URSim ($IMAGE) at $URSIM_IP (no GPU, own network)"
docker run --rm -d --name "$CONT" --net "$NET" --ip "$URSIM_IP" \
  -e ROBOT_MODEL=UR10 "$IMAGE" >/dev/null

echo "[run_ursim_e2e] waiting for the simulator dashboard + RTDE..."
for i in $(seq 1 60); do
  if bash -c "exec 3<>/dev/tcp/$URSIM_IP/29999" 2>/dev/null && \
     bash -c "exec 3<>/dev/tcp/$URSIM_IP/30004" 2>/dev/null; then break; fi
  sleep 2
done

echo "[run_ursim_e2e] powering on + releasing brakes"
python3 - "$URSIM_IP" <<'PY'
import socket, sys, time
ip = sys.argv[1]
def cmd(s, c):
    s.sendall((c+"\n").encode()); time.sleep(0.5); return s.recv(4096).decode(errors="replace").strip()
s = socket.create_connection((ip, 29999), timeout=10); time.sleep(0.5)
try: s.recv(4096)
except Exception: pass
cmd(s, "power on"); time.sleep(4); cmd(s, "brake release"); time.sleep(6)
print("  robotmode:", cmd(s, "robotmode"), "| safetymode:", cmd(s, "safetymode"))
s.close()
PY

echo "[run_ursim_e2e] launching ur_robot_driver -> $URSIM_IP (headless, spawner timeout 60)"
setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$URSIM_IP" \
    use_mock_hardware:=false headless_mode:=true launch_rviz:=false \
    controller_spawner_timeout:=60 > "$LOGDIR/driver.log" 2>&1 &
LAUNCH_PID=$!

echo "[run_ursim_e2e] waiting for /speed_scaling_state_broadcaster/speed_scaling..."
ok=0
for i in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling"; then ok=1; break; fi
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[run_ursim_e2e] speed_scaling not up via launch; spawning active controllers as fallback"
  ros2 run controller_manager spawner \
     joint_state_broadcaster io_and_status_controller speed_scaling_state_broadcaster \
     force_torque_sensor_broadcaster scaled_joint_trajectory_controller \
     --controller-manager /controller_manager --controller-manager-timeout 30 \
     > "$LOGDIR/spawn.log" 2>&1
  for i in $(seq 1 15); do
    ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling" && ok=1 && break
    sleep 1
  done
fi
if [ "$ok" != "1" ]; then
  echo "[run_ursim_e2e] FAILED: speed scaling broadcaster never came up"; tail -20 "$LOGDIR/driver.log"; exit 1
fi
echo "[run_ursim_e2e] driver + broadcasters up."

echo "[run_ursim_e2e] starting publisher, gate, watcher (robot_ip=$URSIM_IP, docker-internal)"
python3 "$HERE/scripted_state_publisher.py" --ros-args -p scenario:=full \
    > "$LOGDIR/publisher.log" 2>&1 &
PUB_PID=$!; PIDS+=("$PUB_PID")
sleep 1
python3 "$HERE/../gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:="$URSIM_IP" \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$HERE/../backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:="$URSIM_IP" \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)

echo "[run_ursim_e2e] waiting for gate RUN before moving..."
for i in $(seq 1 15); do
  grep -q '"action": "RUN"' "$LOGDIR/decisions.jsonl" 2>/dev/null && break
  sleep 1
done
python3 "$HERE/trajectory_mover.py" --duration 40 --amp 0.6 \
    --log "$LOGDIR/mover.jsonl" --watch 90 > "$LOGDIR/mover.log" 2>&1 &
MOVER_PID=$!; PIDS+=("$MOVER_PID")

echo "[run_ursim_e2e] running scenario (~46 s)..."
for i in $(seq 1 70); do kill -0 "$PUB_PID" 2>/dev/null || break; sleep 1; done
sleep 2

echo "[run_ursim_e2e] analysing"
python3 "$HERE/assert_e2e.py" --mode ursim \
    --decisions "$LOGDIR/decisions.jsonl" --mover "$LOGDIR/mover.jsonl" \
    --watcher "$LOGDIR/watcher.jsonl" | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
python3 "$HERE/summarize_decisions.py" "$LOGDIR/decisions.jsonl" \
    "$LOGDIR/decisions_transitions.jsonl" >/dev/null 2>&1 || true
echo "[run_ursim_e2e] logs in $LOGDIR"
exit $RC
