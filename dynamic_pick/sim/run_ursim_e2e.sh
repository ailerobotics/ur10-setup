#!/usr/bin/env bash
# Dynamic-pick end-to-end proof (b): the segmented re-plan/preempt controller
# driving the Universal Robots CB3 SIMULATOR (URSim) in Docker, with the real
# ur_robot_driver connected to it, gated by the merged human-safety gate
# (report §7, §10 Phase A; task brief item 4). This is the rig that exercises the
# EFFECTIVE speed slider (mock cannot): SLOW actually slows the running leg and the
# command-to-effect delay is measurable.
#
# Isolation per the task brief:
#   * URSim runs on its OWN docker network (fm-hdps-ursim-net, 192.168.99.0/24),
#     no GPU, reachable only via its docker-internal IP (no ports published);
#   * container/network names carry the task prefix; nothing named ur10-* or
#     shelfbot_ros; the arming interlock accepts the docker-internal IP and can
#     never reach 192.168.0.1 or a physical NIC;
#   * everything runs on ROS_DOMAIN_ID=83.
# On exit it stops every process it started BY PID and REMOVES its OWN container +
# network. It never touches containers/processes it did not create.
export ROS_DOMAIN_ID=83
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$PKG/.." && pwd)"
GATE="$REPO/humangate_driver"
LOGDIR="${LOGDIR:-$REPO/data/human-dynamic-pick-sim-v1/ursim}"
mkdir -p "$LOGDIR"

NET=fm-hdps-ursim-net
SUBNET=192.168.99.0/24
URSIM_IP=192.168.99.2
CONT=fm-human-dynamic-pick-sim-v1-ursim
IMAGE=universalrobots/ursim_cb3:3.15.8

PIDS=(); LAUNCH_PID=""
cleanup() {
  echo "[dp-ursim] cleanup (by PID; driver by process group; own container/network only)"
  [ -n "$LAUNCH_PID" ] && kill -INT -"$LAUNCH_PID" 2>/dev/null && sleep 4
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  [ -n "$LAUNCH_PID" ] && kill -9 -"$LAUNCH_PID" 2>/dev/null
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  docker rm -f "$CONT" >/dev/null 2>&1
  docker network rm "$NET" >/dev/null 2>&1
  echo "[dp-ursim] removed container $CONT and network $NET"
}
trap cleanup EXIT INT TERM

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "[dp-ursim] BLOCKED: image $IMAGE not present. Run the mock proof instead."; exit 3; }

docker network inspect "$NET" >/dev/null 2>&1 || \
  docker network create --subnet="$SUBNET" "$NET" >/dev/null
docker rm -f "$CONT" >/dev/null 2>&1
echo "[dp-ursim] starting URSim at $URSIM_IP (no GPU, own network)"
docker run --rm -d --name "$CONT" --net "$NET" --ip "$URSIM_IP" \
  -e ROBOT_MODEL=UR10 "$IMAGE" >/dev/null

echo "[dp-ursim] waiting for the simulator dashboard + RTDE..."
for i in $(seq 1 60); do
  if bash -c "exec 3<>/dev/tcp/$URSIM_IP/29999" 2>/dev/null && \
     bash -c "exec 3<>/dev/tcp/$URSIM_IP/30004" 2>/dev/null; then break; fi
  sleep 2
done

echo "[dp-ursim] powering on + releasing brakes"
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

echo "[dp-ursim] launching ur_robot_driver -> $URSIM_IP (headless)"
setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$URSIM_IP" \
    use_mock_hardware:=false headless_mode:=true launch_rviz:=false \
    controller_spawner_timeout:=60 > "$LOGDIR/driver.log" 2>&1 &
LAUNCH_PID=$!

echo "[dp-ursim] waiting for /speed_scaling_state_broadcaster/speed_scaling..."
ok=0
for i in $(seq 1 50); do
  ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling" && ok=1 && break
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[dp-ursim] spawning active controllers as fallback"
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
[ "$ok" = "1" ] || { echo "[dp-ursim] FAILED: speed scaling never came up"; tail -20 "$LOGDIR/driver.log"; exit 1; }
echo "[dp-ursim] driver + broadcasters up; waiting for /joint_states..."
for i in $(seq 1 30); do
  ros2 topic list 2>/dev/null | grep -q "^/joint_states$" && break
  sleep 1
done

echo "[dp-ursim] sim bootstrap: parking the arm at the in-window reference pose"
python3 "$HERE/prepose.py" --ros-args \
    -p limits:="$REPO/cumotion/measured_joint_limits.yaml" > "$LOGDIR/prepose.log" 2>&1
cat "$LOGDIR/prepose.log"

echo "[dp-ursim] starting gate + watcher (sole motion authority)"
python3 "$GATE/gate_driver_node.py" --ros-args \
    -p arm:=true -p robot_ip:="$URSIM_IP" \
    -p decision_log:="$LOGDIR/decisions.jsonl" > "$LOGDIR/gate.log" 2>&1 &
PIDS+=($!)
python3 "$GATE/backstop_watcher_node.py" --ros-args \
    -p arm:=true -p robot_ip:="$URSIM_IP" \
    -p watcher_log:="$LOGDIR/watcher.jsonl" > "$LOGDIR/watcher.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[dp-ursim] starting human states + moving target + controller (aligned clocks)"
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

echo "[dp-ursim] running scenario (~58 s)..."
for i in $(seq 1 70); do kill -0 "$HUMAN_PID" 2>/dev/null || break; sleep 1; done
sleep 3

echo "[dp-ursim] analysing"
python3 "$HERE/assert_dynamic_pick.py" --mode ursim \
    --pick "$LOGDIR/pick.jsonl" --decisions "$LOGDIR/decisions.jsonl" \
    | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[dp-ursim] logs in $LOGDIR"
exit $RC
