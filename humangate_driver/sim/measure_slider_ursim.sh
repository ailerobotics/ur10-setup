#!/usr/bin/env bash
# Measure what the speed slider actually does to a MOVING arm on URSim CB3
# (task brief item 4: "Measure and report ... whether slider zero fully halts or a
# floor applies"). Brings URSim + the driver up on their own network/domain, runs
# one long trajectory, sweeps the slider 1.0 -> 0.5 -> 0.1 -> 0.0 -> 1.0 while
# sampling the applied scaling AND the actual joint motion, prints the table, and
# tears everything down (driver by process group, container + network removed).
#
# Result is written to the log dir; see SLIDER_ZERO_FINDING.md for the conclusion.
export ROS_DOMAIN_ID=79
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LOGDIR="${LOGDIR:-$REPO/data/human-gate-sim-v1/ursim}"; mkdir -p "$LOGDIR"
NET=hg_ursim_net; CONT=hg_ursim; URSIM_IP=192.168.98.2; IMAGE=universalrobots/ursim_cb3:3.15.8
LAUNCH_PID=""
cleanup(){ [ -n "$LAUNCH_PID" ] && kill -INT -"$LAUNCH_PID" 2>/dev/null && sleep 3
  [ -n "$LAUNCH_PID" ] && kill -9 -"$LAUNCH_PID" 2>/dev/null
  docker rm -f "$CONT" >/dev/null 2>&1; docker network rm "$NET" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
docker network inspect "$NET" >/dev/null 2>&1 || docker network create --subnet=192.168.98.0/24 "$NET" >/dev/null
docker rm -f "$CONT" >/dev/null 2>&1
docker run --rm -d --name "$CONT" --net "$NET" --ip "$URSIM_IP" -e ROBOT_MODEL=UR10 "$IMAGE" >/dev/null
until bash -c "exec 3<>/dev/tcp/$URSIM_IP/29999" 2>/dev/null; do sleep 2; done
python3 - "$URSIM_IP" <<'PY'
import socket,sys,time
s=socket.create_connection((sys.argv[1],29999),timeout=10);time.sleep(0.5)
try:s.recv(4096)
except:pass
def c(x):s.sendall((x+"\n").encode());time.sleep(0.5);return s.recv(4096)
c("power on");time.sleep(4);c("brake release");time.sleep(6);s.close()
PY
setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$URSIM_IP" \
  use_mock_hardware:=false headless_mode:=true launch_rviz:=false controller_spawner_timeout:=60 \
  > "$LOGDIR/measure_driver.log" 2>&1 &
LAUNCH_PID=$!
until ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling"; do sleep 2; done
sleep 3
timeout 120 python3 "$HERE/_slider_probe.py" | tee "$LOGDIR/slider_measurement.txt"
