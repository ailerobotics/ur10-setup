#!/usr/bin/env bash
# Investigate whether dashboard PAUSE/PLAY freezes-and-continues a running
# scaled-JTC trajectory on a HEADLESS CB3 driver (this cell runs the driver
# headless — ur-bringup.sh: headless_mode:=true). This answers the captain's
# "pause and resume the robot program" question empirically on URSim, alongside
# the failure cases the design flagged (does PAUSE, like STOP, drop the driver
# connection?).
#
# Isolation (task brief): URSim on its OWN docker network, no GPU, reachable only
# via its docker-internal IP; everything on ROS_DOMAIN_ID=81; teardown by PID +
# own container/network only. It never touches ur10-driver or any process or
# container it did not create.
#
# Output: data/human-gate-sim-v1/ursim-pause/pause_investigation.txt (the probe's
# RESULT json + the driver log tail), consumed by SLIDER_ZERO_FINDING's sibling
# finding file.
set -o pipefail
export ROS_DOMAIN_ID=81
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LOGDIR="${LOGDIR:-$REPO/data/human-gate-sim-v1/ursim-pause}"
mkdir -p "$LOGDIR"

NET=hg_pause_net
SUBNET=192.168.99.0/24
URSIM_IP=192.168.99.2
CONT=hg_pause_ursim
IMAGE=universalrobots/ursim_cb3:3.15.8

LAUNCH_PID=""
cleanup() {
  echo "[investigate_pause] cleanup (driver by process group; own container/network only)"
  [ -n "$LAUNCH_PID" ] && kill -INT -"$LAUNCH_PID" 2>/dev/null && sleep 4
  [ -n "$LAUNCH_PID" ] && kill -9 -"$LAUNCH_PID" 2>/dev/null
  docker rm -f "$CONT" >/dev/null 2>&1
  docker network rm "$NET" >/dev/null 2>&1
  echo "[investigate_pause] removed container $CONT and network $NET"
}
trap cleanup EXIT INT TERM

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[investigate_pause] BLOCKED: image $IMAGE not present."; exit 3
fi

docker network inspect "$NET" >/dev/null 2>&1 || \
  docker network create --subnet="$SUBNET" "$NET" >/dev/null
docker rm -f "$CONT" >/dev/null 2>&1
echo "[investigate_pause] starting URSim ($IMAGE) at $URSIM_IP (no GPU, own network)"
docker run --rm -d --name "$CONT" --net "$NET" --ip "$URSIM_IP" \
  -e ROBOT_MODEL=UR10 "$IMAGE" >/dev/null

echo "[investigate_pause] waiting for the simulator dashboard + RTDE..."
for i in $(seq 1 60); do
  if bash -c "exec 3<>/dev/tcp/$URSIM_IP/29999" 2>/dev/null && \
     bash -c "exec 3<>/dev/tcp/$URSIM_IP/30004" 2>/dev/null; then break; fi
  sleep 2
done

echo "[investigate_pause] powering on + releasing brakes"
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

echo "[investigate_pause] launching ur_robot_driver -> $URSIM_IP (HEADLESS, matches ur-bringup.sh)"
setsid ros2 launch ur_robot_driver ur10.launch.py robot_ip:="$URSIM_IP" \
    use_mock_hardware:=false headless_mode:=true launch_rviz:=false \
    controller_spawner_timeout:=60 > "$LOGDIR/driver.log" 2>&1 &
LAUNCH_PID=$!

echo "[investigate_pause] waiting for /speed_scaling_state_broadcaster/speed_scaling..."
ok=0
for i in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -q "/speed_scaling_state_broadcaster/speed_scaling"; then ok=1; break; fi
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[investigate_pause] FAILED: speed scaling broadcaster never came up"; tail -30 "$LOGDIR/driver.log"; exit 1
fi
echo "[investigate_pause] driver + broadcasters up; dashboard services:"
ros2 service list 2>/dev/null | grep -i dashboard | tee "$LOGDIR/dashboard_services.txt"
sleep 3

echo "[investigate_pause] running pause/resume probe..."
timeout 150 python3 "$HERE/_pause_probe.py" --pause-hold 8 --duration 40 --amp 2.0 \
    2>"$LOGDIR/probe.err" | tee "$LOGDIR/pause_investigation.txt"
PRC=${PIPESTATUS[0]}
echo "[investigate_pause] --- driver.log tail ---" >> "$LOGDIR/pause_investigation.txt"
tail -40 "$LOGDIR/driver.log" >> "$LOGDIR/pause_investigation.txt"

echo "[investigate_pause] analysing (assert the finding still holds)"
python3 "$HERE/assert_pause_investigation.py" --file "$LOGDIR/pause_investigation.txt" \
    | tee "$LOGDIR/ASSERTIONS.txt"
RC=${PIPESTATUS[0]}
echo "[investigate_pause] logs in $LOGDIR (probe rc=$PRC, assert rc=$RC)"
exit $RC
