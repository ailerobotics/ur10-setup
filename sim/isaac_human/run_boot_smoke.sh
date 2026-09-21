#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# BOOT SMOKE TEST orchestrator (task order-of-work step 1).
#
# Under the fleet GPU lock, boots Isaac Sim headless (env_isaaclab), publishes one
# RGB camera on the ROS 2 bridge, and — from a SEPARATE system-ROS process —
# confirms at least one frame arrives on domain 88. Stops Isaac by PID and
# reports whether the RTX renderer survived boot.
#
# SIMULATION ONLY. Domain 88, localhost only. Touches no robot, container or
# RealSense. Usage: sim/isaac_human/run_boot_smoke.sh
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK="${FLEET_GPU_LOCK:-$HOME/.cache/fleet-gpu.lock}"
OUTDIR="${ISAAC_HUMAN_OUT:-$HOME/.cache/isaac_human_sim}"
mkdir -p "${OUTDIR}"
ISAAC_LOG="${OUTDIR}/boot_smoke_isaac.log"
SECONDS_RUN="${SECONDS_RUN:-30}"

# shellcheck source=/dev/null
source "${HERE}/preflight.sh"

echo "[boot_smoke] preflight ..."
if ! preflight_check; then
  echo "[boot_smoke] preflight failed; not booting." >&2
  exit 3
fi

# Hold the fleet GPU lock for the WHOLE session via a dedicated fd.
exec 9>"${LOCK}"
echo "[boot_smoke] acquiring GPU lock ${LOCK} ..."
flock -w 3600 9 || { echo "[boot_smoke] could not acquire GPU lock in time." >&2; exit 4; }
echo "[boot_smoke] GPU lock acquired; booting Isaac (log: ${ISAAC_LOG})"

isaac_pid=""
wd_pid=""
cleanup() {
  [ -n "${wd_pid}" ] && kill -9 "${wd_pid}" 2>/dev/null || true
  if [ -n "${isaac_pid}" ] && kill -0 "${isaac_pid}" 2>/dev/null; then
    kill -INT "${isaac_pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do kill -0 "${isaac_pid}" 2>/dev/null || break; sleep 0.5; done
    kill -9 "${isaac_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Isaac side, backgrounded in its own process group.
(
  # shellcheck source=/dev/null
  source "${HERE}/isaac_env.sh"
  exec "${CONDA_PREFIX}/bin/python" "${HERE}/boot_smoke_test.py" --seconds "${SECONDS_RUN}"
) >"${ISAAC_LOG}" 2>&1 &
isaac_pid=$!
echo "[boot_smoke] Isaac pid ${isaac_pid}; waiting for it to publish ..."

# RAM watchdog (aborts Isaac if available RAM dips under ABORT_RAM_MB).
ram_watchdog "${isaac_pid}" &
wd_pid=$!

# Wait for Isaac to boot + Play + start streaming, or die.
booted=0
for _ in $(seq 1 90); do
  if ! kill -0 "${isaac_pid}" 2>/dev/null; then
    echo "[boot_smoke] Isaac process exited during boot — see ${ISAAC_LOG}." >&2
    break
  fi
  if grep -q "publishing RGB on" "${ISAAC_LOG}" 2>/dev/null; then
    booted=1
    break
  fi
  sleep 2
done

probe_rc=1
if [ "${booted}" = 1 ]; then
  echo "[boot_smoke] Isaac is publishing; probing from system ROS ..."
  set +e
  env -i HOME="${HOME}" PATH="/usr/bin:/bin" \
    bash -lc "source /opt/ros/jazzy/setup.bash; \
              export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
                     RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
                     FASTDDS_BUILTIN_TRANSPORTS=UDPv4; \
              python3 '${HERE}/ros_frame_probe.py' --topics /sim/smoke/rgb \
                     --seconds 20 --min 3"
  probe_rc=$?
  set -e
else
  echo "[boot_smoke] Isaac never reported publishing; treating as boot failure." >&2
fi

# cleanup() (trap) stops Isaac + watchdog by PID.
if [ "${probe_rc}" = 0 ]; then
  echo "[boot_smoke] PASS: Isaac booted headless and >=3 frames reached the ROS side."
  exit 0
fi
echo "[boot_smoke] FAIL: no frames confirmed on the ROS side (probe_rc=${probe_rc}); see ${ISAAC_LOG}." >&2
exit 1
