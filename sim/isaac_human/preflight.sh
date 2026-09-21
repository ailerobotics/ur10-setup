# SPDX-License-Identifier: Apache-2.0
# Preflight resource guard, sourced by every run wrapper BEFORE booting Isaac Sim.
#
# Enforces the brief's hard limits:
#   * >= MIN_GPU_FREE_MB free GPU memory (default 8192)
#   * >= MIN_RAM_MB available RAM (default 10240)
#   * the Blender application is NOT running (only detect; never kill). The
#     `blender-mcp` bridge server is not Blender and is ignored.
#
# Detection only — this never signals, kills or renices any process. Returns
# non-zero (via `return 1`) so the caller can decide to stop and report.

MIN_GPU_FREE_MB="${MIN_GPU_FREE_MB:-8192}"
MIN_RAM_MB="${MIN_RAM_MB:-10240}"

preflight_check() {
  local ok=0

  # Blender application running? (exact process name, not blender-mcp).
  if pgrep -x blender >/dev/null 2>&1; then
    echo "[preflight] BLOCKED: the Blender application is running — brief says do not close it. Stop and report." >&2
    ok=1
  fi

  # Free GPU memory (MiB). Take the first GPU.
  local gpu_free
  gpu_free="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')"
  if [ -z "${gpu_free}" ]; then
    echo "[preflight] BLOCKED: could not read GPU free memory from nvidia-smi." >&2
    ok=1
  elif [ "${gpu_free}" -lt "${MIN_GPU_FREE_MB}" ]; then
    echo "[preflight] BLOCKED: only ${gpu_free} MiB GPU free (< ${MIN_GPU_FREE_MB} MiB required)." >&2
    ok=1
  else
    echo "[preflight] GPU free: ${gpu_free} MiB (>= ${MIN_GPU_FREE_MB})."
  fi

  # Available RAM (MiB).
  local ram_avail
  ram_avail="$(free -m | awk '/^Mem:/ {print $7}')"
  if [ -z "${ram_avail}" ]; then
    echo "[preflight] BLOCKED: could not read available RAM from free." >&2
    ok=1
  elif [ "${ram_avail}" -lt "${MIN_RAM_MB}" ]; then
    echo "[preflight] BLOCKED: only ${ram_avail} MiB RAM available (< ${MIN_RAM_MB} MiB required)." >&2
    ok=1
  else
    echo "[preflight] RAM available: ${ram_avail} MiB (>= ${MIN_RAM_MB})."
  fi

  return "${ok}"
}

# A lightweight RAM watchdog the caller can background: aborts a target PID group
# if available RAM falls below ABORT_RAM_MB (default 3072). Never kills by name.
ram_watchdog() {
  local target_pid="$1"
  local abort_mb="${ABORT_RAM_MB:-3072}"
  while kill -0 "${target_pid}" 2>/dev/null; do
    local avail
    avail="$(free -m | awk '/^Mem:/ {print $7}')"
    if [ -n "${avail}" ] && [ "${avail}" -lt "${abort_mb}" ]; then
      echo "[preflight] RAM watchdog: only ${avail} MiB available (< ${abort_mb}); aborting PID ${target_pid}." >&2
      kill -INT "${target_pid}" 2>/dev/null || true
      sleep 5
      kill -9 "${target_pid}" 2>/dev/null || true
      return 0
    fi
    sleep 3
  done
}
