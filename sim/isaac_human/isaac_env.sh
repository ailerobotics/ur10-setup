# SPDX-License-Identifier: Apache-2.0
# Shared environment for every Isaac Sim invocation in this harness.
#
# SIMULATION ONLY. Isolated ROS_DOMAIN_ID=88, ROS_LOCALHOST_ONLY=1, FastDDS UDPv4.
# Nothing here reaches the real robot, its network (192.168.0.1), the RealSense
# cameras or the ur10-*/shelfbot_ros containers.
#
# `source` this from a wrapper that has ALREADY taken the fleet GPU lock. It sets
# the conda env (env_isaaclab, Isaac Sim 5.1) and the bundled-ROS2-Jazzy DDS env
# the Isaac ros2 bridge needs to interoperate with the system-ROS detector graph.
# It does NOT run python — the caller execs the conda python explicitly.
#
# Pattern mirrors $HOME/workspaces/shelfbot/tools/run_tray_scene.sh
# (read-only reference), retargeted to this user's home and to the task domain.

# Refuse the shared/live ROS domains and default to the SIM-only domain 88,
# BEFORE any heavy setup (conda, Isaac libs), so an ambient ROS_DOMAIN_ID can
# never point this harness at the real robot / shelfbot / live-camera graphs.
# Standalone + unit-tested (tests/test_domain_guard.py). Returns non-zero (which,
# under the wrappers' `set -e`, aborts) whether this file is sourced or executed.
# shellcheck source=/dev/null
source "$(dirname "${BASH_SOURCE[0]}")/domain_guard.sh" || return 3 2>/dev/null || exit 3

CONDA_ROOT="${CONDA_ROOT:-$HOME/miniforge3}"
CONDA_ENV="${CONDA_ENV:-env_isaaclab}"

# shellcheck source=/dev/null
source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}"

# Bundled ROS 2 (Jazzy) shipped inside the Isaac Sim ros2 bridge extension. The
# detector graph uses the SYSTEM Jazzy (/opt/ros/jazzy); both are Jazzy +
# rmw_fastrtps_cpp + FastDDS UDPv4 on the same domain, so they interoperate over
# DDS. We deliberately use Isaac's bundled libs here (never source system ROS in
# the Isaac process — mixing the two ABIs is the classic bridge crash).
JAZZY="${CONDA_PREFIX}/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy"

export ROS_DISTRO=jazzy
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}" # never leave this host
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export LD_LIBRARY_PATH="${JAZZY}/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${JAZZY}/rclpy:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="${JAZZY}:${AMENT_PREFIX_PATH:-}"

# Keep Isaac's caches/logs OUT of git and off the small root fs.
export ISAAC_CACHE_DIR="${ISAAC_CACHE_DIR:-$HOME/.cache/isaac_human_sim}"
mkdir -p "${ISAAC_CACHE_DIR}"
