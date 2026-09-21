# SPDX-License-Identifier: Apache-2.0
# ROS_DOMAIN_ID guard for the Isaac human-detection harness.
#
# SIM ONLY. This harness must never share a graph with the real robot (domain 0),
# the shelfbot_ros container (baked domain 42), or the LIVE human_safety camera
# layer (domain 77 — the one graph allowed to open the real RealSense cameras).
# Default to the task-only domain 88 and REFUSE those three, so an ambient
# ROS_DOMAIN_ID inherited from the shell can never cross the harness into a live
# graph. Kept standalone (sourced first by isaac_env.sh) so it is unit-testable
# with no conda / Isaac dependency; fails loud whether sourced or executed.
_isaac_dom="${ROS_DOMAIN_ID:-88}"
case "${_isaac_dom}" in
    0|42|77)
        echo "isaac domain guard: REFUSING ROS_DOMAIN_ID=${_isaac_dom} — domains" \
             "0/42/77 are the real-robot / shelfbot_ros / live-camera graphs;" \
             "this harness is SIM ONLY and defaults to domain 88." >&2
        unset _isaac_dom
        return 3 2>/dev/null || exit 3
        ;;
esac
export ROS_DOMAIN_ID="${_isaac_dom}"
unset _isaac_dom
