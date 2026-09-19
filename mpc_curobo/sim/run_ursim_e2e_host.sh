#!/usr/bin/env bash
# HOST wrapper: prove the gate-mediated MPC command stream against the Universal
# Robots CB3 SIMULATOR (URSim), SIM ONLY. Starts URSim in its own fm- container
# (no ports published to any physical interface), then runs THIS task's container
# sharing URSim's network namespace so the UR driver reaches URSim over LOOPBACK
# (robot_ip=127.0.0.1) — which the arming interlock accepts as loopback, never a
# physical NIC or 192.168.0.1. Reuses run_mock_e2e.sh with MODE=ursim.
#
# Removes ONLY the containers it creates (both fm-plan-mpc-curobo-v1-*). Never
# touches containers it did not create. No GPU needed (scripted MPC source).
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA=/home/manishsaini/firstmate/data/plan-mpc-curobo-v1
IMAGE=fm-plan-mpc-curobo-v1:latest
URSIM_IMAGE=universalrobots/ursim_cb3:3.15.8
URSIM=fm-plan-mpc-curobo-v1-ursim
RUNNER=fm-plan-mpc-curobo-v1-ursim-run
mkdir -p "$DATA/runs"

cleanup() {
    docker rm -f "$RUNNER" >/dev/null 2>&1
    docker rm -f "$URSIM"  >/dev/null 2>&1
    echo "[ursim-host] removed $RUNNER and $URSIM"
}
trap cleanup EXIT

if ! docker image inspect "$URSIM_IMAGE" >/dev/null 2>&1; then
    echo "[ursim-host] BLOCKED: $URSIM_IMAGE not present"; exit 3
fi

echo "[ursim-host] starting URSim CB3 (UR10, own container, no published ports)"
docker rm -f "$URSIM" >/dev/null 2>&1
docker run --rm -d --name "$URSIM" -e ROBOT_MODEL=UR10 "$URSIM_IMAGE" >/dev/null
echo "[ursim-host] waiting ~40 s for URSim to boot..."
sleep 40

echo "[ursim-host] running driver+bridge in $RUNNER (shares URSim net namespace)"
docker run --rm --name "$RUNNER" --network "container:$URSIM" \
    --user "$(id -u):$(id -g)" -e HOME=/tmp -e MODE=ursim \
    -v "$REPO":/repo:ro -v "$DATA":/out:rw \
    "$IMAGE" bash -lc 'bash /repo/mpc_curobo/sim/run_mock_e2e.sh > /out/runs/ursim_e2e.log 2>&1'
echo "[ursim-host] result:"
grep -E "PASS|FAIL|E2E RESULT" "$DATA/runs/ursim_e2e.log" | tail -5
