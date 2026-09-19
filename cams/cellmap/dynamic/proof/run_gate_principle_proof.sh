#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# PROOF 3 — the safety principle (report human-dynamic-plan-s1 §5): the dynamic
# map is a ROUTING aid, never the stop. Removing the dynamic layer's output
# changes routes but NEVER turns a STOP into motion.
#
# NO GPU, NO ROS, NO robot: pure offline assertion over the merged human-safety
# gate decision core + the acceptance point. Runs the routing_gate + decay + guard
# unit tests and records the result under the repo data dir.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DYN="$(cd "$HERE/.." && pwd)"
ROOT="$(cd "$DYN/../../.." && pwd)"
OUT="$ROOT/data/plan-nvblox-dynamic-v1/gate_principle_proof.log"
mkdir -p "$(dirname "$OUT")"

{
  echo "== gate-principle proof $(date -u +%FT%TZ) =="
  echo "-- routing_gate: map changes routes but never STOP->motion --"
  python3 "$DYN/tests/test_routing_gate.py"
  echo "-- decay model: fresh not missed, stale does not linger --"
  python3 "$DYN/tests/test_decay_model.py"
  echo "-- extrinsics guard: real camera refused until calibrated --"
  python3 "$DYN/tests/test_extrinsics_guard.py"
} 2>&1 | tee "$OUT"
RC=${PIPESTATUS[0]}
if [ "$RC" -eq 0 ]; then
  echo "RESULT: PASS — the map only re-routes; the gate alone starts/stops motion" | tee -a "$OUT"
else
  echo "RESULT: FAIL (rc=$RC)" | tee -a "$OUT"
fi
exit "$RC"
