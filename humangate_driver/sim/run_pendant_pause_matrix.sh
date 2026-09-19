#!/usr/bin/env bash
# Run the full pendant-pause measurement matrix the pendant-pause task needs:
# durability (5 s / 60 s / 10 min holds) + every failure case, each as its OWN
# FRESH URSim bring-up so no scenario can leave residual state that corrupts the
# next (back-to-back goals on one URSim session can wedge the External Control
# program). Each row is just the canonical single-scenario harness
# (investigate_pause_ursim.sh MODE=pendant), so there is zero bring-up duplication.
#
# Everything runs on ROS_DOMAIN_ID=82 on URSim's own docker network; each row tears
# its container/network/driver down by PID before the next. Per-row logs land in
# data/human-gate-sim-v1/ursim-pendant-pause/<row>/ and a matrix summary is written
# at the end. Self-asserting: exits non-zero if any row's assertion failed.
#
# Rows (ACTION:PAUSE_HOLD):
#   pause_play:5      freeze-and-continue, short hold
#   pause_play:60     freeze-and-continue, 60 s hold (durability)
#   pause_play:600    freeze-and-continue, 10 min hold (durability) — LONG (~13 min)
#   pause_only:20     pause and never resume (gate-dies-while-paused shape): stays stopped
#   foreign_play:5    play pressed while STOP still wanted: motion resumes (=> re-pause)
#   stop_instead:5    dashboard STOP not pause: terminal, no same-goal continue
#   pause_no_program:5 pause when nothing runs: rejected, arm keeps moving
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
BASE="$REPO/data/human-gate-sim-v1/ursim-pendant-pause"
mkdir -p "$BASE"
SUMMARY="$BASE/MATRIX_SUMMARY.txt"
: > "$SUMMARY"

# Default matrix; override with MATRIX="pause_play:5 stop_instead:5" for a subset.
MATRIX="${MATRIX:-pause_play:5 pause_play:60 pause_play:600 pause_only:20 foreign_play:5 stop_instead:5 pause_no_program:5}"

overall=0
for row in $MATRIX; do
  action="${row%%:*}"; hold="${row##*:}"
  rowdir="$BASE/${action}_${hold}s"
  mkdir -p "$rowdir"
  echo "==== MATRIX ROW: action=$action pause_hold=${hold}s -> $rowdir ===="
  MODE=pendant ACTION="$action" PAUSE_HOLD="$hold" LOGDIR="$rowdir" \
    bash "$HERE/investigate_pause_ursim.sh" > "$rowdir/harness.log" 2>&1
  rc=$?
  verdict=$(grep -m1 -E 'checks passed|RE-EVALUATE|BLOCKED|FAILED' "$rowdir/harness.log" 2>/dev/null | tail -1)
  printf '%-22s rc=%-2s %s\n' "$action:${hold}s" "$rc" "$verdict" | tee -a "$SUMMARY"
  # A failure-case row (foreign_play/pause_no_program/pause_only/stop_instead) whose
  # assertion passes returns 0; pause_play returns 0 only if freeze-and-continue held.
  [ "$rc" -ne 0 ] && overall=1
done
echo "==== matrix done (overall rc=$overall). Summary: $SUMMARY ===="
exit $overall
