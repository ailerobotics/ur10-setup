#!/usr/bin/env bash
# Detect the UR robot generation over the dashboard server (port 29999).
# Writes the resolved ur_type to ur_type.txt next to this script.
#
# Discriminator: PolyscopeVersion major version.
#   3.x  -> CB3      -> ur_type=ur10
#   5.x+ -> e-Series -> ur_type=ur10e
set -uo pipefail
ROBOT_IP="${1:-192.168.0.1}"
HERE="$HOME/ur10"

echo "==> Pinging $ROBOT_IP"
if ! ping -c2 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "FAIL: $ROBOT_IP not reachable. Check link + pendant network settings."
  exit 1
fi
echo "OK: robot answers ping"

echo "==> Querying dashboard server (29999)"
resp=$( { sleep 0.7; printf 'PolyscopeVersion\n'; sleep 0.7; printf 'get robot model\n'; sleep 0.7; printf 'quit\n'; } \
        | timeout 8 nc "$ROBOT_IP" 29999 2>/dev/null )
echo "--- dashboard says ---"
echo "$resp"
echo "----------------------"

major=$(echo "$resp" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1 | cut -d. -f1)
case "${major:-unknown}" in
  3)     ur_type="ur10"  ; gen="CB3 (PolyScope 3.x)" ;;
  5|6|7) ur_type="ur10e" ; gen="e-Series (PolyScope ${major}.x)" ;;
  *)     echo "FAIL: could not parse PolyScope version from dashboard reply."; exit 2 ;;
esac

echo "$ur_type" > "$HERE/ur_type.txt"
echo "==> Detected: $gen  =>  ur_type=$ur_type  (saved to ur_type.txt)"

echo "==> Port reachability:"
for p in 29999 30001 30002 30003 30004; do
  if timeout 2 bash -c "true < /dev/tcp/$ROBOT_IP/$p" 2>/dev/null; then s=open; else s=CLOSED; fi
  echo "    $p: $s"
done
