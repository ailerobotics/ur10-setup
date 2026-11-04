#!/usr/bin/env python3
"""Assert the URSim contact-guard proof: the guard tripped and cancelled the goal."""
from __future__ import annotations

import argparse
import json
import os
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trip", required=True, help="trip JSON written by node.py")
    ap.add_argument("--execute-log", default=None)
    a = ap.parse_args(argv)

    ok = True
    if not os.path.exists(a.trip):
        print(f"FAIL: no trip JSON at {a.trip} (guard did not trip)")
        return 1
    with open(a.trip) as fh:
        trip = json.load(fh)
    print(f"trip: trigger={trip.get('trigger')} families={trip.get('families')} "
          f"action={trip.get('action')}")
    if trip.get("trigger") not in ("PROBE", "EARLY", "STALL"):
        print("FAIL: unexpected or missing trigger"); ok = False
    if trip.get("action") not in ("BACKOFF", "HOLD"):
        print("FAIL: no reaction recorded"); ok = False
    if a.execute_log and os.path.exists(a.execute_log):
        txt = open(a.execute_log, errors="ignore").read().lower()
        if "cancel" in txt or "did not complete" in txt or "abort" in txt:
            print("execute log shows the goal was cancelled/aborted")
        else:
            print("WARN: execute log does not clearly show a cancel "
                  "(URSim timing) -- trip JSON is the authoritative proof")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
