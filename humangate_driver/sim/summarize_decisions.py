#!/usr/bin/env python3
"""Compress a raw 20 Hz gate decision log into an action-transition log — the
compact, human-readable evidence kept in the repo (the full 20 Hz stream is
regenerable by re-running the e2e script and is gitignored).

    python3 summarize_decisions.py <decisions.jsonl> <decisions_transitions.jsonl>
"""
import json
import sys

KEEP = ("t_mono", "action", "demand", "provenance", "speed_fraction",
        "observed_scaling", "cam_zone", "cam_health_ok", "reason")


def main():
    src, dst = sys.argv[1], sys.argv[2]
    prev = None
    n = 0
    with open(src) as f, open(dst, "w") as out:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            n += 1
            key = (r.get("action"), r.get("demand"), r.get("provenance"))
            if key != prev:
                out.write(json.dumps({k: r.get(k) for k in KEEP}) + "\n")
                prev = key
    print(f"{src}: {n} ticks -> {dst}")


if __name__ == "__main__":
    main()
