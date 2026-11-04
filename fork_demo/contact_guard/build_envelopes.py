#!/usr/bin/env python3
"""Build leg-aware signal envelopes from the teach-002 bag.

Offline, no ROS. Reads the recorded mcap + teach metadata, segments the
continuous bag into legs by dwell window, builds a baseline+tolerance band per
leg/signal/progress, and writes a small committed yaml under ``envelopes/``.

Usage (run from the repo root, with the mcap venv's python so ``mcap`` imports):

    ~/venvs/contact-guard/bin/python -m fork_demo.contact_guard.build_envelopes \
        --run-dir /home/.../teach-002 \
        --out fork_demo/contact_guard/envelopes/teach-002.yaml

The bag itself is never copied into the repo; only the derived band yaml is.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys
from typing import List

import numpy as np
import yaml

# allow running as a script (python build_envelopes.py) or a module
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    from fork_demo.contact_guard import bagsource, envelope, params
else:
    from . import bagsource, envelope, params


def build(run_dir: str, bag_path: str, p: params.ContactGuardParams
          ) -> envelope.Envelope:
    legs = bagsource.load_leg_defs(run_dir)
    windows = bagsource.load_dwell_windows(run_dir)
    labels = bagsource.ordered_labels(run_dir)
    streams = bagsource.read_bag(bag_path)
    env = envelope.Envelope(provenance={
        "source_run_dir": run_dir,
        "source_bag": os.path.basename(bag_path),
        "built_utc": _dt.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
        "n_progress_bins": p.n_progress_bins,
        "band_k_sigma": p.band_k_sigma,
        "note": ("Freedrive teach record; track_err is not observable (no "
                 "commanded trajectory) so it is not banded here and is a "
                 "runtime-only signal. Bands are PROVISIONAL (one record)."),
    })
    for leg in legs:
        iv = bagsource.leg_interval(leg, windows, labels)
        if iv is None:
            print(f"  leg {leg.index:2d} {leg.name:18s} NO INTERVAL (skipped)")
            continue
        ts = bagsource.leg_timeseries(leg, streams, iv)
        travel = float(ts.feats.pop("_tcp_travel_m", np.zeros(1))[0])
        if ts.n < 2:
            print(f"  leg {leg.index:2d} {leg.name:18s} too few samples ({ts.n})")
            continue
        lb = envelope.build_leg_band(ts, leg.contact, p, tcp_travel_m=travel)
        env.legs[leg.name] = lb
        fm = lb.signals.get("force_mag")
        print(f"  leg {leg.index:2d} {leg.name:18s} "
              f"contact={str(leg.contact):5s} n={ts.n:5d} "
              f"dur={ts.duration_s:6.1f}s travel={travel*1000:6.1f}mm "
              f"|F| {fm.baseline.min():.0f}-{fm.baseline.max():.0f}N "
              f"(hi {fm.hi.max():.0f})")
    return env


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", default=bagsource.TEACH_DIR_DEFAULT)
    ap.add_argument("--bag", default=None,
                    help="mcap path (default: <run-dir>/bag/bag_0.mcap)")
    ap.add_argument("--params", default=params.CONFIG_PATH)
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "envelopes", "teach-002.yaml"))
    a = ap.parse_args(argv)
    bag = a.bag or os.path.join(a.run_dir, "bag", "bag_0.mcap")
    p = params.load(a.params)
    print(f"building envelopes from {bag}")
    env = build(a.run_dir, bag, p)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as fh:
        yaml.safe_dump(env.to_dict(), fh, sort_keys=False, default_flow_style=False)
    size = os.path.getsize(a.out)
    print(f"wrote {a.out} ({size/1024:.1f} KiB, {len(env.legs)} legs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
