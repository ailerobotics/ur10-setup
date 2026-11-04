#!/usr/bin/env python3
"""Contact-guard CLI: build envelopes, guard dry-run over a bag, fault report.

    python -m fork_demo.contact_guard.cli dry-run [--bag B] [--run-dir D]
    python -m fork_demo.contact_guard.cli inject [--json]
    python -m fork_demo.contact_guard.cli build  [--out ...]

``dry-run`` is the offline proof over the teach-002 bag (zero false trips);
``inject`` runs every injected fault + the named near-miss negatives and prints
the latency / force-margin table. ``dry-run`` needs the ``mcap`` venv python so
the bag can be read; ``inject`` runs on pure python (it uses only the committed
envelope).
"""

from __future__ import annotations

import argparse
import json
import os
import sys

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))

import yaml  # noqa: E402

from fork_demo.contact_guard import (  # noqa: E402
    bagsource, dryrun, envelope, faults, params,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ENV_DEFAULT = os.path.join(HERE, "envelopes", "teach-002.yaml")


def _load_env(path: str) -> envelope.Envelope:
    with open(path) as fh:
        return envelope.Envelope.from_dict(yaml.safe_load(fh))


def cmd_dry_run(a) -> int:
    p = params.load(a.params)
    env = _load_env(a.env)
    bag = a.bag or os.path.join(a.run_dir, "bag", "bag_0.mcap")
    rep = dryrun.dry_run_bag(a.run_dir, bag, p, env)
    print(rep.text())
    if rep.n_trips > 0:
        print("FALSE TRIP(S) on the clean record -- guard too hot", file=sys.stderr)
        return 1
    return 0


def cmd_inject(a) -> int:
    p = params.load(a.params)
    env = _load_env(a.env)
    rep = faults.fault_report(env, p)
    if a.json:
        print(json.dumps(rep, indent=2))
    else:
        print("INJECTED FAULTS (all must be detected):")
        for r in rep["faults"]:
            print(f"  {r['name']:18s} {r['leg']:16s} det={r['detected']!s:5s} "
                  f"[{str(r['trigger']):9s}] +{r['mm_after_onset']:6.1f}mm "
                  f"+{r['ms_after_onset']:5.0f}ms  |F|@trip={r['force_mag_at_trip_n']:5.1f}N "
                  f"(<200N by {r['margin_to_200n']:.0f})")
        print("STOP-AND-PROBE touch (fresh at-pose baseline, 6 N tip):")
        for r in rep["probe"]:
            print(f"  {r['leg']:16s} det={r['detected']!s:5s} tip~{r['tip_force_n']}N "
                  f"+{r['ms_after_onset']:.0f}ms  (<pendant 100N by {r['margin_to_pendant_n']:.0f})")
        print("NAMED NEGATIVES (must NOT trip):")
        for r in rep["negatives"]:
            print(f"  {r['name']:22s} {r['leg']:16s} tripped={r['detected']}")
        print(f"ALL DETECTED: {rep['all_detected']}  "
              f"ANY NEGATIVE TRIPPED: {rep['any_negative_tripped']}")
    ok = rep["all_detected"] and not rep["any_negative_tripped"]
    return 0 if ok else 1


def cmd_build(a) -> int:
    from fork_demo.contact_guard import build_envelopes
    return build_envelopes.main(
        ["--run-dir", a.run_dir, "--out", a.out, "--params", a.params]
        + (["--bag", a.bag] if a.bag else []))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=params.CONFIG_PATH)
    ap.add_argument("--env", default=ENV_DEFAULT)
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("dry-run", help="guard dry-run over a bag (zero false trips)")
    d.add_argument("--bag", default=None)
    d.add_argument("--run-dir", default=bagsource.TEACH_DIR_DEFAULT)
    d.set_defaults(func=cmd_dry_run)

    i = sub.add_parser("inject", help="injected-fault + negatives report")
    i.add_argument("--json", action="store_true")
    i.set_defaults(func=cmd_inject)

    b = sub.add_parser("build", help="(re)build envelopes from a bag")
    b.add_argument("--bag", default=None)
    b.add_argument("--run-dir", default=bagsource.TEACH_DIR_DEFAULT)
    b.add_argument("--out", default=ENV_DEFAULT)
    b.set_defaults(func=cmd_build)

    a = ap.parse_args(argv)
    return a.func(a)


if __name__ == "__main__":
    raise SystemExit(main())
