"""Verdict JSON file -- the wire contract with the sibling contact-guard lane.

The sibling lane's pre-contact gate READS this file (it is not edited here).
The schema is documented in fork_demo/align_check/README.md "Verdict schema".

Required fields (agreed contract): side, ok, dx_mm, dy_mm, dyaw_deg,
tolerance, margin, image_stamp, assumptions. ``tolerance`` and ``margin`` are
objects with ``mm`` and ``deg`` members so one reader handles both axes.
Extra, non-contract fields (correction, detail, schema_version, generated_at)
are additive and a strict reader may ignore them.
"""

from __future__ import annotations

import datetime
import json
import os

SCHEMA_VERSION = "align_check/v1"


def verdict_to_contract(v, *, generated_at: str | None = None) -> dict:
    """SideVerdict -> the contract dict written to disk."""
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": generated_at
        or datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "side": v.side,
        "ok": bool(v.ok),
        "dx_mm": v.dx_mm,
        "dy_mm": v.dy_mm,
        "dyaw_deg": v.dyaw_deg,
        "tolerance": {"mm": v.tolerance_mm, "deg": v.tolerance_deg},
        "margin": {"mm": v.margin_mm, "deg": v.margin_deg},
        "correction": {
            "dx_mm": v.correction_dx_mm,
            "dy_mm": v.correction_dy_mm,
            "dyaw_deg": v.correction_dyaw_deg,
            "note": "PROPOSAL ONLY -- never applied automatically; captain applies by hand.",
        },
        "image_stamp": v.image_stamp,
        "assumptions": list(v.assumptions),
        "detail": v.detail,
    }


def write_verdict(v, path: str, *, generated_at: str | None = None) -> dict:
    """Atomically write the verdict JSON; returns the contract dict."""
    contract = verdict_to_contract(v, generated_at=generated_at)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(contract, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)
    return contract
