"""External alignment-veto interface (file or topic JSON).

A SEPARATE camera-alignment lane (not implemented here) publishes a small JSON
verdict that can veto the pre-contact gate. This module only READS and validates
it, fail-toward-stop: a missing, malformed or stale verdict vetoes (blocks).

Wire format (``std_msgs/String`` JSON on a topic, or a JSON file)::

    {
      "schema": "contact_guard.align_veto.v1",
      "stamp": 1793799999.5,        # epoch seconds the verdict was produced
      "leg": "insert_forks",         # or "*" for any
      "ok": true,                    # true = aligned, gate may pass
      "reason": "tag 1 reproj 0.4 px, offset 3 mm",
      "source": "cams/cellmap align lane"
    }

``ok`` must be an explicit boolean; anything else is treated as a veto.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

SCHEMA = "contact_guard.align_veto.v1"


@dataclass
class VetoState:
    ok: bool
    reason: str
    leg: str = "*"
    stamp: Optional[float] = None
    stale: bool = False

    @property
    def blocks(self) -> bool:
        return (not self.ok) or self.stale


def parse_veto(text: str, now: Optional[float] = None, max_age_s: float = 5.0,
               leg: Optional[str] = None) -> VetoState:
    """Parse a veto JSON string. Fail-toward-stop on any problem."""
    try:
        d = json.loads(text)
    except Exception as exc:
        return VetoState(False, f"unparseable alignment verdict: {exc}", stale=True)
    if not isinstance(d, dict) or d.get("schema") != SCHEMA:
        return VetoState(False, "alignment verdict schema mismatch", stale=True)
    ok = d.get("ok")
    if not isinstance(ok, bool):
        return VetoState(False, "alignment 'ok' not a boolean -> veto", stale=True)
    stamp = d.get("stamp")
    stale = False
    if now is not None and isinstance(stamp, (int, float)):
        stale = (now - float(stamp)) > max_age_s
    verdict_leg = str(d.get("leg", "*"))
    if leg is not None and verdict_leg not in ("*", leg):
        return VetoState(False, f"alignment verdict is for leg {verdict_leg}, "
                                f"not {leg} -> veto", leg=verdict_leg,
                         stamp=stamp if isinstance(stamp, (int, float)) else None,
                         stale=True)
    return VetoState(ok=bool(ok), reason=str(d.get("reason", "")),
                     leg=verdict_leg,
                     stamp=stamp if isinstance(stamp, (int, float)) else None,
                     stale=stale)


def read_veto_file(path: str, now: Optional[float] = None, max_age_s: float = 5.0,
                   leg: Optional[str] = None) -> VetoState:
    try:
        with open(path) as fh:
            return parse_veto(fh.read(), now=now, max_age_s=max_age_s, leg=leg)
    except FileNotFoundError:
        return VetoState(False, f"no alignment verdict file at {path}", stale=True)
    except Exception as exc:
        return VetoState(False, f"alignment verdict unreadable: {exc}", stale=True)
