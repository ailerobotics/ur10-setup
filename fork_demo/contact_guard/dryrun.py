"""Replay the detector over recorded/synthetic leg samples (the offline proof).

``dry_run_bag`` runs the detector over every leg of a recorded teach/replay bag
and reports, per leg, whether it tripped and how much band margin remained
(max force-family excursion, <1.0 == inside the band). On the clean teach-002
record this must report zero trips.

For a freedrive teach there is no commanded trajectory, so the arm "tracks
itself": ``s_cmd = s_act`` and ``track_err`` is absent. The stall family (K) and
tracking family (T) are therefore inert on a teach record, and the force family
(F) alone never trips -- so a clean record cannot false-trip, by construction.
This module reports the F-margin so that guarantee is visible, not just asserted.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from . import bagsource
from .detector import FORCE_SIGNALS, ContactGuard, Detection
from .envelope import Envelope, band_excursion
from .params import ContactGuardParams
from .signals import BAG_SIGNALS, LegTimeSeries


@dataclass
class LegVerdict:
    leg: str
    contact: bool
    n_samples: int
    tripped: Optional[Detection]
    max_force_excursion: float          # worst F excursion (<1 == inside band)
    worst_signal: str = ""

    @property
    def ok(self) -> bool:
        return self.tripped is None


@dataclass
class DryRunReport:
    verdicts: List[LegVerdict] = field(default_factory=list)

    @property
    def n_trips(self) -> int:
        return sum(1 for v in self.verdicts if not v.ok)

    @property
    def min_margin(self) -> float:
        """Smallest headroom (1 - max excursion) across all legs (>0 == clean)."""
        if not self.verdicts:
            return 1.0
        return 1.0 - max(v.max_force_excursion for v in self.verdicts)

    def text(self) -> str:
        lines = ["per-leg guard dry-run:"]
        for v in self.verdicts:
            status = "TRIP" if not v.ok else "ok  "
            lines.append(
                f"  {status} {v.leg:18s} contact={str(v.contact):5s} "
                f"n={v.n_samples:5d}  max F-excursion {v.max_force_excursion:.2f}"
                f"  ({v.worst_signal})")
            if v.tripped is not None:
                for ln in v.tripped.one_screen().splitlines():
                    lines.append("      " + ln)
        lines.append(
            f"TOTAL: {self.n_trips} trip(s) over {len(self.verdicts)} legs; "
            f"band margin {self.min_margin:.2f} (headroom to the band edge).")
        return "\n".join(lines)


def replay_leg(guard: ContactGuard, ts: LegTimeSeries,
               s_cmd: Optional[np.ndarray] = None,
               track: Optional[np.ndarray] = None) -> LegVerdict:
    """Replay a leg's samples through ``guard``; return its verdict + margin."""
    det: Optional[Detection] = None
    max_exc = 0.0
    worst_sig = ""
    n = ts.n
    for i in range(n):
        feats = {s: float(ts.feats[s][i]) for s in BAG_SIGNALS if s in ts.feats}
        if track is not None:
            feats["track_err"] = float(track[i])
        sc = float(s_cmd[i]) if s_cmd is not None else float(ts.progress[i])
        sa = float(ts.progress[i])
        # track worst force margin regardless of trip
        if guard.lb is not None:
            for sig in FORCE_SIGNALS:
                if sig in feats:
                    exc = band_excursion(guard.lb, sig, sa, feats[sig])
                    if exc is not None and exc > max_exc:
                        max_exc = exc
                        worst_sig = sig
        if det is None:
            det = guard.update(ts.t[i], sc, sa, feats, executing=True)
    return LegVerdict(leg=ts.leg, contact=guard.contact, n_samples=n,
                      tripped=det, max_force_excursion=max_exc,
                      worst_signal=worst_sig)


def dry_run_bag(run_dir: str, bag_path: str, p: ContactGuardParams,
                env: Envelope) -> DryRunReport:
    legs = bagsource.load_leg_defs(run_dir)
    windows = bagsource.load_dwell_windows(run_dir)
    labels = bagsource.ordered_labels(run_dir)
    streams = bagsource.read_bag(bag_path)
    rep = DryRunReport()
    for leg in legs:
        iv = bagsource.leg_interval(leg, windows, labels)
        if iv is None:
            continue
        ts = bagsource.leg_timeseries(leg, streams, iv)
        ts.feats.pop("_tcp_travel_m", None)
        if ts.n < 2:
            continue
        lb = env.leg(leg.name)
        guard = ContactGuard(p, lb, leg.name, leg.contact)
        rep.verdicts.append(replay_leg(guard, ts))
    return rep
