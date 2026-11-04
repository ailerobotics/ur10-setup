"""Leg-aware, progress-indexed signal envelopes (baseline + tolerance band).

An :class:`Envelope` holds, per leg and per normalised-progress bin, a baseline
and a tolerance band for each signal. Built offline from clean teach-002 legs
(:func:`build_leg_band`), evaluated at runtime by interpolating the band at the
current progress (:meth:`LegBand.at`). Serialises to a small, human-diffable
yaml committed under ``envelopes/``.

Band half-width is ``max(k * robust_sigma, absolute_floor)`` where robust_sigma
is MAD-based (1.4826*MAD), so the pose-dependent force-estimate drift (the
427 N wrist-flip artefact is clean motion) sits inside the band by construction
while a real contact step still leaves it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .params import ContactGuardParams
from .signals import ALL_SIGNALS, BAG_SIGNALS, LegTimeSeries

SIGNAL_FLOOR = {
    "force_mag": "floor_force_n",
    "f_ins": "floor_force_n",
    "f_lat": "floor_force_n",
    "torque_mag": "floor_torque_nm",
    "t_pry": "floor_torque_nm",
    "eff_max": "floor_eff_nm",
    "tcp_vel": "floor_tcp_vel",
    "track_err": "floor_track_rad",
}


@dataclass
class SignalBand:
    """Per-signal baseline + band over progress-bin centres (all (B,) arrays)."""
    baseline: np.ndarray
    lo: np.ndarray
    hi: np.ndarray

    def at(self, centres: np.ndarray, s: float) -> Tuple[float, float, float]:
        """Interpolate (baseline, lo, hi) at normalised progress ``s``."""
        s = float(min(1.0, max(0.0, s)))
        b = float(np.interp(s, centres, self.baseline))
        lo = float(np.interp(s, centres, self.lo))
        hi = float(np.interp(s, centres, self.hi))
        return b, lo, hi


@dataclass
class LegBand:
    leg: str
    contact: bool
    centres: np.ndarray                             # (B,) progress bin centres
    signals: Dict[str, SignalBand] = field(default_factory=dict)
    duration_s: float = 0.0                         # nominal clean leg duration
    tcp_travel_m: float = 0.0                       # nominal clean TCP path length
    n_source_samples: int = 0

    def has(self, signal: str) -> bool:
        return signal in self.signals

    def at(self, signal: str, s: float) -> Optional[Tuple[float, float, float]]:
        sb = self.signals.get(signal)
        if sb is None:
            return None
        return sb.at(self.centres, s)

    def mm_per_progress(self) -> float:
        """TCP mm travelled per unit normalised progress (for mm-at-trip)."""
        return 1000.0 * self.tcp_travel_m

    def ms_per_progress(self) -> float:
        """Nominal ms per unit normalised progress (for ms-at-trip at teach speed)."""
        return 1000.0 * self.duration_s


@dataclass
class Envelope:
    legs: Dict[str, LegBand] = field(default_factory=dict)
    provenance: Dict[str, object] = field(default_factory=dict)

    def leg(self, name: str) -> Optional[LegBand]:
        return self.legs.get(name)

    # --- serialisation -------------------------------------------------------
    def to_dict(self) -> dict:
        out = {"provenance": dict(self.provenance), "legs": {}}
        for name, lb in self.legs.items():
            sig = {}
            for s, sb in lb.signals.items():
                sig[s] = {
                    "baseline": [round(float(x), 4) for x in sb.baseline],
                    "lo": [round(float(x), 4) for x in sb.lo],
                    "hi": [round(float(x), 4) for x in sb.hi],
                }
            out["legs"][name] = {
                "contact": bool(lb.contact),
                "centres": [round(float(x), 4) for x in lb.centres],
                "duration_s": round(float(lb.duration_s), 3),
                "tcp_travel_m": round(float(lb.tcp_travel_m), 4),
                "n_source_samples": int(lb.n_source_samples),
                "signals": sig,
            }
        return out

    @classmethod
    def from_dict(cls, d: dict) -> "Envelope":
        env = cls(provenance=dict(d.get("provenance", {})))
        for name, ld in (d.get("legs") or {}).items():
            centres = np.asarray(ld["centres"], dtype=float)
            signals = {}
            for s, sd in (ld.get("signals") or {}).items():
                signals[s] = SignalBand(
                    baseline=np.asarray(sd["baseline"], dtype=float),
                    lo=np.asarray(sd["lo"], dtype=float),
                    hi=np.asarray(sd["hi"], dtype=float),
                )
            env.legs[name] = LegBand(
                leg=name,
                contact=bool(ld.get("contact", False)),
                centres=centres,
                signals=signals,
                duration_s=float(ld.get("duration_s", 0.0)),
                tcp_travel_m=float(ld.get("tcp_travel_m", 0.0)),
                n_source_samples=int(ld.get("n_source_samples", 0)),
            )
        return env


def _robust_sigma(x: np.ndarray) -> float:
    if x.size < 2:
        return 0.0
    med = np.median(x)
    mad = np.median(np.abs(x - med))
    return float(1.4826 * mad)


def build_signal_band(progress: np.ndarray, values: np.ndarray,
                      floor: float, p: ContactGuardParams) -> SignalBand:
    """Build one signal's band over ``p.n_progress_bins`` progress bins.

    Empty bins are filled by nearest-neighbour then linear interpolation so the
    band is defined over all of [0, 1].
    """
    B = p.n_progress_bins
    edges = np.linspace(0.0, 1.0, B + 1)
    centres = 0.5 * (edges[:-1] + edges[1:])
    baseline = np.full(B, np.nan)
    tol = np.full(B, np.nan)
    idx = np.clip(np.digitize(progress, edges) - 1, 0, B - 1)
    for b in range(B):
        vals = values[idx == b]
        if vals.size == 0:
            continue
        med = float(np.median(vals))
        baseline[b] = med
        sigma = _robust_sigma(vals)
        # Band must (a) absorb the pose-dependent estimate drift (k*sigma),
        # (b) never be narrower than an absolute floor, and (c) fully ENVELOPE
        # the clean samples seen in this bin so the training record cannot
        # false-trip the force family. A small pad covers cross-bin interpolation.
        envelope_half = max(float(vals.max()) - med, med - float(vals.min()))
        tol[b] = max(p.band_k_sigma * sigma, floor, envelope_half)
        tol[b] += max(0.08 * tol[b], 0.15 * floor)  # interpolation pad
    good = ~np.isnan(baseline)
    if not good.any():
        baseline[:] = 0.0
        tol[:] = floor
    else:
        gc = centres[good]
        baseline = np.interp(centres, gc, baseline[good])
        tol = np.interp(centres, gc, tol[good])
        tol = np.maximum(tol, floor)
    return SignalBand(baseline=baseline, lo=baseline - tol, hi=baseline + tol)


def build_leg_band(ts: LegTimeSeries, contact: bool, p: ContactGuardParams,
                   tcp_travel_m: float = 0.0) -> LegBand:
    """Build a :class:`LegBand` from one clean leg's time series."""
    B = p.n_progress_bins
    edges = np.linspace(0.0, 1.0, B + 1)
    centres = 0.5 * (edges[:-1] + edges[1:])
    lb = LegBand(
        leg=ts.leg, contact=contact, centres=centres,
        duration_s=ts.duration_s, tcp_travel_m=tcp_travel_m,
        n_source_samples=ts.n,
    )
    for sig, vals in ts.feats.items():
        if sig not in ALL_SIGNALS:
            continue
        floor = getattr(p, SIGNAL_FLOOR[sig])
        lb.signals[sig] = build_signal_band(ts.progress, np.asarray(vals), floor, p)
    return lb


def band_excursion(lb: LegBand, signal: str, s: float, value: float
                   ) -> Optional[float]:
    """Signed excursion past the band in tolerance-widths.

    Returns 0.0 inside the band, >0 above ``hi`` / below ``lo`` measured as a
    multiple of the local half-width, or None if the leg has no such signal.
    """
    got = lb.at(signal, s)
    if got is None:
        return None
    baseline, lo, hi = got
    half = max((hi - lo) / 2.0, 1e-9)
    if value > hi:
        return (value - hi) / half
    if value < lo:
        return (lo - value) / half
    return 0.0
