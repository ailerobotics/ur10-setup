"""Offline fault injection and the named near-miss negatives.

Builds a clean synthetic *replay* of a leg from its envelope baselines (the arm
following its taught path at replay speed, tracking it), then injects the
signatures of the six mechanical faults in the brief, and separately reproduces
the session's real near-misses (427 N / 318 N estimate artefacts, the 8.64 deg
loaded-lift lag, the External Control drop) as negatives. All pure; feeds the
same :class:`detector.ContactGuard` the live node uses.

A replay sample is ``(t, s_cmd, s_act, feats, executing, creeping)``. On clean
motion ``s_act == s_cmd`` (perfect tracking) and ``feats`` are the band
baselines. A fault perturbs the stream from an onset progress; the detector's
trip and the travel/latency past onset are measured.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from .detector import ContactGuard, Detection
from .envelope import LegBand
from .params import ContactGuardParams
from .signals import BAG_SIGNALS

# a nominal, benign, PLATEAUED tracking lag on a clean dense-waypoint replay (rad)
NOMINAL_TRACK_RAD = 0.02
# rad of dominant-joint motion per unit normalised progress (track gain for a
# commanded-vs-actual progress gap); a contact leg spans ~1-2 rad of a joint.
TRACK_GAIN_RAD = 2.0


@dataclass
class Sample:
    t: float
    s_cmd: float
    s_act: float
    feats: Dict[str, float]
    executing: bool = True
    creeping: bool = False


@dataclass
class InjectResult:
    name: str
    leg: str
    detected: bool
    detection: Optional[Detection]
    onset_s: float                 # progress at fault onset
    mm_after_onset: float          # commanded travel past onset at trip
    ms_after_onset: float          # time past onset at trip
    force_mag_at_trip: float
    margin_to_200n: float          # 200 - |F| (old watchdog headroom)
    margin_to_pendant_n: float     # pendant floor - inferred tip/|F|


def _baseline_feats(lb: LegBand, s: float) -> Dict[str, float]:
    feats: Dict[str, float] = {}
    for sig in BAG_SIGNALS:
        band = lb.at(sig, s)
        if band is not None:
            feats[sig] = band[0]            # baseline value
    # raw components consistent with the scalar baselines (fz == f_ins etc.)
    feats["fx"] = 0.0
    feats["fy"] = feats.get("f_lat", 0.0)
    feats["fz"] = feats.get("f_ins", 0.0)
    feats["mx"] = 0.0
    feats["my"] = feats.get("t_pry", 0.0)
    feats["mz"] = 0.0
    feats["track_err"] = NOMINAL_TRACK_RAD
    return feats


def clean_replay(lb: LegBand, hz: float = 100.0,
                 duration_s: Optional[float] = None) -> List[Sample]:
    """A clean, perfectly-tracked replay of the leg from its baselines."""
    dur = duration_s if duration_s is not None else max(lb.duration_s, 1.0)
    n = max(int(dur * hz), 4)
    out: List[Sample] = []
    for i in range(n):
        t = i / hz
        s = i / (n - 1)
        out.append(Sample(t=t, s_cmd=s, s_act=s, feats=_baseline_feats(lb, s)))
    return out


def _run(guard: ContactGuard, samples: List[Sample]
         ) -> Tuple[Optional[Detection], int]:
    det = None
    trip_i = -1
    for i, s in enumerate(samples):
        if det is None:
            det = guard.update(s.t, s.s_cmd, s.s_act, s.feats,
                               executing=s.executing, creeping=s.creeping)
            if det is not None:
                trip_i = i
    return det, trip_i


def _result(name: str, lb: LegBand, samples: List[Sample], onset_i: int,
            det: Optional[Detection], trip_i: int) -> InjectResult:
    onset = samples[onset_i]
    travel_mm = 1000.0 * lb.tcp_travel_m
    if det is not None and trip_i >= 0:
        trip = samples[trip_i]
        # intrusion = how far the COMMANDED path drove past the contact point
        mm = max(0.0, (trip.s_cmd - onset.s_cmd) * travel_mm)
        ms = max(0.0, (trip.t - onset.t) * 1000.0)
        fmag = trip.feats.get("force_mag", 0.0)
        tip = det.tip_force_n if det.tip_force_n is not None else fmag
    else:
        mm = ms = 0.0
        fmag = samples[-1].feats.get("force_mag", 0.0)
        tip = fmag
    return InjectResult(
        name=name, leg=lb.leg, detected=det is not None, detection=det,
        onset_s=onset.s_cmd, mm_after_onset=mm, ms_after_onset=ms,
        force_mag_at_trip=fmag, margin_to_200n=200.0 - fmag,
        margin_to_pendant_n=100.0 - tip)


# --------------------------------------------------------------------------- #
# the six injected faults (impede motion -> stall / probe; or load step)
# --------------------------------------------------------------------------- #
def _stall_from(samples: List[Sample], onset_i: int, *, f_ins=0.0, f_lat=0.0,
                t_pry=0.0, eff=0.0, mz=0.0) -> None:
    """Freeze actual progress from onset; grow tracking; add a wrench signature
    that scales with how far the plan has run past the contact (intrusion)."""
    s_frozen = samples[onset_i].s_act
    for j in range(onset_i, len(samples)):
        gap = samples[j].s_cmd - s_frozen
        samples[j].s_act = s_frozen
        samples[j].feats["track_err"] = NOMINAL_TRACK_RAD + TRACK_GAIN_RAD * gap
        g = gap / max(samples[-1].s_cmd - s_frozen, 1e-6)
        f = samples[j].feats
        f["f_ins"] = f.get("f_ins", 0.0) + f_ins * g
        f["fz"] = f["f_ins"]
        f["f_lat"] = f.get("f_lat", 0.0) + f_lat * g
        f["fy"] = f["f_lat"]
        f["t_pry"] = f.get("t_pry", 0.0) + t_pry * g
        f["my"] = f["t_pry"]
        f["mz"] = mz * g
        f["eff_max"] = f.get("eff_max", 0.0) + eff * g
        f["force_mag"] = (f["fx"] ** 2 + f["fy"] ** 2 + f["fz"] ** 2) ** 0.5
        f["torque_mag"] = (f["mx"] ** 2 + f["my"] ** 2 + f["mz"] ** 2) ** 0.5


FAULTS = (
    "insert_hit", "lift_under_post", "extract_snag",
    "setdown_misaligned", "carry_collision", "tray_slide",
)


def inject_fault(name: str, lb: LegBand, p: ContactGuardParams,
                 onset: float = 0.45) -> InjectResult:
    samples = clean_replay(lb)
    onset_i = min(range(len(samples)), key=lambda i: abs(samples[i].s_cmd - onset))
    contact = lb.contact
    if name == "insert_hit":
        _stall_from(samples, onset_i, f_ins=120.0, eff=40.0)
    elif name == "lift_under_post":
        _stall_from(samples, onset_i, t_pry=18.0, mz=4.0, eff=35.0)
    elif name == "extract_snag":
        _stall_from(samples, onset_i, f_ins=-90.0, f_lat=40.0, eff=30.0)
    elif name == "setdown_misaligned":
        # premature vertical load + lateral force, arm resists descent -> stall
        _stall_from(samples, onset_i, f_ins=70.0, f_lat=50.0, eff=28.0)
    elif name == "carry_collision":
        _stall_from(samples, onset_i, f_ins=60.0, f_lat=60.0, eff=45.0)
        contact = False
    elif name == "tray_slide":
        # load shifts: a SUSTAINED lateral + insertion-axis force step, arm keeps
        # tracking the free carry (no stall). Caught as LOAD_STEP on a carry leg.
        for j in range(onset_i, len(samples)):
            f = samples[j].feats
            f["f_lat"] = f.get("f_lat", 0.0) + 55.0
            f["fy"] = f["f_lat"]
            f["f_ins"] = f.get("f_ins", 0.0) + 35.0
            f["fz"] = f["f_ins"]
            f["force_mag"] = (f["fx"] ** 2 + f["fy"] ** 2 + f["fz"] ** 2) ** 0.5
        contact = False
    else:
        raise ValueError(f"unknown fault {name!r}")
    guard = ContactGuard(p, lb, lb.leg, contact)
    det, ti = _run(guard, samples)
    return _result(name, lb, samples, onset_i, det, ti)


def inject_probe_touch(lb: LegBand, p: ContactGuardParams,
                       seg_start: float = 0.45, tip_force_n: float = 6.0
                       ) -> InjectResult:
    """A gentle tip touch during a STOP-AND-PROBE creep of ONE 15 mm segment.

    Models the real protocol: halt, capture a fresh at-pose baseline, then creep
    a single short segment (so the pose bias barely drifts and the baseline-
    relative torque cleanly reveals the tip force).
    """
    span = p.segment_m / max(lb.tcp_travel_m, 1e-6)       # 15 mm in progress units
    seg_dur = max(p.segment_m / 0.01, 0.5)                # creep ~1 cm/s, min 0.5 s
    hz = 100.0
    n = max(int(seg_dur * hz), p.probe_min_samples + 4)
    samples: List[Sample] = []
    for i in range(n):
        s = seg_start + span * (i / (n - 1))
        f = _baseline_feats(lb, s)
        samples.append(Sample(t=i / hz, s_cmd=s, s_act=s, feats=f, creeping=True))
    baseline = dict(_baseline_feats(lb, seg_start))        # fresh at-pose baseline
    onset_i = 2
    dT = tip_force_n * p.fork_lever_m                       # tip force -> wrist torque
    for j in range(onset_i, n):
        f = samples[j].feats
        f["my"] = f.get("my", 0.0) + dT
        f["torque_mag"] = (f["mx"] ** 2 + f["my"] ** 2 + f["mz"] ** 2) ** 0.5
    guard = ContactGuard(p, lb, lb.leg, lb.contact)
    guard.set_probe_baseline(baseline)
    det, ti = _run(guard, samples)
    return _result(f"probe_touch_{tip_force_n:.0f}N", lb, samples, onset_i, det, ti)


# --------------------------------------------------------------------------- #
# the named near-miss NEGATIVES (must NOT trip)
# --------------------------------------------------------------------------- #
def negative_force_artifact(lb: LegBand, p: ContactGuardParams,
                            peak_n: float = 427.0, onset: float = 0.5
                            ) -> InjectResult:
    """A pose-locked force-estimate spike (427/318 N) with a PLATEAUED (not
    rising) tracking lag and no stall -- the wrist-flip artefact."""
    samples = clean_replay(lb)
    onset_i = min(range(len(samples)), key=lambda i: abs(samples[i].s_cmd - onset))
    width = 0.06                              # localised in progress (pose-locked)
    for s in samples:
        bump = peak_n * np.exp(-((s.s_cmd - onset) ** 2) / (2 * width ** 2))
        f = s.feats
        f["force_mag"] = f.get("force_mag", 0.0) + bump
        f["f_ins"] = f.get("f_ins", 0.0) + bump * 0.8
        f["fz"] = f["f_ins"]
        # a benign steady lag that RISES then PLATEAUS at ~4.6 deg (0.08 rad)
        s.feats["track_err"] = min(0.08, NOMINAL_TRACK_RAD + 0.4 * s.s_cmd)
    guard = ContactGuard(p, lb, lb.leg, lb.contact)
    det, ti = _run(guard, samples)
    return _result(f"artifact_{peak_n:.0f}N", lb, samples, onset_i, det, ti)


def negative_loaded_lift_lag(lb: LegBand, p: ContactGuardParams,
                             peak_deg: float = 8.64) -> InjectResult:
    """A loaded-lift dynamic lag: tracking rises to peak then PLATEAUS while the
    arm keeps advancing (no stall), force in band. Must NOT trip."""
    samples = clean_replay(lb)
    peak = np.radians(peak_deg)
    for s in samples:
        # rise over the first 20% then hold (plateau), arm advances normally
        frac = min(1.0, s.s_cmd / 0.2)
        s.feats["track_err"] = NOMINAL_TRACK_RAD + (peak - NOMINAL_TRACK_RAD) * frac
    guard = ContactGuard(p, lb, lb.leg, lb.contact)
    det, ti = _run(guard, samples)
    return _result(f"loaded_lift_lag_{peak_deg:.2f}deg", lb, samples, 0, det, ti)


FAULT_LEG = {
    "insert_hit": "insert_forks",
    "lift_under_post": "lift",
    "extract_snag": "lift_from_table",
    "setdown_misaligned": "lower_on_table",
    "carry_collision": "carry_to_table",
    "tray_slide": "carry_to_table",
}


def fault_report(env, p: ContactGuardParams) -> Dict[str, object]:
    """Run every injected fault + probe-touch + the named negatives.

    Returns a dict with ``faults`` (list of InjectResult-as-dict), ``probe``,
    ``negatives``, and ``all_detected`` / ``any_negative_tripped`` booleans.
    """
    def row(r: InjectResult) -> Dict[str, object]:
        return {
            "name": r.name, "leg": r.leg, "detected": r.detected,
            "trigger": (r.detection.trigger if r.detection else None),
            "mm_after_onset": round(r.mm_after_onset, 1),
            "ms_after_onset": round(r.ms_after_onset, 0),
            "force_mag_at_trip_n": round(r.force_mag_at_trip, 1),
            "margin_to_200n": round(r.margin_to_200n, 1),
            "margin_to_pendant_n": round(r.margin_to_pendant_n, 1),
            "tip_force_n": (round(r.detection.tip_force_n, 1)
                            if (r.detection and r.detection.tip_force_n is not None)
                            else None),
        }
    faults = [row(inject_fault(f, env.leg(FAULT_LEG[f]), p)) for f in FAULTS]
    probe = [row(inject_probe_touch(env.leg(leg), p, tip_force_n=6.0))
             for leg in ("insert_forks", "lift", "lower_on_table") if env.leg(leg)]
    negatives = []
    for leg, peak in (("to_pre_grip", 427.0), ("insert_forks", 318.0)):
        if env.leg(leg):
            negatives.append(row(negative_force_artifact(env.leg(leg), p, peak)))
    if env.leg("lift"):
        negatives.append(row(negative_loaded_lift_lag(env.leg("lift"), p)))
    if env.leg("insert_forks"):
        negatives.append(row(negative_ec_drop(env.leg("insert_forks"), p)))
    return {
        "faults": faults, "probe": probe, "negatives": negatives,
        "all_detected": all(r["detected"] for r in faults + probe),
        "any_negative_tripped": any(r["detected"] for r in negatives),
    }


def negative_ec_drop(lb: LegBand, p: ContactGuardParams,
                     drop_at: float = 0.5) -> InjectResult:
    """External Control drops mid-leg: executing goes False -> stand down."""
    samples = clean_replay(lb)
    for s in samples:
        if s.s_cmd >= drop_at:
            s.executing = False
            s.s_act = drop_at          # arm halts where it was
    guard = ContactGuard(p, lb, lb.leg, lb.contact)
    det, ti = _run(guard, samples)
    return _result("ec_drop", lb, samples, 0, det, ti)
