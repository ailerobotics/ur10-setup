"""Runtime contact detector.

Pure, deterministic, time-passed-in (no wall clock) so it unit-tests and
replays offline. Consumes a stream of samples for one leg (or one stop-and-probe
segment) and decides whether a contact fault is happening.

The CB3 reality (sibling scout ``data/ur10-pick-failure-modes-s1``): a
current-estimate wrench that swings to ~427 N / ~1471 Nm on clean motion at a
wrist-flip, while the parts break at 15-350 N. So in motion the force estimate
can only CORROBORATE; the trustworthy in-motion signals are tracking lag and an
actual-progress stall. BUT if the arm HALTS at the contact pose and captures a
fresh wrench baseline there, the pose bias cancels and a baseline-relative
torque/force delta resolves ~3-5 N of tip force (the 0.38 m fork is a lever) --
so in STOP-AND-PROBE creep, torque-from-baseline is a first-class trip signal.

Signal families:

* **P (probe)** -- baseline-relative |F| or wrist-torque delta past the probe
  thresholds, while creeping a segment with a fresh at-pose baseline. First-class
  and reliable (pose bias cancelled). Resolves ~3-5 N tip force.
* **K (stall)** -- commanded progress advances while ACTUAL progress is frozen:
  the arm is physically blocked. First-class and reliable in motion.
* **T (tracking)** -- commanded-vs-actual joint error, *rising* (not a benign
  plateaued lag like the 4.6 deg / 8.64 deg artefacts). Corroborates K, and with
  F gives an EARLY trip.
* **F (force band)** -- fork-frame force/torque/effort out of the leg+pose band.
  Corroboration only in motion (never sufficient); the training record sits
  inside the band, so clean motion never trips on F alone.
* **L (load step)** -- a *sustained* force-band excursion across advancing
  progress on a carry leg: an added load (tray sliding), distinct from a
  pose-locked estimate bump that clears with the pose.

Trips (each with persistence): PROBE (P), STALL (K), EARLY (F+T), LOAD_STEP (L),
and STALE (feed gap -> fail toward stop). Never on F alone, on a benign
plateaued lag, or when not executing.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

from .envelope import LegBand, band_excursion
from .params import ContactGuardParams

FORCE_SIGNALS = ("force_mag", "f_ins", "f_lat", "torque_mag", "t_pry", "eff_max")


@dataclass
class SignalReading:
    signal: str
    value: float
    lo: float
    hi: float
    excursion: float            # 0 inside band, >0 multiples of half-width past it


@dataclass
class Detection:
    leg: str
    trigger: str                # PROBE | STALL | EARLY | LOAD_STEP | STALE
    families: Tuple[str, ...]
    s_at_trip: float
    mm_at_trip: float
    ms_at_trip: float
    readings: List[SignalReading] = field(default_factory=list)
    track_err: Optional[float] = None
    tip_force_n: Optional[float] = None   # probe: inferred tip force from torque
    message: str = ""

    def one_screen(self) -> str:
        lines = [
            f"CONTACT-GUARD TRIP  [{self.trigger}]  leg={self.leg}",
            f"  families: {'+'.join(self.families)}",
            f"  at progress {self.s_at_trip * 100:.1f}%  "
            f"({self.mm_at_trip:.1f} mm, {self.ms_at_trip:.0f} ms into the leg)",
        ]
        if self.track_err is not None:
            lines.append(f"  tracking error: {self.track_err * 57.2958:.2f} deg")
        if self.tip_force_n is not None:
            lines.append(f"  inferred tip force: {self.tip_force_n:.1f} N "
                         f"(torque-from-baseline / lever)")
        for r in self.readings:
            lines.append(
                f"  {r.signal:10s} {r.value:9.2f}  band[{r.lo:.2f},{r.hi:.2f}]"
                f"  x{r.excursion:.2f}")
        if self.message:
            lines.append(f"  {self.message}")
        return "\n".join(lines)


class ContactGuard:
    """Stateful per-leg (or per-segment) detector."""

    def __init__(self, params: ContactGuardParams, leg_band: Optional[LegBand],
                 leg_name: str, contact: bool):
        self.p = params
        self.lb = leg_band
        self.leg = leg_name
        self.contact = contact
        self._probe_baseline: Optional[Dict[str, float]] = None
        self.reset()

    def reset(self) -> None:
        self._hist: Deque[Tuple[float, float, float]] = deque()  # (t, s_cmd, s_act)
        self._te_hist: Deque[Tuple[float, float]] = deque()      # (t, track_err)
        self._early_streak = 0
        self._probe_streak = 0
        self._load_hist: Deque[Tuple[float, float, bool]] = deque()  # (t, s_act, F)
        self._last_t: Optional[float] = None
        self._tripped: Optional[Detection] = None

    # -- stop-and-probe baseline --------------------------------------------
    def set_probe_baseline(self, baseline: Dict[str, float]) -> None:
        """Install a fresh at-pose wrench baseline (mean over the halt dwell)."""
        self._probe_baseline = dict(baseline)

    def clear_probe_baseline(self) -> None:
        self._probe_baseline = None

    # -- family computations -------------------------------------------------
    def _force_band(self, s: float, feats: Dict[str, float]
                    ) -> Tuple[bool, List[SignalReading]]:
        readings: List[SignalReading] = []
        fired = False
        if self.lb is None:
            return False, readings
        for sig in FORCE_SIGNALS:
            if sig not in feats:
                continue
            band = self.lb.at(sig, s)
            if band is None:
                continue
            _, lo, hi = band
            exc = band_excursion(self.lb, sig, s, feats[sig]) or 0.0
            if exc > 0:
                readings.append(SignalReading(sig, feats[sig], lo, hi, exc))
                if exc >= 1.0:
                    fired = True
        return fired, readings

    def _probe(self, feats: Dict[str, float]
               ) -> Tuple[bool, Optional[float], List[SignalReading]]:
        """Baseline-relative |F| / wrist-torque delta trip (reliable at a halt)."""
        b = self._probe_baseline
        if b is None:
            return False, None, []
        def d(k):
            return feats.get(k, 0.0) - b.get(k, 0.0)
        dF = (d("fx") ** 2 + d("fy") ** 2 + d("fz") ** 2) ** 0.5
        dT_axis = abs(d("mz"))                       # torque about the fork axis
        dT_lat = (d("mx") ** 2 + d("my") ** 2) ** 0.5  # lateral (prying) torque
        dT = max(dT_axis, dT_lat)
        tip = dT / self.p.fork_lever_m if self.p.fork_lever_m > 0 else 0.0
        fired = (dF > self.p.probe_force_n) or (dT > self.p.probe_torque_nm)
        readings = [
            SignalReading("dF", dF, 0.0, self.p.probe_force_n,
                          dF / self.p.probe_force_n),
            SignalReading("dT", dT, 0.0, self.p.probe_torque_nm,
                          dT / self.p.probe_torque_nm),
        ]
        return fired, tip, readings

    def _track(self, t: float, feats: Dict[str, float]
               ) -> Tuple[bool, Optional[float]]:
        """Tracking RISING past the rate + elevated past the floor (not a plateau)."""
        te = feats.get("track_err")
        if te is None:
            return False, None
        self._te_hist.append((t, abs(te)))
        while self._te_hist and self._te_hist[0][0] < t - self.p.persist_s:
            self._te_hist.popleft()
        rising = False
        if len(self._te_hist) >= 2:
            dt = self._te_hist[-1][0] - self._te_hist[0][0]
            if dt > 1e-6:
                rate = (self._te_hist[-1][1] - self._te_hist[0][1]) / dt
                rising = rate > self.p.track_rate_rad_s
        elevated = abs(te) > self.p.track_rise_rad
        return (rising and elevated), float(te)

    def _stall(self, now: float) -> bool:
        if len(self._hist) < 2:
            return False
        t0 = now - self.p.progress_stall_s
        window = [h for h in self._hist if h[0] >= t0]
        if len(window) < 2 or (window[-1][0] - window[0][0]) < 0.5 * self.p.progress_stall_s:
            return False
        cmd_adv = window[-1][1] - window[0][1]
        act_adv = window[-1][2] - window[0][2]
        # scale-free: the plan commanded meaningful motion but the arm barely
        # moved. Works on both a short insert and a long carry leg.
        return bool(cmd_adv >= self.p.stall_min_cmd_progress
                    and act_adv < self.p.stall_ratio * cmd_adv)

    def _load_step(self, now: float, f_fired: bool, s_act: float) -> bool:
        """Sustained force-band excursion across advancing progress (carry slide)."""
        self._load_hist.append((now, s_act, f_fired))
        while self._load_hist and self._load_hist[0][0] < now - self.p.load_step_persist_s:
            self._load_hist.popleft()
        if (now - self._load_hist[0][0]) < self.p.load_step_persist_s:
            return False
        n = len(self._load_hist)
        frac = sum(1 for x in self._load_hist if x[2]) / max(n, 1)
        prog_adv = self._load_hist[-1][1] - self._load_hist[0][1]
        # sustained (>=80% of the window out of band) across advancing progress:
        # a real added load, not a pose-locked estimate bump that clears quickly.
        return bool(frac >= 0.8 and prog_adv >= self.p.progress_stall_frac)

    def _mm(self, s_act: float) -> float:
        return s_act * (self.lb.mm_per_progress() if self.lb else 0.0)

    def _ms(self, t: float) -> float:
        return 1000.0 * (t - self._hist[0][0]) if self._hist else 0.0

    # -- main ----------------------------------------------------------------
    def update(self, t: float, s_cmd: float, s_act: float,
               feats: Dict[str, float], executing: bool = True,
               creeping: bool = False) -> Optional[Detection]:
        """Feed one sample. Returns the Detection the first time it trips.

        ``creeping`` is True during a stop-and-probe segment creep (a fresh
        baseline must be installed via :meth:`set_probe_baseline`); then the
        probe family P is a first-class trip.
        """
        if self._tripped is not None:
            return self._tripped
        if not executing:
            self._early_streak = self._probe_streak = 0
            self._hist.clear(); self._te_hist.clear(); self._load_hist.clear()
            self._last_t = t
            return None
        if self._last_t is not None and (t - self._last_t) > self.p.stale_s:
            self._tripped = Detection(
                leg=self.leg, trigger="STALE", families=("feed",),
                s_at_trip=s_act, mm_at_trip=self._mm(s_act), ms_at_trip=self._ms(t),
                message=(f"no fresh sample for {t - self._last_t:.2f}s > "
                         f"{self.p.stale_s}s -- fail toward stop"))
            return self._tripped
        self._last_t = t
        self._hist.append((t, s_cmd, s_act))
        horizon = max(self.p.progress_stall_s, self.p.persist_s,
                      self.p.load_step_persist_s) + 0.05
        while self._hist and self._hist[0][0] < t - horizon:
            self._hist.popleft()

        f_fired, f_readings = self._force_band(s_act, feats)
        t_rising, te = self._track(t, feats)
        k_fired = self._stall(t)
        p_fired, tip, p_readings = (self._probe(feats) if creeping
                                    else (False, None, []))
        l_fired = (self._load_step(t, f_fired, s_act)
                   if not self.contact else False)

        self._early_streak = self._early_streak + 1 if (f_fired and t_rising) else 0
        self._probe_streak = self._probe_streak + 1 if p_fired else 0

        trigger = None
        families: Tuple[str, ...] = ()
        readings = f_readings
        tip_force = None
        if self._probe_streak >= self.p.persist_samples:
            trigger = "PROBE"
            families = ("P",) + (("K",) if k_fired else ()) + (("T",) if t_rising else ())
            readings = p_readings + f_readings
            tip_force = tip
        elif k_fired:
            trigger = "STALL"
            families = ("K",) + (("T",) if te is not None else ()) + (("F",) if f_fired else ())
        elif self._early_streak >= self.p.persist_samples:
            trigger = "EARLY"
            families = ("F", "T")
        elif l_fired:
            trigger = "LOAD_STEP"
            families = ("L",)

        if trigger is None:
            return None
        self._tripped = Detection(
            leg=self.leg, trigger=trigger, families=families,
            s_at_trip=s_act, mm_at_trip=self._mm(s_act), ms_at_trip=self._ms(t),
            readings=readings, track_err=te if "track_err" in feats else None,
            tip_force_n=tip_force,
            message={
                "PROBE": "baseline-relative wrist load past the stop-and-probe floor",
                "STALL": "actual progress stalled while commanded to advance",
                "EARLY": "force out of band with rising tracking error",
                "LOAD_STEP": "sustained carry load step across advancing progress",
            }[trigger])
        return self._tripped
