#!/usr/bin/env python3
"""
Sole-writer monitor — the gate must be the ONLY writer of the speed slider during
guarded motion (report §5 #16; RUNBOOK §6b "one writer at a time").

It watches the ACTUAL applied scaling on
`/speed_scaling_state_broadcaster/speed_scaling` and compares it to what the gate
last commanded. It must separate two look-alike situations that need OPPOSITE
responses:

  * FOREIGN WRITE — the scaling MOVED to a value the gate did not command (some
    other operator/session touched the slider). Response: conflict -> STOP
    (the node escalates to the backstop) and surface it loudly.
  * INEFFECTIVE SLIDER — the gate commanded a change but the scaling did NOT move
    from where it already was (e.g. the driver's mock hardware reports scaling
    inert at 1.0; report §8). This is NOT a foreign writer; it is handled by the
    primary->backstop fall-through via the driver-stopped feedback, so this
    monitor deliberately does NOT raise a conflict for it.

The discriminator is movement, not mere mismatch: a sustained mismatch AT THE
PRE-COMMAND VALUE is "ineffective"; a sustained mismatch at some OTHER value is
"foreign". A brief mismatch while the scaling ramps toward the commanded value is
tolerated for `settle_s`.

PURE: standard library only. Fully unit-tested offline against a synthetic
scaling timeline.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class SoleWriterMonitor:
    tol: float = 0.05          # scaling within this of a target counts as "matched"
    settle_s: float = 0.5      # time allowed to ramp to a commanded value
    lookback_s: float = 1.0    # a scaling value matching a command this recent is ours

    def __post_init__(self) -> None:
        self._expected: Optional[float] = None       # last fraction the gate commanded
        self._pre_command_scaling: Optional[float] = None  # scaling when it was commanded
        self._last_scaling: Optional[float] = None
        self._recent_commands: List[Tuple[float, float]] = []  # (t, fraction)
        self._mismatch_since: Optional[float] = None
        self.conflict: bool = False
        self.conflict_reason: str = ""

    # ---- inputs ---------------------------------------------------------- #
    def note_command(self, fraction: float, t: float) -> None:
        """Record that the gate commanded the slider to `fraction` at time `t`."""
        self._expected = float(fraction)
        # snapshot where the scaling was so a non-moving slider reads as inert.
        self._pre_command_scaling = self._last_scaling
        self._recent_commands.append((t, float(fraction)))
        self._recent_commands = [
            (ct, cf) for ct, cf in self._recent_commands if t - ct <= self.lookback_s]
        self._mismatch_since = None

    def observe(self, scaling: float, t: float) -> Optional[str]:
        """Feed one observed speed-scaling sample. Returns a conflict reason
        string the FIRST time a foreign write is confirmed, else None. The
        conflict latches (stays set until `reset`)."""
        scaling = float(scaling)
        prev = self._last_scaling
        self._last_scaling = scaling
        if self.conflict:
            return None                      # already latched; node holds STOP
        if self._expected is None:
            return None                      # gate has not taken control yet

        if abs(scaling - self._expected) <= self.tol:
            self._mismatch_since = None      # in sync with our command
            return None

        # Mismatch. Is the slider simply inert (still at its pre-command value)?
        if (self._pre_command_scaling is not None
                and abs(scaling - self._pre_command_scaling) <= self.tol):
            # ineffective slider, not a foreign writer -> backstop path owns it.
            self._mismatch_since = None
            return None

        # Does it match a fraction the gate itself commanded very recently
        # (e.g. a value observed just after a rapid re-command)? Then it's ours.
        if any(abs(scaling - cf) <= self.tol
               and t - ct <= self.lookback_s for ct, cf in self._recent_commands):
            self._mismatch_since = None
            return None

        # Scaling moved to a value we did not command and that is not the inert
        # pre-command value: could be a ramp transient -> allow settle_s.
        if self._mismatch_since is None:
            self._mismatch_since = t
        if (t - self._mismatch_since) < self.settle_s:
            return None

        self.conflict = True
        self.conflict_reason = (
            f"foreign speed-slider write: observed scaling {scaling:.3f} "
            f"(was {prev if prev is not None else float('nan'):.3f}) does not match "
            f"gate command {self._expected:.3f} and is not the inert pre-command "
            f"value {self._pre_command_scaling} — the gate is not the sole writer")
        return self.conflict_reason

    def reset(self) -> None:
        """Clear a latched conflict (e.g. after an operator re-establishes sole
        control). Not called automatically — a conflict must be acknowledged."""
        self.conflict = False
        self.conflict_reason = ""
        self._mismatch_since = None
