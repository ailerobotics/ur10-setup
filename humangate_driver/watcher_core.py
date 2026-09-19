#!/usr/bin/env python3
"""
Independent backstop-watcher decision core (task brief item 2; report §2, §5 #11,
#12, #15).

The backstop watcher is a SEPARATE PROCESS with its OWN timer and its OWN copy of
the camera-state heartbeat, so a hang or crash in the primary gate path cannot
disable it. This module is the pure decision it runs each tick; the ROS shell
(backstop_watcher_node.py) only wires topics and issues the actual
FollowJointTrajectory cancel.

It cancels the active trajectory goal (arm decelerates to a hold) on ANY of:

  * the CAMERA-STATE heartbeat is lost — it watches `/human_safety/state`
    itself, not through the gate (report §5 #12);
  * the GATE's decision heartbeat is lost — the primary path died (report §5 #11);
  * the gate explicitly decided BACKSTOP;
  * a STOP is demanded (by the camera state OR the gate) but the speed scaling has
    NOT fallen to a near-stop within a deadline — the slider did not take effect
    (report §5 #15).

Everything fails toward cancel. Once tripped it latches, so the watcher keeps the
goal cancelled until an operator resets it. All time is monotonic seconds, passed
in, so the core is deterministic and unit-tested offline.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# Gate/camera enums mirrored here to avoid importing the gate core into the
# independent process (independence is the point). These are just strings.
ACTION_BACKSTOP = "BACKSTOP"
ACTION_STOP = "STOP"
ZONE_STOP = "STOP"

# STOP-strategy names, mirrored from stop_strategy.py (kept as bare strings so the
# independent process does not import the gate/strategy modules).
STRAT_CANCEL = "cancel"
STRAT_PENDANT_PAUSE = "pendant_pause"


@dataclass
class WatcherParams:
    # Its own camera-state heartbeat timeout. Slightly looser than the gate's own
    # (gate default 0.3 s) so, in the healthy case, the gate acts first; the
    # watcher is the independent net, not a racing duplicate.
    state_heartbeat_timeout_s: float = 0.5
    # No gate decision within this -> the primary path is dead -> cancel.
    gate_heartbeat_timeout_s: float = 0.5
    # After a STOP is demanded, the scaling must fall below stop_scaling_threshold
    # within this (EFFECTIVE) deadline, else the slider did not take effect ->
    # cancel. This is the `cancel`-strategy value; under `pendant_pause` the
    # EFFECTIVE deadline is sized from pendant_freeze_confirm_s (see below), so one
    # knob configures both the gate node and this independent watcher.
    stop_effect_deadline_s: float = 0.7
    stop_scaling_threshold: float = 0.05
    # Grace after startup before "never seen the gate/state" can trip, so the
    # watcher does not cancel during normal graph discovery.
    startup_grace_s: float = 3.0
    # The STOP strategy THIS watcher is configured for. Must match the gate's
    # (params.yaml expresses it once via a YAML anchor). A pendant pause freezes
    # the arm over ~1.8 s — far slower than the slider — so under pendant_pause the
    # effective stop-effect deadline is widened to pendant_freeze_confirm_s, or the
    # watcher would cancel every pause before it freezes.
    stop_strategy: str = STRAT_CANCEL
    pendant_freeze_confirm_s: float = 2.5

    def validate(self) -> "WatcherParams":
        for name in ("state_heartbeat_timeout_s", "gate_heartbeat_timeout_s",
                     "stop_effect_deadline_s", "startup_grace_s",
                     "pendant_freeze_confirm_s"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be > 0")
        if not (0.0 <= self.stop_scaling_threshold < 1.0):
            raise ValueError("stop_scaling_threshold must be in [0, 1)")
        if self.stop_strategy not in (STRAT_CANCEL, STRAT_PENDANT_PAUSE):
            raise ValueError(
                f"stop_strategy must be one of {STRAT_CANCEL!r}/{STRAT_PENDANT_PAUSE!r}")
        return self

    @property
    def effective_stop_effect_deadline_s(self) -> float:
        """The deadline the watcher actually enforces. Under `pendant_pause` it is
        sized from the SAME freeze-confirm knob the gate node uses (never shorter
        than the configured cancel deadline), so the single setting configures both
        processes and the two cannot silently drift."""
        if self.stop_strategy == STRAT_PENDANT_PAUSE:
            return max(self.stop_effect_deadline_s, self.pendant_freeze_confirm_s)
        return self.stop_effect_deadline_s


class BackstopWatcher:
    """Stateful watcher core. Call update() each tick; cancel when it returns a
    reason. Latches once tripped."""

    def __init__(self, params: Optional[WatcherParams] = None,
                 start_time: float = 0.0) -> None:
        self.p = (params or WatcherParams()).validate()
        self._t0 = start_time
        self._stop_demand_since: Optional[float] = None
        self.tripped: bool = False
        self.reason: str = ""
        # A running record of a configuration mismatch with the gate. Refreshed
        # every tick (even once tripped) so the status/log keep reporting it; it
        # never changes the SAFE cancel behaviour, it only surfaces the misconfig.
        self.misconfig: Optional[str] = None

    @property
    def effective_stop_effect_deadline_s(self) -> float:
        return self.p.effective_stop_effect_deadline_s

    def check_config(self, gate_stop_strategy: Optional[str]) -> Optional[str]:
        """Cross-check the gate's ANNOUNCED strategy (from /human_gate/decision)
        against this watcher's OWN configuration, using no shared state. Returns a
        loud reason string on a mismatch (or on pendant_pause running while this
        watcher's deadline is too short to outlast the freeze), else None. This is
        pure reporting: the watcher keeps cancelling at its own effective deadline
        regardless, so the arm still ends stopped."""
        eff = self.effective_stop_effect_deadline_s
        if gate_stop_strategy is not None and gate_stop_strategy != self.p.stop_strategy:
            return (f"stop_strategy MISCONFIG: gate is running "
                    f"'{gate_stop_strategy}' but this independent watcher is "
                    f"configured '{self.p.stop_strategy}' — the single freeze-confirm "
                    f"knob was not applied to both processes. Watcher stays SAFE "
                    f"(cancels at its own {eff:.2f}s deadline); fix params.yaml so "
                    f"both processes share stop_strategy/pendant_freeze_confirm_s.")
        if gate_stop_strategy == STRAT_PENDANT_PAUSE and eff < self.p.pendant_freeze_confirm_s:
            return (f"stop_strategy MISCONFIG: pendant_pause is active but this "
                    f"watcher's stop-effect deadline {eff:.2f}s is shorter than the "
                    f"freeze-confirm {self.p.pendant_freeze_confirm_s:.2f}s — it would "
                    f"cancel a pause before it freezes. Watcher stays SAFE; widen the "
                    f"watcher's pendant_freeze_confirm_s to match the gate.")
        return None

    def update(
        self,
        now: float,
        *,
        last_state_arrival: Optional[float],
        last_gate_arrival: Optional[float],
        state_zone: Optional[str],
        gate_action: Optional[str],
        observed_scaling: Optional[float],
        gate_demand: Optional[str] = None,
        gate_stop_strategy: Optional[str] = None,
    ) -> Optional[str]:
        """Return a cancel reason if the watcher should cancel this tick (newly or
        still tripped), else None."""
        # Refresh the configuration cross-check first so a mismatch is visible even
        # while tripped/latched. It never gates the cancel decision below.
        self.misconfig = self.check_config(gate_stop_strategy)

        if self.tripped:
            return self.reason

        past_grace = (now - self._t0) >= self.p.startup_grace_s

        # --- gate explicitly asked for the backstop ---------------------- #
        if gate_action == ACTION_BACKSTOP:
            return self._trip("gate decided BACKSTOP")

        # --- camera-state heartbeat (its OWN copy) ----------------------- #
        # "Never received past grace" is a startup/discovery condition, not a
        # fault: it cancels (harmless if no goal exists) but does NOT latch, so it
        # recovers the moment the state starts flowing. Losing a heartbeat we HAD
        # been receiving is a genuine fault and latches.
        if last_state_arrival is None:
            if past_grace:
                return "no camera-state received yet (past startup grace)"
        elif (now - last_state_arrival) > self.p.state_heartbeat_timeout_s:
            return self._trip(
                f"camera-state heartbeat lost "
                f"({(now - last_state_arrival) * 1e3:.0f} ms silent)")

        # --- gate decision heartbeat ------------------------------------- #
        if last_gate_arrival is None:
            if past_grace:
                return "gate has not published a decision yet (past startup grace)"
        elif (now - last_gate_arrival) > self.p.gate_heartbeat_timeout_s:
            return self._trip(
                f"gate decision heartbeat lost "
                f"({(now - last_gate_arrival) * 1e3:.0f} ms silent) — primary path dead")

        # --- STOP demanded but slider did not take effect ---------------- #
        # A hazard STOP is the camera zone STOP or the gate's DEMAND being STOP
        # (which includes a fault-forced STOP). It deliberately does NOT include
        # the gate holding STOP while dwelling toward a resume (action STOP but
        # demand CLEAR): cancelling there would discard the very trajectory the
        # slider is freezing to resume (report §2).
        stop_demanded = (state_zone == ZONE_STOP) or (gate_demand == ACTION_STOP)
        if stop_demanded:
            if self._stop_demand_since is None:
                self._stop_demand_since = now
            effected = (observed_scaling is not None
                        and observed_scaling <= self.p.stop_scaling_threshold)
            deadline = self.effective_stop_effect_deadline_s
            if (not effected
                    and (now - self._stop_demand_since) > deadline):
                sc = "unknown" if observed_scaling is None else f"{observed_scaling:.3f}"
                return self._trip(
                    f"STOP demanded but scaling still {sc} > "
                    f"{self.p.stop_scaling_threshold:.2f} after "
                    f"{deadline:.2f}s — slider ineffective")
        else:
            self._stop_demand_since = None

        return None

    def _trip(self, reason: str) -> str:
        self.tripped = True
        self.reason = reason
        return reason

    def reset(self) -> None:
        self.tripped = False
        self.reason = ""
        self._stop_demand_since = None
