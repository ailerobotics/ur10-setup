#!/usr/bin/env python3
"""
The robot-side human-safety gate — decision core (report human-safety-s1 §2/§5/§7).

Given a stream of camera-side CameraState samples and driver feedback, this
decides what the robot SHOULD do:

  RUN       run the planned trajectory at the operating speed fraction
  SLOW      scale the running trajectory down to a low speed fraction
  STOP      scale the speed fraction to zero (freeze on the planned path)
  BACKSTOP  cancel the trajectory goal so the arm decelerates to a hold

The core is PURE and DETERMINISTIC: standard library + the pure gate modules
only, no rclpy, no wall-clock reads of its own (all time is passed in). This is
what lets every rule be unit-tested offline and replayed. The core NEVER touches
the robot; it only returns a Decision, which the node hands to a RecorderPort.

Design invariants, all fail-toward-stop (report §5, §7.4):
  * absence of evidence is NEVER clearance;
  * RUN is reachable ONLY through an explicit healthy CLEAR held for the full
    dwell the gate observes ITSELF (defence in depth over the camera-side dwell);
  * an unknown distance is never treated as "far" (distance is not on the path
    to RUN at all — RUN comes only from an explicit healthy CLEAR zone);
  * a person LOST while close to the robot does not enable a resume; only a
    person TRACKED leaving does (report §5 #9);
  * after a STOP the gate escalates to BACKSTOP if the arm is not confirmed
    stopped within a deadline, or on any gate self-fault (report §2, §5 #15).
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_state import (  # noqa: E402
    HEALTH_TRACK_LOST_CLOSE,
    ZONE_CLEAR,
    ZONE_SLOW,
    ZONE_STOP,
    CameraState,
)
from gate_config import GateConfig  # noqa: E402

# --- the gate's own action vocabulary ---
RUN = "RUN"
SLOW = "SLOW"
STOP = "STOP"
BACKSTOP = "BACKSTOP"
ACTIONS = (RUN, SLOW, STOP, BACKSTOP)

# --- how the most recent CLEAR was reached (drives the resume refusal) ---
PROV_CLEAN_START = "CLEAN_START"     # no person had been close since startup
PROV_TRACKED_LEAVING = "TRACKED_LEAVING"  # person tracked out (…->SLOW->CLEAR)
PROV_LOST_CLOSE = "LOST_CLOSE"       # person vanished while close (…STOP->CLEAR)


@dataclass(frozen=True)
class Decision:
    """One gate decision, everything the node needs to command + log."""

    action: str          # one of ACTIONS
    reason: str          # human-readable justification
    speed_fraction: Optional[float]  # slider fraction for RUN/SLOW/STOP; None for BACKSTOP
    demand: str          # the pre-dwell/pre-backstop demand (STOP/SLOW/CLEAR)
    clear_dwell_s: float # seconds of continuous trusted CLEAR observed so far
    provenance: str      # how the current/last CLEAR was reached
    healthy: bool        # was a fresh, healthy state in hand this tick


class HumanGate:
    """Stateful decision core. Feed it states with on_state(); ask decide()."""

    def __init__(self, config: Optional[GateConfig] = None) -> None:
        self.cfg = (config or GateConfig()).validate()
        # latest received state + liveness bookkeeping
        self._latest: Optional[CameraState] = None
        self._latest_arrival: Optional[float] = None
        self._ever_received = False
        self._last_seq: Optional[int] = None
        self._last_seq_change: Optional[float] = None
        # decision state
        self._clear_since: Optional[float] = None
        self._stop_since: Optional[float] = None
        self._backstop_latched = False
        self._self_fault: Optional[str] = None
        # resume-provenance tracking
        self._provenance = PROV_CLEAN_START
        self._prev_zone: Optional[str] = None
        self._lost_close_flag_seen = False

    # ---- inputs -----------------------------------------------------------

    def on_state(self, state: CameraState, now: float,
                 now_wall: Optional[float] = None) -> None:
        """Ingest one freshly-arrived camera state (now = monotonic seconds;
        now_wall = the wall/ROS clock the evidence stamp is in, used for the
        per-message staleness/skew part of the dwell-continuity check)."""
        self._ever_received = True
        self._latest = state
        self._latest_arrival = now
        # heartbeat advancement bookkeeping
        if self._last_seq is None or state.heartbeat_seq != self._last_seq:
            self._last_seq = state.heartbeat_seq
            self._last_seq_change = now
        # resume-provenance: classify the STOP->CLEAR transition character.
        self._update_provenance(state)
        self._prev_zone = state.zone
        # Dwell continuity is a property of EVERY received message, not only of
        # the states sampled at decide ticks: any message that is not a fresh,
        # healthy CLEAR breaks the continuous-CLEAR run and resets the resume
        # timer at once, even if a later CLEAR overwrites it before the next tick.
        if not self._is_trusted_clear(state, now_wall):
            self._clear_since = None

    def inject_self_fault(self, reason: str) -> None:
        """The node flags an internal fault (e.g. its decide loop overran or an
        exception was caught). Forces BACKSTOP until a healthy resume (report §5
        #11)."""
        self._self_fault = reason

    def _is_trusted_clear(self, state: CameraState, now_wall: Optional[float]) -> bool:
        """A state that counts toward the continuous healthy-CLEAR dwell: a CLEAR
        zone, fully healthy, and — when the wall clock is known — neither stale
        nor skewed into the future. Anything else breaks the dwell."""
        if not state.is_clear:
            return False
        if now_wall is not None:
            age = now_wall - state.stamp
            if age > self.cfg.state_max_age_s:
                return False
            if age < -self.cfg.clock_skew_tolerance_s:
                return False
        return True

    # ---- provenance -------------------------------------------------------

    def _update_provenance(self, state: CameraState) -> None:
        # Remember a lost-close signal seen while a person is NOT clear, so a
        # later CLEAR cannot be trusted even if it arrives a few frames later.
        if state.zone != ZONE_CLEAR and HEALTH_TRACK_LOST_CLOSE in state.health_flags:
            self._lost_close_flag_seen = True

        if state.zone != ZONE_CLEAR:
            return  # provenance only changes at the moment CLEAR is (re)entered
        if self._prev_zone == ZONE_CLEAR:
            return  # a run of CLEARs is not a transition; keep the provenance

        # transition INTO CLEAR: classify how we got here, using only the
        # just-ended non-clear episode. The lost-close flag is scoped to that
        # episode and is cleared below once it has been consumed, so a LATER
        # genuine tracked departure can still reach the TRACKED_LEAVING branch.
        if self._prev_zone == ZONE_STOP or self._lost_close_flag_seen:
            # jumped straight from STOP to CLEAR, or a lost-close flag was raised
            # during the episode -> the person was lost close, not tracked out.
            # This provenance holds through the whole clear run; it is only
            # displaced by a subsequent …->SLOW->CLEAR tracked departure.
            self._provenance = PROV_LOST_CLOSE
        elif self._prev_zone == ZONE_SLOW:
            # …->SLOW->CLEAR: a genuine tracked departure. This is the ONLY way
            # a prior LOST_CLOSE can be cleared without an operator reset.
            self._provenance = PROV_TRACKED_LEAVING
        else:  # self._prev_zone is None: the first-ever state is CLEAR
            self._provenance = PROV_CLEAN_START
        # the flag described exactly the episode just classified; drop it so the
        # next non-clear episode is judged on its own evidence.
        self._lost_close_flag_seen = False

    # ---- decision ---------------------------------------------------------

    def _demand(self, now: float, now_wall: float) -> tuple[str, bool, str]:
        """Compute the pre-dwell demand (STOP/SLOW/CLEAR), whether a fresh
        healthy state is in hand, and a reason. Everything here fails to STOP."""
        c = self.cfg
        if not self._ever_received or self._latest is None or self._latest_arrival is None:
            return STOP, False, "startup: no camera-state received yet"
        if (now - self._latest_arrival) > c.heartbeat_timeout_s:
            dt_ms = (now - self._latest_arrival) * 1e3
            return STOP, False, f"heartbeat timeout: no state for {dt_ms:.0f} ms"
        if self._last_seq_change is not None and \
                (now - self._last_seq_change) > c.heartbeat_timeout_s:
            return STOP, False, "heartbeat_seq stalled (publisher stuck)"

        s = self._latest
        age = now_wall - s.stamp
        if age > c.state_max_age_s:
            return STOP, False, f"stale state: evidence {age * 1e3:.0f} ms old"
        if age < -c.clock_skew_tolerance_s:
            return STOP, False, f"clock skew: stamp {(-age) * 1e3:.0f} ms in the future"

        # fresh + live. Health forces non-CLEAR. The camera side forces a
        # non-clear zone on any fault it detects, so a fault reported alongside a
        # CLEAR zone is internally inconsistent and cannot be trusted -> STOP
        # (fail toward stop on anything the gate cannot confirm). Only a fault
        # WITH a SLOW zone (camera judged itself degraded but is still tracking)
        # is trusted enough to stay SLOW; a fault with STOP stays STOP.
        if not s.health_ok or s.health_flags:
            if s.zone == ZONE_SLOW:
                return SLOW, True, f"health fault, camera still tracking (SLOW) {sorted(s.health_flags)}"
            return STOP, True, f"health fault forcing STOP {sorted(s.health_flags)}"
        if s.zone == ZONE_STOP:
            return STOP, True, self._zone_reason("STOP", s)
        if s.zone == ZONE_SLOW:
            return SLOW, True, self._zone_reason("SLOW", s)
        return ZONE_CLEAR, True, "camera reports CLEAR and healthy"

    @staticmethod
    def _zone_reason(zone: str, s: CameraState) -> str:
        d = "unknown" if not s.distance_known else f"{s.nearest_distance_m:.2f} m"
        return f"camera zone {zone}, nearest person {d}"

    def decide(self, now: float, now_wall: float,
               driver_stopped: Optional[bool] = None) -> Decision:
        """Produce a Decision. `now` is monotonic seconds (timeouts/dwell);
        `now_wall` is the clock the evidence stamp is in (staleness/skew);
        `driver_stopped` is the injected feedback (True/False/None)."""
        c = self.cfg
        demand, healthy, reason = self._demand(now, now_wall)

        # --- map demand -> primary decision (RUN/SLOW/STOP) ---
        if demand == ZONE_CLEAR:
            if self._provenance == PROV_LOST_CLOSE:
                self._clear_since = None
                primary, reason = STOP, (
                    "CLEAR not trusted: last person was lost close to the robot, "
                    "not tracked leaving — holding until a clean tracked departure")
            else:
                if self._clear_since is None:
                    self._clear_since = now
                dwell = now - self._clear_since
                if dwell >= c.clear_dwell_s:
                    primary = RUN
                    reason = f"CLEAR healthy for {dwell:.2f} s >= dwell {c.clear_dwell_s:.2f} s"
                else:
                    primary = STOP
                    reason = (f"CLEAR dwell {dwell:.2f}/{c.clear_dwell_s:.2f} s — "
                              "holding until the dwell completes")
        else:
            # STOP or SLOW: never accumulating a resume dwell.
            self._clear_since = None
            primary = demand

        dwell_s = 0.0 if self._clear_since is None else (now - self._clear_since)
        provenance = self._provenance

        # --- backstop deadline bookkeeping ---
        # Only a HAZARD stop (demand == STOP: a person or a fault requires the
        # arm to stop) arms the fall-through. A STOP-hold while dwelling toward a
        # resume (demand == CLEAR) must NEVER escalate to BACKSTOP: cancelling the
        # goal there would discard the very trajectory the primary lever exists
        # to freeze and resume (report §2).
        hazard_stop = (demand == STOP)
        if hazard_stop:
            if self._stop_since is None:
                self._stop_since = now
        else:
            self._stop_since = None

        # --- primary -> BACKSTOP fall-through + self-fault (report §2, §5 #15) ---
        if primary == RUN:
            # a full trusted clear+dwell is the only de-escalation from BACKSTOP;
            # note the resume needs a fresh plan if a cancel had fired.
            if self._backstop_latched:
                reason = reason + " (resume requires a fresh plan after backstop)"
            self._backstop_latched = False
            self._self_fault = None
            action = RUN
            speed = c.operating_speed_fraction
        else:
            action = primary
            speed = c.slow_speed_fraction if primary == SLOW else c.stop_speed_fraction
            if self._self_fault is not None:
                self._backstop_latched = True
                reason = f"gate self-fault: {self._self_fault}"
            if hazard_stop and self._stop_since is not None and \
                    (now - self._stop_since) > c.backstop_deadline_s and \
                    driver_stopped is not True:
                self._backstop_latched = True
                conf = "still moving" if driver_stopped is False else "unconfirmed"
                reason = (f"primary STOP not confirmed within "
                          f"{c.backstop_deadline_s:.2f} s (arm {conf}) — escalating to backstop")
            if self._backstop_latched:
                action = BACKSTOP
                speed = None

        return Decision(
            action=action, reason=reason, speed_fraction=speed, demand=demand,
            clear_dwell_s=dwell_s, provenance=provenance, healthy=healthy,
        )
