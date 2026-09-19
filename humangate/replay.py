#!/usr/bin/env python3
"""
Replay + fault-injection harness for the human-safety gate — no ROS, no robot.

Feeds scripted sequences of camera-side states (person approaches, hovers at the
boundary, leaves; heartbeat drops; stamps go stale; distance unknown; person
vanishes close to the robot; clock skew) into a HumanGate against a virtual
clock, and returns the timeline of decisions. Every scenario in report §8 is
expressible here; test_replay.py asserts the decisions and their timing.

The harness is fully deterministic: it never reads a real clock, so a scenario
replays identically on any host. The same JSON transport the ROS integration
test uses is available via state_adapter.to_json, so a scenario can be turned
into a rosbag-style replay later without changing its definition.

PURE: standard library + the pure gate modules only.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_state import (  # noqa: E402
    HEALTH_TRACK_LOST_CLOSE,
    ZONE_CLEAR,
    ZONE_SLOW,
    ZONE_STOP,
    CameraState,
)
from gate_config import GateConfig  # noqa: E402
from gate_core import Decision, HumanGate  # noqa: E402

EPS = 1e-9


@dataclass
class Timeline:
    """A scripted scenario: messages that arrive, faults, and how the arm's
    stop feedback and wall clock behave over the run."""

    messages: List[Tuple[float, CameraState]] = field(default_factory=list)
    self_faults: List[Tuple[float, str]] = field(default_factory=list)
    duration: float = 3.0
    # monotonic->wall map; default identity (no skew). Scenarios override it.
    wall_from_mono: Callable[[float], float] = lambda t: t
    # arm-stopped feedback as a function of monotonic time.
    driver_stopped_fn: Callable[[float], Optional[bool]] = lambda t: True


def run_scenario(tl: Timeline, cfg: Optional[GateConfig] = None,
                 tick_s: Optional[float] = None) -> List[Tuple[float, Decision]]:
    """Replay a Timeline and return [(t, Decision), ...] at every tick."""
    cfg = (cfg or GateConfig()).validate()
    gate = HumanGate(cfg)
    tick = tick_s if tick_s is not None else 1.0 / cfg.decide_rate_hz

    # union of regular ticks with the exact message/fault instants
    ticks = set()
    t = 0.0
    while t <= tl.duration + EPS:
        ticks.add(round(t, 9))
        t += tick
    for mt, _ in tl.messages:
        ticks.add(round(mt, 9))
    for ft, _ in tl.self_faults:
        ticks.add(round(ft, 9))
    schedule = sorted(ticks)

    msgs = {round(mt, 9): st for mt, st in tl.messages}
    faults = {round(ft, 9): r for ft, r in tl.self_faults}

    out: List[Tuple[float, Decision]] = []
    for tt in schedule:
        if tt in msgs:
            gate.on_state(msgs[tt], tt)
        if tt in faults:
            gate.inject_self_fault(faults[tt])
        d = gate.decide(tt, tl.wall_from_mono(tt), tl.driver_stopped_fn(tt))
        out.append((tt, d))
    return out


# --- scenario builders -----------------------------------------------------

def _stream(zones_dist, start=0.0, dt=0.1, health=True, flags=(), seq_start=1,
            stamp_from_mono=None):
    """Turn a list of (zone, distance) into timed messages at `dt` spacing.

    stamp = the message's monotonic arrival time by default (fresh); override
    stamp_from_mono for stale/skew scenarios.
    """
    stamp_from_mono = stamp_from_mono or (lambda t: t)
    msgs = []
    seq = seq_start
    t = start
    for zone, dist in zones_dist:
        msgs.append((round(t, 9), CameraState(
            stamp=stamp_from_mono(t), zone=zone, nearest_distance_m=dist,
            health_ok=health, health_flags=frozenset(flags), heartbeat_seq=seq)))
        seq += 1
        t += dt
    return msgs


def approach() -> Timeline:
    """Person walks in: CLEAR -> SLOW -> STOP (gets close)."""
    seq = [(ZONE_CLEAR, None)] * 3 + [(ZONE_SLOW, 2.5)] * 3 + [(ZONE_STOP, 1.2)] * 5
    return Timeline(messages=_stream(seq), duration=1.2)


def hover_at_boundary() -> Timeline:
    """Person approaches then hovers right at the STOP ring (no chatter to RUN)."""
    seq = ([(ZONE_CLEAR, None)] * 2 + [(ZONE_SLOW, 2.2)] * 2 +
           [(ZONE_STOP, 2.05)] * 2 + [(ZONE_SLOW, 2.1)] * 2 + [(ZONE_STOP, 2.0)] * 6)
    return Timeline(messages=_stream(seq), duration=1.6)


def approach_hover_leave() -> Timeline:
    """Full arc: approach, hover close, then TRACKED leaving -> dwell -> RUN."""
    seq = ([(ZONE_CLEAR, None)] * 2 + [(ZONE_SLOW, 2.4)] * 2 + [(ZONE_STOP, 1.5)] * 4 +
           [(ZONE_SLOW, 2.3)] * 3 + [(ZONE_CLEAR, None)] * 40)  # >2 s of CLEAR to arm RUN
    # last message lands at t=5.0; end there so the scenario holds RUN rather
    # than tripping the heartbeat timeout on the trailing silence.
    return Timeline(messages=_stream(seq), duration=5.0)


def heartbeat_drop() -> Timeline:
    """Healthy CLEAR, then the state topic goes silent -> STOP within timeout."""
    seq = [(ZONE_CLEAR, None)] * 5  # messages only for the first 0.5 s
    return Timeline(messages=_stream(seq), duration=2.0)


def heartbeat_stall() -> Timeline:
    """Messages keep arriving but heartbeat_seq freezes -> STOP."""
    msgs = _stream([(ZONE_CLEAR, None)] * 30)
    frozen = []
    for i, (t, s) in enumerate(msgs):
        # freeze the seq from the 6th message onward (publisher stuck republishing)
        seq = s.heartbeat_seq if i < 6 else 6
        frozen.append((t, CameraState(
            stamp=s.stamp, zone=s.zone, nearest_distance_m=s.nearest_distance_m,
            health_ok=s.health_ok, heartbeat_seq=seq)))
    return Timeline(messages=frozen, duration=3.0)


def stale_stamps() -> Timeline:
    """Messages arrive on time but their evidence stamp is frozen in the past
    (a latched camera frame republished) -> STOP."""
    return Timeline(
        messages=_stream([(ZONE_CLEAR, None)] * 30, stamp_from_mono=lambda t: 0.0),
        duration=3.0)


def distance_unknown() -> Timeline:
    """A person seen only by presence-only cameras: zone SLOW, distance unknown.
    Must SLOW and NEVER RUN (unknown distance is not 'far')."""
    seq = [(ZONE_SLOW, None)] * 40
    return Timeline(messages=_stream(seq), duration=4.0)


def vanish_close() -> Timeline:
    """Person is close (STOP) then the track is lost: STOP -> CLEAR directly.
    The gate must STAY stopped through the whole dwell and beyond (report §5 #9)."""
    seq = [(ZONE_SLOW, 2.3)] * 2 + [(ZONE_STOP, 0.8)] * 4 + [(ZONE_CLEAR, None)] * 60
    return Timeline(messages=_stream(seq), duration=6.5)


def vanish_close_flagged() -> Timeline:
    """Same, but the camera raises TRACK_LOST_CLOSE on the last STOP frame."""
    pre = _stream([(ZONE_SLOW, 2.3)] * 2 + [(ZONE_STOP, 0.8)] * 3)
    flagged = _stream([(ZONE_STOP, 0.8)], start=0.5, seq_start=6,
                      flags=(HEALTH_TRACK_LOST_CLOSE,))
    post = _stream([(ZONE_CLEAR, None)] * 60, start=0.6, seq_start=7)
    return Timeline(messages=pre + flagged + post, duration=6.5)


def clock_skew() -> Timeline:
    """State stamps land in the future relative to the gate clock -> STOP."""
    return Timeline(
        messages=_stream([(ZONE_CLEAR, None)] * 30, stamp_from_mono=lambda t: t + 1.0),
        duration=3.0)


def stop_not_confirmed() -> Timeline:
    """Person close (STOP) and the arm never confirms it stopped -> after the
    backstop deadline the gate escalates STOP -> BACKSTOP (report §5 #15)."""
    seq = [(ZONE_STOP, 1.0)] * 40
    return Timeline(messages=_stream(seq), duration=2.0,
                    driver_stopped_fn=lambda t: False)


ALL_SCENARIOS = {
    "approach": approach,
    "hover_at_boundary": hover_at_boundary,
    "approach_hover_leave": approach_hover_leave,
    "heartbeat_drop": heartbeat_drop,
    "heartbeat_stall": heartbeat_stall,
    "stale_stamps": stale_stamps,
    "distance_unknown": distance_unknown,
    "vanish_close": vanish_close,
    "vanish_close_flagged": vanish_close_flagged,
    "clock_skew": clock_skew,
    "stop_not_confirmed": stop_not_confirmed,
}


def actions_over_time(trace: List[Tuple[float, Decision]]) -> List[Tuple[float, str]]:
    return [(t, d.action) for t, d in trace]


if __name__ == "__main__":
    for name, builder in ALL_SCENARIOS.items():
        trace = run_scenario(builder())
        actions = [d.action for _, d in trace]
        # compress consecutive equal actions for a readable summary
        seq = [a for i, a in enumerate(actions) if i == 0 or a != actions[i - 1]]
        print(f"{name:24s} -> {' '.join(seq)}")
