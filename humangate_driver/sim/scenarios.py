#!/usr/bin/env python3
"""
Scripted camera-side scenarios for the end-to-end simulation proofs (task brief
item 4). These drive the REAL camera wire format (humangate.state_adapter.build_
state_json) so the gate sees exactly what `cams/human_safety` would publish.

A scenario is a list of phases; each phase covers a wall-clock window
[start, end) relative to the publisher's start and says what to publish during it
(a zone, a distance, a health condition) — or to go SILENT (heartbeat loss), or
to publish a STALE-stamped state (a latched camera frame). The publisher samples
at a fixed rate and emits the active phase's state with a fresh stamp and an
advancing heartbeat_seq.

The default `full` scenario walks the whole story the brief asks to be shown:
  approach -> hover at the boundary -> tracked departure -> resume same motion
  -> STOP -> heartbeat lost -> recover -> vanish CLOSE (must NOT resume)
  -> fault reported alongside a CLEAR zone (must STOP) -> final clear.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "humangate"))
import state_adapter as sa  # noqa: E402

_FRESH_CAM = {"camera": "sim", "status": "fresh", "present_in_graph": True,
              "detector_alive": True, "positioned": True, "fresh": True,
              "frame_age": 0.0, "note": ""}
_STALE_CAM = {"camera": "sim", "status": "stale", "present_in_graph": True,
              "detector_alive": True, "positioned": True, "fresh": False,
              "frame_age": 3.0, "note": "stale/no frame"}


def _clear():
    return dict(zone="CLEAR", distance_status="none", cameras=[_FRESH_CAM],
                localised=False)


def _slow(dist):
    return dict(zone="SLOW", distance_status="known", nearest_distance_base=dist,
                any_person_visible=True, cameras=[_FRESH_CAM], localised=True,
                reasons=[f"nearest person id=1 at {dist:.2f} m from base"])


def _stop(dist, lost_close=False):
    reasons = [f"nearest person id=1 at {dist:.2f} m from base"]
    if lost_close:
        reasons = [f"track id=1 lost while close ({dist:.2f} m) -> hold STOP"]
    return dict(zone="STOP", distance_status="known", nearest_distance_base=dist,
                any_person_visible=True, cameras=[_FRESH_CAM], localised=True,
                reasons=reasons)


def _slow_presence():
    # someone visible only on a presence-only (uncalibrated) camera: distance
    # unknown, camera side forces SLOW. The gate must SLOW, never treat as far.
    return dict(zone="SLOW", distance_status="unknown", any_person_visible=True,
                cameras=[_FRESH_CAM], localised=False,
                reasons=["person visible without a trustworthy position -> SLOW"])


def _fault_clear():
    # a health fault reported ALONGSIDE a CLEAR zone (a stale camera): an internal
    # contradiction the gate must resolve toward STOP.
    return dict(zone="CLEAR", distance_status="none", cameras=[_STALE_CAM],
                localised=False, reasons=["camera 'sim' slow -> SLOW"])


def full():
    """The full narrative. Times are seconds from publisher start."""
    return [
        (0.0, 4.0, _clear()),          # settle + dwell -> RUN (motion at op speed)
        (4.0, 6.0, _slow(2.4)),        # person approaching -> SLOW
        (6.0, 11.0, _stop(1.2)),       # person close -> STOP (freeze on path)
        (11.0, 13.5, _slow(2.3)),      # TRACKED leaving -> SLOW
        (13.5, 18.0, _clear()),        # CLEAR + dwell -> RESUME same trajectory
        (18.0, 20.0, _stop(1.0)),      # close again -> STOP
        (20.0, 21.2, {"silence": True}),  # heartbeat LOST -> STOP (watchdog)
        (21.2, 24.0, _clear()),        # recover -> dwell -> RUN
        (24.0, 26.0, _slow(2.2)),
        (26.0, 28.0, _stop(0.6)),      # very close
        (28.0, 33.0, dict(**_clear(), vanish_close=True)),  # person VANISHES (STOP->CLEAR direct); must NOT resume
        (33.0, 35.0, _slow_presence()),  # presence-only person -> SLOW, never RUN
        (35.0, 37.0, {"stale": True}),   # latched/stale evidence stamp -> STOP
        (37.0, 40.0, _fault_clear()),    # fault reported with CLEAR zone -> STOP
        (40.0, 44.0, _clear()),          # finally genuinely clear -> RUN
    ]


def pendant_pause_demo():
    """A clean freeze-and-continue narrative for the PENDANT-PAUSE gate e2e: settle
    to RUN and let the mover get well into its motion FIRST, then a person walks close
    (STOP -> the gate PAUSES the loaded program, the arm freezes MID-motion on its
    planned path), then a genuine tracked departure + CLEAR dwell (the gate PLAYS the
    program -> the SAME trajectory continues to completion). The long initial CLEAR
    covers DDS discovery (~2 s) + the gate's 2 s dwell so RUN + real motion happen well
    before the STOP; the STOP window is long enough to exceed the pendant freeze
    confirmation time so the pause fully takes effect. No heartbeat loss and no
    vanish-close during motion, so nothing but the pause acts on the running goal."""
    return [
        (0.0, 12.0, _clear()),         # discovery + dwell -> RUN, then the mover moves
        (12.0, 14.0, _slow(2.4)),      # person approaching -> SLOW
        (14.0, 30.0, _stop(1.0)),      # person close -> STOP -> PAUSE (freeze MID-motion)
        (30.0, 32.0, _slow(2.3)),      # TRACKED leaving -> SLOW (still frozen)
        (32.0, 80.0, _clear()),        # CLEAR + dwell -> RESUME (play) same trajectory
    ]


SCENARIOS = {"full": full, "pendant_pause_demo": pendant_pause_demo}


def state_json_for(phase: dict, now_wall: float, seq: int, *, stale_age: float = 5.0):
    """Render one phase into a camera-side wire JSON string at time `now_wall`."""
    localised = phase.get("localised", False)
    evidence = now_wall if localised else None
    return sa.build_state_json(
        phase["zone"], stamp=now_wall, evidence_stamp=evidence, heartbeat_seq=seq,
        distance_status=phase.get("distance_status", "none"),
        nearest_distance_base=phase.get("nearest_distance_base"),
        any_person_visible=phase.get("any_person_visible", False),
        reasons=phase.get("reasons"), cameras=phase.get("cameras"))


def stale_state_json(now_wall: float, seq: int, stale_age: float = 5.0):
    """A STOP state whose evidence stamp is frozen in the past (latched frame)."""
    return sa.build_state_json(
        "STOP", stamp=now_wall, evidence_stamp=now_wall - stale_age,
        heartbeat_seq=seq, distance_status="known", nearest_distance_base=1.0,
        any_person_visible=True, cameras=[_FRESH_CAM],
        reasons=["nearest person id=1 at 1.00 m from base"])
