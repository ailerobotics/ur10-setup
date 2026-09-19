#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Scripted scenarios for the dynamic-pick simulation proofs (report §7, §10 Phase A).

TWO perception streams are scripted here, aligned on one wall-clock timeline:

  * the HUMAN-safety state (the REAL camera wire format via
    `humangate.state_adapter.build_state_json`) — a person approaches, is close,
    is tracked leaving, vanishes close, the heartbeat drops — exactly what
    `cams/human_safety` would publish. No cameras are ever opened.
  * the PICK TARGET (a `geometry_msgs/PointStamped` in the base frame) — a target
    that is repositioned repeatedly during RUN windows (to trigger preemptions),
    is ALSO moved while the gate is STOP (to prove the acceptance shim forwards
    nothing), and is occluded/lost for a window (to exercise the tracker's
    hold-not-chase behaviour).

The narrative (`dynamic_pick_full`, ~52 s), which the analyzer asserts against:

  0-6   CLEAR            discovery + gate dwell -> RUN; target static at A
  6-20  CLEAR            RUN window: target STEPS +x repeatedly -> PREEMPTIONS
                         (this is the goal-preemption-latency / re-target-rate window)
  20-22 SLOW  (~2.4 m)   person approaching mid-leg -> shim HOLDS new goals
  22-28 STOP  (~1.0 m)   person close -> gate cancels/holds; target STILL moves ->
                         the shim must forward NOTHING (no motion while not RUN)
  28-30 SLOW  (~2.3 m)   tracked leaving
  30-38 CLEAR            RUN (keep-moving): plan AFRESH to the FRESHEST target
  38-40 STOP  (~0.8 m)   close again
  40-41.2 (silence)      heartbeat LOST -> STOP
  41.2-45 CLEAR          recover -> RUN
  45-47 STOP  (~0.6 m)   very close
  47-52 CLEAR vanish     person VANISHES close (STOP->CLEAR direct) -> LOST_CLOSE ->
                         gate must NOT resume; the shim forwards nothing
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


def _clear():
    return dict(zone="CLEAR", distance_status="none", cameras=[_FRESH_CAM], localised=False)


def _slow(dist):
    return dict(zone="SLOW", distance_status="known", nearest_distance_base=dist,
                any_person_visible=True, cameras=[_FRESH_CAM], localised=True,
                reasons=[f"nearest person id=1 at {dist:.2f} m from base"])


def _stop(dist):
    return dict(zone="STOP", distance_status="known", nearest_distance_base=dist,
                any_person_visible=True, cameras=[_FRESH_CAM], localised=True,
                reasons=[f"nearest person id=1 at {dist:.2f} m from base"])


def human_full():
    """Human-safety phases: (start_s, end_s, spec). `spec` with {"silence": True}
    means publish nothing (heartbeat loss). The human and target clocks share a t0
    (both publishers start together, after the sim-bootstrap prepose), so the
    events below line up with the target motion described in `target_full`."""
    return [
        (0.0, 8.0, _clear()),        # discovery (~2 s) + gate dwell (2 s) -> RUN ~4 s
        (8.0, 24.0, _clear()),       # long RUN window: the target steps -> PREEMPTIONS
        (24.0, 26.0, _slow(2.4)),    # person approaching mid-leg -> shim HOLDS new goals
        (26.0, 32.0, _stop(1.0)),    # person close -> gate cancels/holds; target STILL moves
        (32.0, 34.0, _slow(2.3)),    # tracked leaving
        (34.0, 42.0, _clear()),      # RUN (keep-moving): plan AFRESH to the freshest target
        (42.0, 44.0, _stop(0.8)),    # close again
        (44.0, 45.2, {"silence": True}),   # heartbeat LOST -> STOP
        (45.2, 49.0, _clear()),      # recover -> RUN
        (49.0, 51.0, _stop(0.6)),    # very close
        (51.0, 56.0, _clear()),      # direct STOP->CLEAR = vanish-close -> LOST_CLOSE (no resume)
    ]


# Pick-target waypoints: (time_s, spec). spec is either {"xyz": [x,y,z]} or
# {"occluded": True} (publish nothing -> tracker coasts then LOST). The target
# holds the last xyz until the next waypoint. Steps in the RUN window are sized so
# a leg (at the 5 % cap) is still running when the target next moves -> the running
# leg is PREEMPTED (that is the behaviour under measurement).
def target_full():
    wp = [(0.0, {"xyz": [0.45, -0.22, 0.25]})]
    # RUN window (controller sees RUN ~4-24 s): the target PING-PONGS between two
    # far-apart points every 2.5 s. Each jump (~0.40 m) is a long leg at the 5 %
    # cap (several seconds), MUCH longer than the 2.5 s jump interval — so the next
    # jump always lands while the previous leg is still running, and the running leg
    # is PREEMPTED. The arm never reaches either point: it stays in a genuine
    # follow-and-preempt regime, which is exactly what the goal-preemption-latency
    # and re-target-rate measurement needs. (A small continuous drift would produce
    # only short legs that finish before the next move, which does not exercise
    # preemption — the whole point of C1.)
    A = [0.45, -0.22, 0.25]
    B = [0.85, 0.26, 0.48]
    t = 6.0
    for k in range(7):
        wp.append((t, {"xyz": (B if k % 2 == 0 else A)}))
        t += 2.5
    # during the STOP window (26-32) the target STILL moves — the shim must forward
    # NOTHING, so this produces no motion.
    wp.append((28.0, {"xyz": [0.55, 0.20, 0.40]}))
    # after resume (34-42): the freshest target is the moved one; then a couple of steps.
    wp.append((35.0, {"xyz": [0.58, 0.24, 0.42]}))
    wp.append((37.0, {"xyz": [0.62, 0.26, 0.44]}))
    wp.append((39.0, {"xyz": [0.66, 0.28, 0.46]}))
    # occlusion window: the target disappears 43-47 -> tracker OCCLUDED then LOST
    wp.append((43.0, {"occluded": True}))
    wp.append((47.0, {"xyz": [0.66, 0.28, 0.46]}))
    return wp


def target_cumotion():
    """Target scenario for the cuMotion proof: the SAME ping-pong + STOP-move +
    occlusion story, but in a SMALL Cartesian region near the parked-arm centre so
    every surrogate joint config is close to the reference pose and therefore
    collision-free in the real cell ESDF (a synthetic far-reach target would map to
    a config that collides with the mapped shelf — a real-cell pick-pose concern,
    out of this sim's scope). cuMotion still plans COLLISION-AWARE legs against the
    nvblox world (ground plane + cell) and the preemption behaviour is identical."""
    wp = [(0.0, {"xyz": [0.60, 0.00, 0.35]})]
    # moderate (non-degenerate) deltas around the mid-workspace centre so cuMotion
    # trajopt has a real trajectory to optimise, while staying within the reachable
    # window the arm occupies collision-free.
    A = [0.48, -0.16, 0.26]
    B = [0.72, 0.16, 0.46]
    t = 6.0
    for k in range(7):
        wp.append((t, {"xyz": (B if k % 2 == 0 else A)}))
        t += 3.0                                   # cuMotion solves take ~0.1-0.5 s
    wp.append((28.0, {"xyz": [0.55, 0.10, 0.40]}))    # moves during STOP
    wp.append((35.0, {"xyz": [0.66, 0.12, 0.42]}))
    wp.append((37.0, {"xyz": [0.50, -0.10, 0.30]}))
    wp.append((43.0, {"occluded": True}))
    wp.append((47.0, {"xyz": [0.50, -0.10, 0.30]}))
    return wp


HUMAN_SCENARIOS = {"dynamic_pick_full": human_full, "dynamic_pick_cumotion": human_full}
TARGET_SCENARIOS = {"dynamic_pick_full": target_full, "dynamic_pick_cumotion": target_cumotion}
SCENARIO_DURATION = {"dynamic_pick_full": 56.0, "dynamic_pick_cumotion": 56.0}


def human_state_json(spec: dict, now_wall: float, seq: int) -> str:
    localised = spec.get("localised", False)
    evidence = now_wall if localised else None
    return sa.build_state_json(
        spec["zone"], stamp=now_wall, evidence_stamp=evidence, heartbeat_seq=seq,
        distance_status=spec.get("distance_status", "none"),
        nearest_distance_base=spec.get("nearest_distance_base"),
        any_person_visible=spec.get("any_person_visible", False),
        reasons=spec.get("reasons"), cameras=spec.get("cameras"))
