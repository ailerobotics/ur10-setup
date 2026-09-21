# SPDX-License-Identifier: Apache-2.0
"""Concrete geometry of the simulated UR10 cell — the ONE source of truth for the
Isaac scene, the generated detector TFs (params/camera_poses.sim.yaml) and the
offline analysis. Pure data + SimCamera objects; no Isaac / ROS imports.

Frame: ``base_link`` at the world origin, +Z up, the robot column at the origin.
The person walks along -Y (in front of the robot). All lengths in metres.

Camera poses are plausible stand-ins for the real rig (wrist D435i eye-in-hand,
shelf D415 high near-frontal, table D415 wide side view). The real rig's fixed
D415s are UNcalibrated today (``params/camera_poses.yaml`` is empty), so there is
no measured extrinsic to copy; these are documented, plausible poses chosen so
the near approach band (~1.1-3.3 m) is well covered by at least two cameras.
"""
from __future__ import annotations

from typing import List, Tuple

from sim_poses import SimCamera

# --- robot + furniture stand-ins (boxes) ----------------------------------- #
# (centre_xyz, half_extents_xyz). The robot is a primitive stand-in (the brief
# permits this); it only has to occupy the cell, not be detected.
ROBOT_BASE = ((0.0, 0.0, 0.30), (0.18, 0.18, 0.30))    # pedestal
ROBOT_COLUMN = ((0.0, 0.0, 0.85), (0.09, 0.09, 0.55))  # arm column stand-in
SHELF_BOX = ((0.95, -1.0, 0.9), (0.10, 0.6, 0.9))      # gondola shelf
TABLE_BOX = ((1.25, -1.9, 0.40), (0.5, 0.35, 0.40))    # side table

# --- cameras (see module docstring) ---------------------------------------- #
CAMERAS: List[SimCamera] = [
    # Wrist D435i eye-in-hand stand-in: near the arm, looking forward-down the
    # corridor. Static in sim (the arm does not move in this test).
    SimCamera(role="wrist", optical_frame="wrist_color_optical_frame",
              eye=(0.12, -0.30, 1.45), target=(0.12, -2.3, 0.35)),
    # Shelf D415: high, near-frontal, behind/above the robot looking down -Y.
    SimCamera(role="shelf", optical_frame="shelf_color_optical_frame",
              eye=(0.75, 0.55, 2.05), target=(0.15, -2.1, 0.9)),
    # Table D415: wide side view from +X across the transit volume.
    SimCamera(role="table", optical_frame="table_color_optical_frame",
              eye=(2.35, -1.7, 1.25), target=(0.05, -1.9, 0.85)),
]


def camera(role: str) -> SimCamera:
    for c in CAMERAS:
        if c.role == role:
            return c
    raise KeyError(role)


# --- person path ----------------------------------------------------------- #
# Waypoints as (x, y) on the floor; the character root (feet) follows these at a
# fixed speed, pausing at the dwell waypoint. Distances to base (origin):
#   -4.5 m start (well outside the 3.3 m SLOW ring) -> through SLOW -> ~1.15 m
#   (deep inside the 2.0 m STOP ring) -> pause -> back out. z stays 0 (on floor);
# motion is a straight teleport-interpolation, no skeletal walk cycle.
PERSON_PATH: List[Tuple[float, float]] = [
    (0.30, -4.50),   # start: 4.51 m from base
    (0.25, -3.30),   # 3.31 m: entering the SLOW ring
    (0.20, -2.40),   # 2.41 m: mid SLOW
    (0.18, -1.95),   # 1.96 m: crossing into STOP (2.0 m of base)
    (0.15, -1.15),   # 1.16 m: deep in STOP (dwell here)
]
PERSON_SPEED = 0.7            # m/s along the path
PERSON_DWELL_S = 3.0          # pause at the innermost waypoint
PERSON_BODY_HEIGHT = 1.75     # standing height (m); torso centre ~= 0.5 * this
# The detector's published body CENTRE sits at torso height; the ground-truth
# body centre for 3-D error is the root xy lifted to this z.
PERSON_CENTRE_Z = 0.95


def robot_base_xy() -> Tuple[float, float]:
    return (0.0, 0.0)
