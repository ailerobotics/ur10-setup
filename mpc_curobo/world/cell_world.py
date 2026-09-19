#!/usr/bin/env python3
"""The cell world model for the cuRobo MPC prototype (SIM ONLY).

Static obstacles first (a ground plane + a representative shelf), then a moving
capsule standing in for a person's swept volume. cuRobo consumes this as its
scene_model dict; the moving obstacle is repositioned at run time with
``mpc.scene_collision_checker.update_obstacle_pose(name, Pose)``.

IMPORTANT — this is a REPRESENTATIVE shelf, not the cell's real geometry. The
real cell obstacles live in the nvblox ESDF / the cleaned static map
(``~/ur10/cumotion/maps``); feeding that live map to the planner is option (3)
of the human-dynamic-plan-s1 report, a SEPARATE task. Here we hand-place a
plausible shelf so the MPC has real static geometry to avoid while following a
target. Poses are in base_link, metres, wxyz quaternion.
"""
from __future__ import annotations

# names of the moving obstacle(s) so the runner can address them by key
PERSON = "person"

# A feasible in-window "pick anchor" joint config (all six within the measured
# windows). Both the MPC prototype and the C1 comparison FK this config to get a
# REACHABLE pick pose (position AND tool orientation) to track — so the target is
# genuinely reachable and the comparison is fair (a fixed park-pose orientation is
# infeasible for a front reach and makes both look artificially bad).
# FK(this) = TCP approx [0.82, 0.18, 0.71] in base_link.
PICK_ANCHOR = [0.0, -1.4, 1.4, 4.2, -1.4, 0.0]


def static_world():
    """Ground plane + representative shelf (horizontal plate + back panel),
    placed around the pick-anchor TCP (~[0.82, 0.18, 0.71])."""
    return {
        "cuboid": {
            # floor: top surface at z=-0.05 (base sits on a pedestal; the arm
            # works above it). Kept below the base spheres so the floor is not the
            # binding clearance during normal work.
            "ground": {"dims": [3.0, 3.0, 0.10], "pose": [0.0, 0.0, -0.10, 1, 0, 0, 0]},
            # shelf plate the item sits on, ~0.35 m up, below the pick region.
            "shelf_plate": {"dims": [0.50, 1.20, 0.04], "pose": [0.85, 0.0, 0.35, 1, 0, 0, 0]},
            # shelf back wall behind the pick region (~1.13 m front face).
            "shelf_back": {"dims": [0.04, 1.20, 1.20], "pose": [1.15, 0.0, 0.60, 1, 0, 0, 0]},
        },
    }


# person stand-in dimensions: a 0.5 x 0.5 m footprint, 1.8 m tall box. A cuboid
# (not a capsule) because cuRobo's SceneCollision only exposes CUBOIDS as
# named, runtime-updatable obstacles (get_obstacle_names / update_obstacle_pose);
# a capsule is baked in and cannot be moved. The centre sits at z=0.9 so the box
# spans the floor to 1.8 m.
PERSON_DIMS = [0.5, 0.5, 1.8]
PERSON_Z = 0.9


def person_pose(px, py):
    return [px, py, PERSON_Z, 1.0, 0.0, 0.0, 0.0]


def world_with_person(person_start=(0.60, -0.9)):
    """Static world plus a box standing in for a person's swept volume, placed
    off to one side at start; the runner walks it across with
    update_obstacle_pose (works because it is a named cuboid)."""
    w = static_world()
    px, py = person_start
    w["cuboid"][PERSON] = {"dims": list(PERSON_DIMS), "pose": person_pose(px, py)}
    return w


# --- analytic obstacle distances, for an honest collision-margin metric ------ #
# cuRobo optimises against these same shapes; we recompute the signed distance
# from each robot collision sphere (from FK) to each obstacle so the reported
# margin is independent of the optimiser's internal cost.

def _point_box_distance(p, center, dims):
    """Distance from point p to an axis-aligned box (0 inside)."""
    dx = max(abs(p[0] - center[0]) - dims[0] / 2.0, 0.0)
    dy = max(abs(p[1] - center[1]) - dims[1] / 2.0, 0.0)
    dz = max(abs(p[2] - center[2]) - dims[2] / 2.0, 0.0)
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _point_segment_distance(p, a, b):
    """Distance from point p to segment a-b."""
    abx, aby, abz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    apx, apy, apz = p[0] - a[0], p[1] - a[1], p[2] - a[2]
    denom = abx * abx + aby * aby + abz * abz
    t = 0.0 if denom == 0 else (apx * abx + apy * aby + apz * abz) / denom
    t = min(max(t, 0.0), 1.0)
    cx, cy, cz = a[0] + t * abx, a[1] + t * aby, a[2] + t * abz
    return ((p[0] - cx) ** 2 + (p[1] - cy) ** 2 + (p[2] - cz) ** 2) ** 0.5


def min_sphere_clearance(sphere_xyzr, world):
    """Minimum signed clearance (m) between any robot sphere and any obstacle.
    sphere_xyzr: iterable of (x, y, z, r). Negative == penetration."""
    worst = float("inf")
    cuboids = world.get("cuboid", {})
    caps = world.get("capsule", {})
    for (x, y, z, r) in sphere_xyzr:
        p = (x, y, z)
        for c in cuboids.values():
            cx, cy, cz = c["pose"][0], c["pose"][1], c["pose"][2]
            d = _point_box_distance(p, (cx, cy, cz), c["dims"]) - r
            worst = min(worst, d)
        for c in caps.values():
            px, py, pz = c["pose"][0], c["pose"][1], c["pose"][2]
            a = (px + c["base"][0], py + c["base"][1], pz + c["base"][2])
            b = (px + c["tip"][0], py + c["tip"][1], pz + c["tip"][2])
            d = _point_segment_distance(p, a, b) - r - c["radius"]
            worst = min(worst, d)
    return worst
