#!/usr/bin/env python3
"""
Verify a saved cell map against geometry we already know - reproducibly.

The 2026-08-28 map was checked by hand and the numbers typed into its
descriptor. That is fine once and unreproducible thereafter, so the same four
checks live here. Run it on the .ply the scan writes; it prints the descriptor
table directly.

(a) TABLE HEIGHT   the dominant horizontal plane. Known independently: the two
    AprilTag boards were measured by camera at z = -0.053 / -0.054 m, so the
    table surface should come out within a board thickness plus a voxel.
(b) SHELF FRONT    vertical structure beyond y 0.8. The approach demo measured
    the gondola front at y = 1.516 m at x ~ 0.
(c) ARM ABSENT     nearest mapped point to each arm link origin, read LIVE from
    TF. If the robot segmenter worked, the arm is not in its own map, and the
    only near points should be the table and the mounting pedestal.
(d) COVERAGE       extent and point count, for the descriptor.

Checks (a) and (b) are the ones that would catch a bad calibration: a wrong
mount tilts and offsets every surface, so a table that is not flat at the known
height means STOP, do not ship the map.

  python3 verify_map.py ~/ur10/cumotion/maps/cell-2026-09-03.ply
  python3 verify_map.py <ply> --no-tf        # skip the live arm check
"""
import argparse
import itertools
import sys

import numpy as np


def read_ply(path):
    f = open(path, 'r')
    n, props = None, []
    for line in f:
        line = line.strip()
        if line.startswith('element vertex'):
            n = int(line.split()[-1])
        elif line.startswith('property'):
            props.append(line.split()[-1])
        elif line == 'end_header':
            break
    cols = {p: i for i, p in enumerate(props)}
    arr = np.loadtxt(itertools.islice(f, n), usecols=[cols['x'], cols['y'], cols['z']])
    return arr.reshape(-1, 3)


def link_origins():
    """Arm link origins in base_link, live from TF."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    import tf2_ros
    import time
    rclpy.init()
    n = Node('verify_map')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    out = {}
    for link in ('shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link',
                 'wrist_2_link', 'wrist_3_link', 'tool0'):
        try:
            t = buf.lookup_transform('base_link', link, Time()).transform.translation
            out[link] = np.array([t.x, t.y, t.z])
        except Exception:                                        # noqa: BLE001
            pass
    rclpy.shutdown()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ply')
    ap.add_argument('--no-tf', action='store_true')
    ap.add_argument('--tag-z', type=float, default=-0.0535,
                    help='independently measured tag-board height, m')
    ap.add_argument('--shelf-y', type=float, default=1.516,
                    help='independently measured gondola front, m')
    a = ap.parse_args()

    P = read_ply(a.ply)
    print(f'{a.ply}: {len(P)} vertices')
    print(f'  extent x {P[:,0].min():+.2f}..{P[:,0].max():+.2f}  '
          f'y {P[:,1].min():+.2f}..{P[:,1].max():+.2f}  '
          f'z {P[:,2].min():+.2f}..{P[:,2].max():+.2f} m')

    # (a) table: densest 1 cm z-bin among plausible table heights
    band = P[(P[:, 2] > -0.40) & (P[:, 2] < 0.30)]
    if len(band):
        h, e = np.histogram(band[:, 2], bins=np.arange(-0.40, 0.30, 0.01))
        i = int(np.argmax(h))
        z0, z1 = e[i], e[i + 1]
        d = abs((z0 + z1) / 2 - a.tag_z) * 1000
        print(f'\n(a) table height : dominant plane z {z0:+.3f}..{z1:+.3f} m '
              f'({h[i]} pts) vs tag boards at {a.tag_z:+.4f} -> {d:.0f} mm')
        print('    (board thickness + one 20 mm voxel is expected; > ~40 mm means STOP)')

    # (b) vertical faces beyond y 0.8, listed in ASCENDING y - not ranked by
    # density. On this cell the wall behind the gondola is far denser than the
    # shelf front (cell-2026-08-28: front 1.36-1.52, wall 1.96-1.98), so ranking
    # by density hides the shelf entirely and names the wall as 'the' face.
    # What matters for collision avoidance is the FIRST face the arm meets.
    far = P[(P[:, 1] > 0.8) & (P[:, 2] > 0.0)]
    if len(far):
        edges = np.arange(0.8, far[:, 1].max() + 0.02, 0.02)
        h, e = np.histogram(far[:, 1], bins=edges)
        thr = max(h.max() * 0.10, 200)
        peaks = [i for i in range(len(h)) if h[i] >= thr]
        print(f'\n(b) vertical faces beyond y 0.8 (ascending; bins >= {thr:.0f} pts):')
        first = None
        for k, i in enumerate(peaks):
            mid = (e[i] + e[i + 1]) / 2
            if first is None:
                first = mid
            if k < 6:
                print(f'      y {e[i]:.2f}..{e[i+1]:.2f}  {h[i]:6d} pts')
            elif k == 6:
                print(f'      ... and {len(peaks) - 6} further bins out to y {e[peaks[-1]+1]:.2f}')
        if first is not None:
            print(f'    FIRST face at y ~ {first:.2f} m vs measured gondola front '
                  f'{a.shelf_y:.3f} -> {abs(first - a.shelf_y)*1000:.0f} mm')
            print('    (the densest bins further out are the wall behind the shelf)')
    else:
        print('\n(b) shelf front  : nothing mapped beyond y 0.8 (not swept that far)')

    # (c) arm absence
    if not a.no_tf:
        origins = link_origins()
        if origins:
            print('\n(c) arm absent   : nearest mapped point to each live link origin')
            for link, c in origins.items():
                dmin = float(np.linalg.norm(P - c, axis=1).min())
                print(f'      {link:16s} {dmin:.3f} m')
            print('    (link radii are 0.06-0.08 m; anything under ~0.15 m on a MOVING')
            print('     link means the segmenter let the arm into its own map)')
        else:
            print('\n(c) arm absent   : no TF available, skipped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
