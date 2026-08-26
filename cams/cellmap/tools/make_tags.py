#!/usr/bin/env python3
"""
Print-ready AprilTag (36h11) anchor sheets at EXACT physical scale.

One tag per A4 page, black square = --size mm (default 120), corner labels
A B C D placed in OpenCV's detection order (A = top-left, then clockwise) so
the captain touches exactly the corners tools/tag_corners.py expects, plus a
100 mm ruler to verify the printer did not "fit to page".

Why matplotlib and not a PNG: a PDF page carries physical units, so
"print at 100 % / actual size" gives a 120.0 mm square; a PNG's size depends
on whatever DPI the print dialog assumes (a 5 % scale error is a 6 mm tag
error, i.e. 5 % range error on every anchored pose).

  python3 make_tags.py --ids 0 1 --out cams/cellmap/tags/
Mount each sheet on something rigid and flat (foam board / clipboard); a
curled sheet breaks the planar-tag assumption of the pose solver.
"""
import argparse
import os

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

A4_IN = (8.27, 11.69)
MM = 1 / 25.4


def make_sheet(tag_id, size_mm, out_path, dictionary):
    marker = cv2.aruco.drawMarker(dictionary, tag_id, 1200)   # 8x8 cells for 36h11
    fig = plt.figure(figsize=A4_IN)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, A4_IN[0] / MM)      # axes in millimetres
    ax.set_ylim(0, A4_IN[1] / MM)
    ax.set_aspect('equal')
    ax.axis('off')
    cx, cy = A4_IN[0] / MM / 2, A4_IN[1] / MM / 2 + 20
    half = size_mm / 2
    # imshow with an explicit extent -> exact millimetre placement
    ax.imshow(marker, cmap='gray', vmin=0, vmax=255, interpolation='nearest',
              extent=(cx - half, cx + half, cy - half, cy + half))
    labels = {'A': (cx - half, cy + half, 'right', 'bottom'),
              'B': (cx + half, cy + half, 'left', 'bottom'),
              'C': (cx + half, cy - half, 'left', 'top'),
              'D': (cx - half, cy - half, 'right', 'top')}
    for k, (x, y, ha, va) in labels.items():
        # Labels sit 18 mm out, diagonal from each corner: >1 tag cell (15 mm)
        # of white quiet zone, so no ink touches the quad the detector fits.
        dx = -18 if ha == 'right' else 18
        dy = 18 if va == 'bottom' else -18
        ax.text(x + dx, y + dy, k, fontsize=28, weight='bold', ha=ha, va=va)
    ax.text(cx, cy + half + 34, 'TOP', fontsize=18, ha='center', va='bottom')
    ax.annotate('', xy=(cx, cy + half + 32), xytext=(cx, cy + half + 20),
                arrowprops=dict(arrowstyle='->', lw=2))
    ax.text(cx, cy - half - 22,
            f'AprilTag 36h11  id {tag_id}   black square {size_mm:.0f} mm',
            fontsize=13, ha='center', va='top')
    ax.text(cx, cy - half - 32,
            'Touch corners in order A, B, C (D optional) with the calibrated tip.\n'
            'Corner order = OpenCV detection order (A top-left, clockwise).',
            fontsize=10, ha='center', va='top', color='#444')
    # 100 mm ruler
    rx, ry = cx - 50, 40
    ax.add_patch(Rectangle((rx, ry), 100, 3, facecolor='black'))
    for i in range(0, 101, 10):
        ax.plot([rx + i, rx + i], [ry - 3, ry], color='black', lw=1)
    ax.text(cx, ry - 5, '100 mm — measure this after printing; must be 100 ± 0.5 mm '
            '(print at 100 % / "actual size", NOT fit-to-page)',
            fontsize=9, ha='center', va='top')
    fig.savefig(out_path, format=os.path.splitext(out_path)[1][1:], dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ids', type=int, nargs='+', default=[0, 1])
    ap.add_argument('--size', type=float, default=120.0, help='black square, mm')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'tags'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    for i in a.ids:
        for ext in ('pdf', 'png'):
            p = os.path.join(a.out, f'apriltag36h11-id{i}-{a.size:.0f}mm.{ext}')
            make_sheet(i, a.size, p, d)
            print('wrote', p)


if __name__ == '__main__':
    main()
