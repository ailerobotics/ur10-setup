#!/usr/bin/env python3
"""
Print-ready AprilTag (36h11) anchor sheets at EXACT physical scale.

One tag per A4 page, black square = --size mm (default 120), corner labels
A B C D placed in OpenCV's detection order (A = top-left, then clockwise) so
the captain touches exactly the corners tools/tag_corners.py expects, plus a
100 mm ruler to verify the printer did not "fit to page".

Why a PDF and not just a PNG: a PDF page carries physical units, so
"print at 100 % / actual size" gives a 120.0 mm square; a PNG's size depends
on whatever DPI the print dialog assumes (a 5 % scale error is a 6 mm tag
error, i.e. 5 % range error on every anchored pose).

Why the tag is drawn as VECTOR RECTANGLES and never as an image
(TRAP, 2026-08-27 — see ../README.md): the first version drew the marker
with `ax.imshow(marker, cmap='gray')`. matplotlib 3.6.3's PDF backend
encodes a two-valued grayscale array as a 1-bit `/Indexed /DeviceRGB`
image with `/Predictor 10`, and its row packing is wrong: both poppler and
ghostscript render the tag as diagonal black/white stripes and
`cv2.aruco.detectMarkers` finds nothing at any scale. The PNG written from
the same figure was perfect, so the corruption was invisible in a file
manager thumbnail and would only have surfaced as "the detector never sees
the printed tag" after the captain had already printed and mounted it.
Drawing the 8x8 cells as `Rectangle` patches avoids every image codec path
and is resolution-independent as a bonus. `--check` (on by default) renders
the finished PDF with pdftoppm and re-detects it before saying "wrote".

  python3 make_tags.py --ids 0 1 --out cams/cellmap/tags/
Mount each sheet on something rigid and flat (foam board / clipboard); a
curled sheet breaks the planar-tag assumption of the pose solver.
"""
import argparse
import os
import subprocess
import sys
import tempfile

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

A4_IN = (8.27, 11.69)
MM = 1 / 25.4
CELLS = 8          # 36h11 = 6x6 data bits + a 1-cell black border all round


def apriltag_dict():
    """DICT_APRILTAG_36h11, across the OpenCV 4.6 / 4.7+ aruco API rename."""
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def marker_bits(dictionary, tag_id):
    """CELLS x CELLS bool array, True = black cell."""
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    return draw(dictionary, tag_id, CELLS) == 0


def detect(img, dictionary):
    """(ids, corners) across the OpenCV 4.6 / 4.7+ aruco API rename."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(img)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            img, dictionary, parameters=cv2.aruco.DetectorParameters_create())
    return ids, corners


def make_sheet(tag_id, size_mm, out_path, dictionary):
    bits = marker_bits(dictionary, tag_id)
    fig = plt.figure(figsize=A4_IN)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, A4_IN[0] / MM)      # axes in millimetres
    ax.set_ylim(0, A4_IN[1] / MM)
    ax.set_aspect('equal')
    ax.axis('off')
    cx, cy = A4_IN[0] / MM / 2, A4_IN[1] / MM / 2 + 20
    half = size_mm / 2
    cell = size_mm / CELLS

    # Vector cells. Neighbouring black cells are grown by eps so they merge
    # without hairline seams from the renderer's anti-aliasing; eps is 0.3 %
    # of a cell (~0.045 mm at 15 mm), far inside the +-0.5 mm print tolerance
    # the ruler checks, and --check measures the finished side length anyway.
    eps = cell * 0.003
    for r in range(CELLS):
        for c in range(CELLS):
            if bits[r, c]:
                # row 0 is the TOP row of the marker
                x = cx - half + c * cell
                y = cy + half - (r + 1) * cell
                ax.add_patch(Rectangle((x - eps, y - eps),
                                       cell + 2 * eps, cell + 2 * eps,
                                       facecolor='black', edgecolor='none',
                                       linewidth=0))

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


def check_pdf(pdf_path, tag_id, size_mm, dictionary, dpi=300, tol_mm=0.5):
    """Render the finished PDF and re-detect it. Returns the measured side (mm).

    This is the gate the striped-PDF bug walked straight through: the figure
    looked right, the PNG was right, and only the PDF was unreadable.
    """
    if not shutil_which('pdftoppm'):
        raise RuntimeError('pdftoppm (poppler-utils) not found — cannot verify '
                           'the PDF; re-run with --no-check to skip at your own risk')
    with tempfile.TemporaryDirectory() as td:
        stem = os.path.join(td, 'page')
        subprocess.run(['pdftoppm', '-r', str(dpi), '-png', '-singlefile',
                        pdf_path, stem], check=True, capture_output=True)
        img = cv2.imread(stem + '.png', cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f'pdftoppm produced no readable page for {pdf_path}')
    ids, corners = detect(img, dictionary)
    found = [] if ids is None else ids.ravel().tolist()
    if tag_id not in found:
        raise RuntimeError(
            f'{os.path.basename(pdf_path)}: rendered at {dpi} dpi, detector found '
            f'{found or "no marker"} — expected id {tag_id}. The PDF is corrupt; '
            f'do NOT print it.')
    quad = corners[found.index(tag_id)].reshape(4, 2)
    sides_px = [np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]
    side_mm = float(np.mean(sides_px)) / dpi * 25.4
    if abs(side_mm - size_mm) > tol_mm:
        raise RuntimeError(
            f'{os.path.basename(pdf_path)}: black square measures {side_mm:.2f} mm, '
            f'expected {size_mm:.1f} ± {tol_mm} mm')
    return side_mm


def shutil_which(name):
    from shutil import which
    return which(name)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ids', type=int, nargs='+', default=[0, 1])
    ap.add_argument('--size', type=float, default=120.0, help='black square, mm')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'tags'))
    ap.add_argument('--no-check', dest='check', action='store_false',
                    help='skip the render-and-detect verification (not recommended)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    d = apriltag_dict()
    rc = 0
    for i in a.ids:
        paths = {}
        for ext in ('pdf', 'png'):
            p = os.path.join(a.out, f'apriltag36h11-id{i}-{a.size:.0f}mm.{ext}')
            make_sheet(i, a.size, p, d)
            paths[ext] = p
        if a.check:
            try:
                side = check_pdf(paths['pdf'], i, a.size, d)
            except Exception as e:                       # noqa: BLE001
                print(f'FAILED  {paths["pdf"]}\n        {e}', file=sys.stderr)
                rc = 1
                continue
            print(f'wrote {paths["pdf"]}  (verified: id {i} detected, '
                  f'black square {side:.2f} mm)')
        else:
            print('wrote', paths['pdf'], '(UNVERIFIED)')
        print('wrote', paths['png'])
    return rc


if __name__ == '__main__':
    sys.exit(main())
