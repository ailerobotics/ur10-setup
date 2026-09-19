#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run the pluggable person detector on a set of images and report.

For each image: number of people, scores, whether keypoints came back, and an
annotated copy (boxes + COCO skeleton) written next to it. Prints peak GPU
memory so the <2 GB budget can be verified. Use it on public sample images that
include partial bodies and seated people.

  ~/venvs/human-safety/bin/python cams/human_safety/tools/detector_selfcheck.py \
      --images img1.jpg img2.jpg --out /tmp/hs_selfcheck --device cuda
"""
import argparse
import pathlib
import sys

import numpy as np

_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import cv2  # noqa: E402

from human_safety.detector import build_detector  # noqa: E402

SKELETON = [(5, 7), (7, 9), (6, 8), (8, 10), (11, 13), (13, 15),
            (12, 14), (14, 16), (5, 6), (11, 12), (5, 11), (6, 12)]


def annotate(bgr, dets):
    for d in dets:
        x0, y0, x1, y1 = [int(v) for v in d.bbox]
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(bgr, f"{d.score:.2f}", (x0, max(12, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if d.keypoints is not None:
            pts = d.keypoints
            for a, b in SKELETON:
                if pts[a, 2] > 0.3 and pts[b, 2] > 0.3:
                    cv2.line(bgr, tuple(pts[a, :2].astype(int)),
                             tuple(pts[b, :2].astype(int)), (255, 200, 0), 2)
            for x, y, c in pts:
                if c > 0.3:
                    cv2.circle(bgr, (int(x), int(y)), 3, (0, 0, 255), -1)
    return bgr


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="+", required=True)
    ap.add_argument("--out", default="/tmp/hs_selfcheck")
    ap.add_argument("--device", default="cuda")
    args = ap.parse_args(argv)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    det = build_detector("keypoint_rcnn", device=args.device)
    print(f"detector: {det.name}  license: {det.license}  device: {det.device}")
    det.warmup()

    total = 0
    for path in args.images:
        bgr = cv2.imread(path)
        if bgr is None:
            print(f"  SKIP (unreadable): {path}")
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        dets = det.detect(rgb)
        total += len(dets)
        kp = sum(1 for d in dets if d.keypoints is not None)
        scores = ", ".join(f"{d.score:.2f}" for d in dets)
        print(f"  {pathlib.Path(path).name}: {len(dets)} person(s) "
              f"[{scores}]  {kp} with keypoints")
        outp = out / f"annotated_{pathlib.Path(path).stem}.png"
        cv2.imwrite(str(outp), annotate(bgr, dets))
    print(f"total detections: {total}")
    print(f"peak GPU memory: {det.gpu_mem_mb():.0f} MB "
          f"({'<2GB OK' if det.gpu_mem_mb() < 2048 else 'OVER 2GB BUDGET'})")
    print(f"annotated images -> {out}")


if __name__ == "__main__":
    main()
