#!/usr/bin/env python3
"""
Explain a two-tag DISAGREEMENT: is it a printed-tag SIZE error?

When the two tags solve to different mounts, the first suspect is not the
robot - it is the paper. PnP recovers the tag's range from its apparent size,
so if a sheet printed at 98% the solver believes every view of it is 2% nearer
than it is, and that bias lands almost entirely in the solved translation. A
2% error on a 120 mm tag at 0.4 m is about 8 mm of mount error, which is the
size of disagreement that makes a two-tag check fail.

This re-solves one tag's samples over a range of assumed tag sizes and finds
the size that best reconciles it with the other tag. Because the tag pose
scales exactly linearly with assumed size, no re-detection is needed - the
stored T_cam_tag translations are simply rescaled.

READ THE RESULT CAREFULLY. A best-fit scale near 1.00 means size is NOT the
explanation and the disagreement is real (mount flex, a bent board, or poor
pose diversity). A best-fit near a plausible print error (0.97-1.03) is a
HYPOTHESIS to confirm with the 100 mm ruler printed on the sheet - it is not
licence to adopt the fitted number. Never "calibrate" the tag size to make the
answer pretty: that hides a real error behind a fudged constant.

  python3 tag_scale_probe.py calib/samples-<date>-tag1.yaml \
      --against calib/tool0_to_camera_<date>-tag0.yaml
"""
import argparse
import sys

import numpy as np
import yaml

sys.path.insert(0, __file__.rsplit('/', 2)[0])
from handeye_calibrate import solve, inv  # noqa: E402


def X_of(path):
    d = yaml.safe_load(open(path))
    k = 'tool0_to_camera_color_optical_frame'
    m = d[k]
    from scipy.spatial.transform import Rotation as R
    M = np.eye(4)
    M[:3, :3] = R.from_quat(m['quaternion_xyzw']).as_matrix()
    M[:3, 3] = m['xyz']
    return M


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('samples', help='samples yaml of the tag to probe')
    ap.add_argument('--against', required=True, help='the OTHER tag result yaml')
    ap.add_argument('--method', default='park')
    ap.add_argument('--lo', type=float, default=0.94)
    ap.add_argument('--hi', type=float, default=1.06)
    ap.add_argument('--steps', type=int, default=61)
    a = ap.parse_args()

    sd = yaml.safe_load(open(a.samples))
    ref = X_of(a.against)
    base = [np.asarray(s['T_cam_tag'], float) for s in sd['samples']]

    rows = []
    for s in np.linspace(a.lo, a.hi, a.steps):
        scaled = []
        for smp, T in zip(sd['samples'], base):
            T2 = T.copy()
            T2[:3, 3] = T[:3, 3] * s        # range scales linearly with assumed tag size
            scaled.append({'T_base_tool': smp['T_base_tool'], 'T_cam_tag': T2.tolist()})
        res = solve(scaled)
        if 'X' not in res.get(a.method, {}):
            continue
        X = res[a.method]['X']
        D = inv(ref) @ X
        rows.append((s, float(np.linalg.norm(D[:3, 3]) * 1e3),
                     res[a.method]['tag_spread_rms_mm']))

    if not rows:
        sys.exit('no solution over the scanned range')
    rows.sort(key=lambda r: r[1])
    best_s, best_mm, best_res = rows[0]
    at1 = min(rows, key=lambda r: abs(r[0] - 1.0))

    print(f'tag {sd["tag_id"]} samples: {len(sd["samples"])}, nominal size '
          f'{sd["tag_size_m"] * 1e3:.1f} mm')
    print(f'at scale 1.000 (as printed): disagreement {at1[1]:.1f} mm with '
          f'{a.against.rsplit("/", 1)[-1]}')
    print(f'best-fit scale {best_s:.4f} -> disagreement {best_mm:.1f} mm '
          f'(implied printed size {sd["tag_size_m"] * best_s * 1e3:.1f} mm, '
          f'own residual {best_res:.2f} mm)')
    # Judge on how much the disagreement IMPROVES, not on the fitted scale alone.
    # A best-fit scale can sit a percent from nominal and still explain almost
    # nothing; reporting that as a size hypothesis would send the captain to
    # re-measure a sheet that is fine.
    gain_mm = at1[1] - best_mm
    frac = gain_mm / at1[1] if at1[1] > 0 else 0.0
    if frac < 0.5 or gain_mm < 2.0:
        print(f'\n=> size does NOT explain it: the best possible size only recovers '
              f'{gain_mm:.1f} mm of {at1[1]:.1f} mm ({frac * 100:.0f}%).')
        print('   The disagreement is real. Look at pose diversity, a bent or non-flat')
        print('   board, or accept it as the honest accuracy of this calibration.')
    else:
        pct = (best_s - 1.0) * 100
        print(f'\n=> HYPOTHESIS: a size error explains {frac * 100:.0f}% of the gap, '
              f'consistent with this sheet printing {pct:+.1f}% '
              f'({sd["tag_size_m"] * best_s * 1e3:.1f} mm instead of '
              f'{sd["tag_size_m"] * 1e3:.0f} mm).')
        print('   CONFIRM WITH THE RULER on the sheet before believing it. Do not adopt')
        print('   the fitted size to make the numbers agree - that hides the real error.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
