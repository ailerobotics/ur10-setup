#!/usr/bin/env python3
"""
Validate a hand-eye result from the CALIBRATION SAMPLES THEMSELVES - no extra
captain time, no second sweep.

validate_tag1.py answers the same questions but needs the captain to freedrive
the camera over both tags again. Everything it needs is already in the two
per-tag sample files, so this recomputes the checks offline. It is not a
replacement for a fresh independent sweep - the samples ARE what the extrinsic
was fitted on, so check (a) is partly self-referential - but checks (b) and (c)
are geometry the fit cannot fake, and they cost nothing.

Each tag detection is carried into base_link through exactly the chain the map
will use:

  base_link -> tool0 (TF at the sample) -> optical (the CALIBRATED X) -> tag (PnP)

(a) CONSISTENCY  a static tag must land in the same base_link place from every
    viewpoint; the spread is the calibration error made visible.
(b) SCALE        the base_link distance between the two tag centres. Solved
    from two disjoint sample sets, so a translation or scale error shows up
    here even though each set is individually self-consistent.
(c) TABLE PLANE  both tags lie flat on the same table, so they must come out at
    the same z with parallel normals. Catches a tilt error - the component the
    fronto-parallel poses constrain worst.

  python3 crosscheck_samples.py calib/tool0_to_camera_2026-09-03.yaml \
      calib/samples-2026-09-03-tag0.yaml calib/samples-2026-09-03-tag1.yaml
"""
import argparse
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


def T_of(xyz, quat):
    M = np.eye(4)
    M[:3, :3] = R.from_quat(quat).as_matrix()
    M[:3, 3] = xyz
    return M


def tag_poses_in_base(samples, X):
    """T_base_tag for every sample, through the calibrated X."""
    out = []
    for s in samples:
        T_base_tool = np.asarray(s['T_base_tool'], float)
        T_cam_tag = np.asarray(s['T_cam_tag'], float)
        out.append(T_base_tool @ X @ T_cam_tag)
    return out


def report_tag(name, Ts):
    P = np.array([T[:3, 3] for T in Ts])
    c = P.mean(axis=0)
    d = np.linalg.norm(P - c, axis=1) * 1e3
    N = np.array([T[:3, 2] for T in Ts])          # tag plane normal = its z axis
    nmean = N.mean(axis=0)
    nmean /= np.linalg.norm(nmean)
    ang = np.degrees(np.arccos(np.clip(N @ nmean, -1, 1)))
    print(f'{name}: {len(Ts)} views')
    print(f'  centre  base_link [{c[0]:+.4f} {c[1]:+.4f} {c[2]:+.4f}] m')
    print(f'  position spread   med {np.median(d):5.1f} | p95 {np.percentile(d, 95):5.1f} | '
          f'rms {np.sqrt((d ** 2).mean()):5.1f} | max {d.max():5.1f} mm')
    print(f'  normal   spread   med {np.median(ang):5.2f} | p95 {np.percentile(ang, 95):5.2f} | '
          f'max {ang.max():5.2f} deg')
    return c, nmean, float(np.median(d)), float(np.median(ang))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('result', help='combined (or per-tag) result yaml')
    ap.add_argument('samples', nargs=2, help='the two per-tag samples yamls')
    ap.add_argument('--expected-mm', type=float, default=None,
                    help='tape-measured tag0->tag1 centre distance, if known')
    ap.add_argument('--tol-spread-mm', type=float, default=5.0)
    ap.add_argument('--tol-scale-mm', type=float, default=10.0)
    ap.add_argument('--tol-z-mm', type=float, default=5.0)
    ap.add_argument('--tol-normal-deg', type=float, default=1.0)
    a = ap.parse_args()

    d = yaml.safe_load(open(a.result))
    key = 'tool0_to_camera_color_optical_frame'
    if key not in d:
        sys.exit(f'{a.result}: no {key}')
    X = T_of(d[key]['xyz'], d[key]['quaternion_xyzw'])
    src = d.get('source', {})
    print(f'X = {d[key]["parent"]} -> {d[key]["child"]}')
    if src.get('image_topic'):
        print(f'stream: {src["image_topic"]}')
    print()

    cs, ns, meds = [], [], []
    for f in a.samples:
        sd = yaml.safe_load(open(f))
        c, n, med, medang = report_tag(f'tag {sd["tag_id"]}', tag_poses_in_base(sd['samples'], X))
        cs.append(c); ns.append(n); meds.append((med, medang))
        print()

    ok = all(m <= a.tol_spread_mm for m, _ in meds)
    print(f'(a) consistency : median spreads '
          f'{", ".join(f"{m:.1f} mm" for m, _ in meds)} -> '
          f'{"PASS" if ok else "CHECK"} (< {a.tol_spread_mm} mm)')

    dist_mm = float(np.linalg.norm(cs[0] - cs[1]) * 1e3)
    line = f'(b) scale       : tag centres {dist_mm:.1f} mm apart'
    if a.expected_mm:
        err = abs(dist_mm - a.expected_mm)
        line += (f' vs {a.expected_mm:.1f} measured -> {err:.1f} mm '
                 f'{"PASS" if err <= a.tol_scale_mm else "FAIL"} (< {a.tol_scale_mm} mm)')
        ok = ok and err <= a.tol_scale_mm
    else:
        line += ' - no --expected-mm given, not judged'
    print(line)

    dz_mm = abs(cs[0][2] - cs[1][2]) * 1e3
    dn_deg = float(np.degrees(np.arccos(np.clip(ns[0] @ ns[1], -1, 1))))
    zok = dz_mm <= a.tol_z_mm and dn_deg <= a.tol_normal_deg
    print(f'(c) table plane : dz {dz_mm:.1f} mm (< {a.tol_z_mm}), normals differ '
          f'{dn_deg:.2f} deg (< {a.tol_normal_deg}) -> {"PASS" if zok else "CHECK"}')
    print(f'\nVERDICT: {"ACCEPT" if ok and zok else "REVIEW"}  '
          '(judged on medians; (a) is partly self-referential - these are the fitted samples)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
