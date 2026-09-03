#!/usr/bin/env python3
"""
Combine the two per-tag hand-eye solutions into the one file the launch uses.

WHY TWO TAGS. A single-tag hand-eye solve reports its own residual, and that
residual is optimistic by construction: it is the spread of ONE tag seen from
the very poses that were fitted, so any error correlated with those poses
disappears into the fit. Solving a SECOND tag, at a different place in the
cell, from a different set of poses, gives a genuinely independent estimate of
the same fixed quantity. The disagreement between the two is the honest
accuracy number, and it is the figure quoted in the report.

WHY NOT ONE JOINT SOLVE OVER BOTH TAG SETS. cv2.calibrateHandEye assumes ONE
target rigidly fixed in the base frame: it forms relative motions and cancels
the target pose. Concatenating samples from two tags at different places
silently violates that (every cross-tag motion pair encodes the tag-to-tag
offset as if it were camera motion), and the solver does not complain - it
just returns a wrong X with a plausible residual. So each tag is solved alone
and only the ANSWERS are combined.

COMBINING. When the two agree within tolerance, the output is their mean -
two independent unbiased estimates average to a better one (rotation is
averaged as a proper quaternion mean, not component-wise, so the result stays
a rotation). When they disagree beyond --max-mm/--max-deg, this REFUSES to
write: a mount that two tags cannot agree on is not a mount worth publishing,
and quietly picking the prettier of the two is how a bad calibration ships.

  python3 combine_tags.py calib/tool0_to_camera_2026-09-03-tag0.yaml \
                          calib/tool0_to_camera_2026-09-03-tag1.yaml
"""
import argparse
import datetime as dt
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


def load(path):
    d = yaml.safe_load(open(path))
    link = d['tool0_to_camera_link']
    opt = d['tool0_to_camera_color_optical_frame']
    return d, link, opt


def quat_mean(quats):
    """Markley's quaternion mean: dominant eigenvector of sum(q q^T).
    Component-wise averaging of quaternions does not generally give a unit
    quaternion and is not rotation-averaging; this is."""
    Q = np.array(quats, dtype=float)
    Q = np.array([q if q[3] >= 0 else -q for q in Q])   # resolve q/-q sign ambiguity
    M = sum(np.outer(q, q) for q in Q)
    w, v = np.linalg.eigh(M)
    q = v[:, int(np.argmax(w))]
    return q / np.linalg.norm(q)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs=2, help='the two per-tag result yamls')
    ap.add_argument('--max-mm', type=float, default=10.0,
                    help='refuse if the two tags disagree by more than this in translation')
    ap.add_argument('--max-deg', type=float, default=2.0,
                    help='refuse if the two tags disagree by more than this in rotation')
    ap.add_argument('--out', default=None, help='default calib/tool0_to_camera_<date>.yaml')
    ap.add_argument('--force', action='store_true',
                    help='write even if the two tags disagree (records the disagreement)')
    a = ap.parse_args()

    ds, links, opts = [], [], []
    for f in a.files:
        d, link, opt = load(f)
        ds.append(d); links.append(link); opts.append(opt)

    # Both solves must describe the SAME frames and the SAME stream, or the numbers
    # are not comparable and averaging them is meaningless.
    for key in ('parent', 'child'):
        if links[0][key] != links[1][key]:
            sys.exit(f'frame mismatch: {key} {links[0][key]} vs {links[1][key]}')
    s0, s1 = ds[0].get('source', {}), ds[1].get('source', {})
    if s0.get('image_topic') != s1.get('image_topic'):
        sys.exit(f'stream mismatch: {s0.get("image_topic")} vs {s1.get("image_topic")} '
                 '- the two solves are not of the same camera stream')

    t = [np.array(l['xyz'], float) for l in links]
    q = [np.array(l['quaternion_xyzw'], float) for l in links]
    dt_mm = float(np.linalg.norm(t[0] - t[1]) * 1e3)
    dR = R.from_quat(q[0]).inv() * R.from_quat(q[1])
    dr_deg = float(np.degrees(dR.magnitude()))

    for d_ in ds:
        print(f'tag {d_["tag_id"]}: {d_["samples"]} samples, '
              f'residual {d_["residuals"]["tag_spread_rms_mm"]:.2f} mm / '
              f'{d_["residuals"]["tag_rot_rms_deg"]:.2f} deg')
    print(f'\nINDEPENDENT AGREEMENT (the honest accuracy figure): '
          f'{dt_mm:.2f} mm, {dr_deg:.2f} deg')

    ok = dt_mm <= a.max_mm and dr_deg <= a.max_deg
    if not ok:
        print(f'  -> EXCEEDS tolerance ({a.max_mm:.1f} mm / {a.max_deg:.1f} deg)')
        if not a.force:
            sys.exit('REFUSING to write a combined mount the two tags do not agree on. '
                     'Re-take the tag with the worse residual (more viewpoint variety: '
                     'tilt and roll, not just standoff), or pass --force to record it anyway.')
    else:
        print(f'  -> within tolerance ({a.max_mm:.1f} mm / {a.max_deg:.1f} deg)')

    t_mean = (t[0] + t[1]) / 2.0
    q_mean = quat_mean(q)
    to = [np.array(o['xyz'], float) for o in opts]
    qo_mean = quat_mean([np.array(o['quaternion_xyzw'], float) for o in opts])

    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.files[0])),
                                f'tool0_to_camera_{dt.date.today().isoformat()}.yaml')
    d = {
        'calibrated': dt.datetime.now().isoformat(timespec='seconds'),
        'combined_from': [os.path.basename(f) for f in a.files],
        'tag_ids': [ds[0]['tag_id'], ds[1]['tag_id']],
        'method': f'mean of two independent per-tag solves ({ds[0]["method"]})',
        'source': ds[0]['source'],
        'tool0_to_camera_link': {
            'parent': links[0]['parent'], 'child': links[0]['child'],
            'xyz': [float(v) for v in t_mean],
            'quaternion_xyzw': [float(v) for v in q_mean],
            'rpy_deg': [float(v) for v in R.from_quat(q_mean).as_euler('xyz', degrees=True)]},
        'tool0_to_camera_color_optical_frame': {
            'parent': opts[0]['parent'], 'child': opts[0]['child'],
            'xyz': [float(v) for v in (to[0] + to[1]) / 2.0],
            'quaternion_xyzw': [float(v) for v in qo_mean]},
        'cross_tag_agreement': {
            'translation_mm': dt_mm, 'rotation_deg': dr_deg,
            'tolerance_mm': a.max_mm, 'tolerance_deg': a.max_deg,
            'within_tolerance': bool(ok)},
        'per_tag': {f'tag{ds[i]["tag_id"]}': {
            'samples': ds[i]['samples'],
            'xyz': [float(v) for v in t[i]],
            'residual_mm': ds[i]['residuals']['tag_spread_rms_mm'],
            'residual_deg': ds[i]['residuals']['tag_rot_rms_deg']} for i in (0, 1)},
    }
    with open(out, 'w') as f:
        yaml.safe_dump(d, f, sort_keys=False)
    print(f'\n>>> {out}')
    print(f'    {links[0]["parent"]} -> {links[0]["child"]}: xyz {np.round(t_mean, 4)} '
          f'rpy {np.round(d["tool0_to_camera_link"]["rpy_deg"], 2)} deg')
    print(f'    stream: {d["source"]["image_topic"]}  ({d["source"]["optical_frame"]})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
