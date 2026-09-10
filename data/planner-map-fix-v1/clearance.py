#!/usr/bin/env python3
"""Offline park-pose / park->tag1-hover clearance against an nvblx map.

Pure recorded-data reproduction: parse the calibrated URDF, FK the XRDF
collision spheres into base_link, and for each sphere compute
  clearance = (nearest_occupied_voxel_centre_dist - voxel_size/2) - radius - buffer
occupied := TSDF weight >= 0.1 AND distance <= 0 (nvblox_cell.yaml). A negative
worst-clearance is exactly the boundary-state invalidity cuMotion rejects with.
No ROS, no rig, no /compute_fk — FK is done here from the URDF.

usage: clearance.py <map.nvblx>
"""
import sqlite3
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

VPS = 8
NVOX = VPS ** 3
BS = 0.16
VS = BS / VPS  # 0.02

URDF = 'cumotion/ur10_calibrated.urdf'
XRDF = 'cumotion/ur10_map.xrdf'

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Bench-recorded poses (report §1 / §5), JOINTS order, rad.
PARK = [-0.0828, -2.8959, 0.8008, 5.2519, -0.6772, 0.0196]
TAG1_HOVER = [1.2971, -3.0399, -0.2456, 5.9301, -1.1607, -0.5524]

# Per-link WORLD buffer (ur10_map.xrdf collision.buffer_distance), metres.
BUFFER = {'shoulder_link': 0.02, 'upper_arm_link': 0.02, 'forearm_link': 0.04,
          'wrist_1_link': 0.04, 'wrist_2_link': 0.04, 'wrist_3_link': 0.04,
          'tool0': 0.04}


def rpy(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T(xyz, rpy_):
    m = np.eye(4)
    m[:3, :3] = rpy(*rpy_)
    m[:3, 3] = xyz
    return m


def Rz_th(th):
    m = np.eye(4)
    c, s = np.cos(th), np.sin(th)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
    return m


def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = [float(x) for x in (o.get('xyz', '0 0 0')).split()]
        rp = [float(x) for x in (o.get('rpy', '0 0 0')).split()]
        joints[j.get('name')] = dict(type=j.get('type'),
                                     parent=j.find('parent').get('link'),
                                     child=j.find('child').get('link'),
                                     xyz=xyz, rpy=rp)
    return joints


def parse_spheres(path):
    """link -> list of (center[3], radius) from the XRDF geometry block."""
    import yaml
    doc = yaml.safe_load(open(path))
    g = doc['geometry']['ur10e_collision_spheres']['spheres']
    return {link: [(np.array(s['center'], float), float(s['radius'])) for s in lst]
            for link, lst in g.items()}


def link_frames(joints, q):
    """Return dict link_name -> 4x4 base_link transform for joint vector q."""
    qd = dict(zip(JOINTS, q))
    frames = {'base_link': np.eye(4)}
    # ordered chain from the URDF
    chain = ['base_link-base_link_inertia', 'shoulder_pan_joint',
             'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
             'wrist_2_joint', 'wrist_3_joint', 'wrist_3-flange', 'flange-tool0']
    for jn in chain:
        j = joints[jn]
        Tj = T(j['xyz'], j['rpy'])
        if j['type'] == 'revolute':
            Tj = Tj @ Rz_th(qd[jn])
        frames[j['child']] = frames[j['parent']] @ Tj
    return frames


def sphere_points(joints, spheres, q):
    """(N,3) base_link sphere centres, and parallel arrays radius, buffer, link."""
    frames = link_frames(joints, q)
    pts, rad, buf, lnk = [], [], [], []
    for link, lst in spheres.items():
        Tf = frames[link]
        for c, r in lst:
            p = Tf @ np.array([c[0], c[1], c[2], 1.0])
            pts.append(p[:3]); rad.append(r); buf.append(BUFFER[link]); lnk.append(link)
    return np.array(pts), np.array(rad), np.array(buf), lnk


def occupied_centres(path):
    con = sqlite3.connect(path)
    pts = []
    L = np.arange(NVOX)
    vx, vy, vz = L // 64, (L // 8) % 8, L % 8
    for ix, iy, iz, blob in con.execute(
            "SELECT index_x,index_y,index_z,data FROM tsdf_layer_data").fetchall():
        a = np.frombuffer(blob, '<f4').reshape(NVOX, 2)
        m = (a[:, 1] >= 0.1) & (a[:, 0] <= 0.0)
        if m.any():
            cx = (ix * VPS + vx[m] + 0.5) * VS
            cy = (iy * VPS + vy[m] + 0.5) * VS
            cz = (iz * VPS + vz[m] + 0.5) * VS
            pts.append(np.column_stack([cx, cy, cz]))
    con.close()
    return np.vstack(pts)


def clearances(tree, joints, spheres, q):
    pts, rad, buf, lnk = sphere_points(joints, spheres, q)
    d, idx = tree.query(pts, k=1)
    esdf_surf = d - VS / 2.0
    clr = esdf_surf - rad - buf
    return clr, esdf_surf, rad, buf, lnk, tree.data[idx]


def worst(tree, joints, spheres, q):
    clr, esdf, rad, buf, lnk, near = clearances(tree, joints, spheres, q)
    i = int(np.argmin(clr))
    return clr[i], lnk[i], esdf[i], rad[i], buf[i], near[i]


def main():
    mp = sys.argv[1]
    joints = parse_urdf(URDF)
    spheres = parse_spheres(XRDF)
    occ = occupied_centres(mp)
    tree = cKDTree(occ)
    print(f'map: {mp}')
    print(f'occupied voxels (w>=0.1 & d<=0): {len(occ)}')

    # --- park start state ---
    clr, lnk, esdf, rad, buf, near = worst(tree, joints, spheres, PARK)
    print('\n[PARK start state]')
    print(f'  worst sphere: {lnk}  esdf_surf={esdf*100:.2f} cm  r={rad*100:.1f} buf={buf*100:.1f}')
    print(f'  nearest voxel: [{near[0]:.3f}, {near[1]:.3f}, {near[2]:.3f}]')
    print(f'  WORST CLEARANCE (with buffer): {clr*100:+.2f} cm')
    print(f'  worst un-buffered clearance:   {(clr+buf)*100:+.2f} cm')

    # --- park -> tag1-hover, naive straight-line joint interpolation ---
    N = 101
    ts = np.linspace(0, 1, N)
    p0, p1 = np.array(PARK), np.array(TAG1_HOVER)
    wc, wlink, wt = 1e9, None, None
    for t in ts:
        q = (1 - t) * p0 + t * p1
        clr, lnk, esdf, rad, buf, near = worst(tree, joints, spheres, q)
        if clr < wc:
            wc, wlink, wt = clr, lnk, t
    print(f'\n[PARK -> TAG1-HOVER  (naive linear interp, {N} samples)]')
    print(f'  WORST CLEARANCE along path: {wc*100:+.2f} cm  (sphere {wlink}, at t={wt:.2f})')
    print(f'  path plan.: {"CLEAR" if wc > 0 else "IN COLLISION"} for naive straight line')


if __name__ == '__main__':
    main()
