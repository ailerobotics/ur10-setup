#!/usr/bin/env python3
"""
Headless renders of a saved nvblox mesh (.ply from map_ctl.py ply) — for the
report when no display is available (the workstation screen is often
locked; RViz needs X). Draws orthographic top-down (x-y) and side (y-z)
views of the mesh vertices coloured by height / mesh colour, plus optional
tool paths from plans/*.yaml and boxes, all in base_link.

  python3 render_map.py ~/ur10/cumotion/maps/cell-2026-08-26.ply --out cellmap-map.png
  python3 render_map.py cell.ply --plan plans/with-box.yaml --plan plans/box-removed.yaml \
      --box 0.2,0.5,0.5,0.8,0,0.4 --view 3d
"""
import argparse
import os
import struct

import numpy as np
import yaml


def read_ply(path):
    """Minimal PLY reader (ascii or binary_little_endian; vertex x,y,z[,r,g,b]; faces)."""
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            header.append(line)
            if line == 'end_header':
                break
        fmt = next(h.split()[1] for h in header if h.startswith('format'))
        elements, cur = [], None
        for h in header:
            p = h.split()
            if p[0] == 'element':
                cur = {'name': p[1], 'n': int(p[2]), 'props': []}
                elements.append(cur)
            elif p[0] == 'property' and cur is not None:
                cur['props'].append(p[1:])
        verts = colors = faces = None
        for el in elements:
            if el['name'] == 'vertex':
                names = [p[-1] for p in el['props']]
                types = [p[0] for p in el['props']]
                tmap = {'float': 'f4', 'float32': 'f4', 'double': 'f8', 'uchar': 'u1',
                        'uint8': 'u1', 'int': 'i4', 'uint': 'u4', 'short': 'i2', 'ushort': 'u2'}
                if fmt == 'ascii':
                    rows = [f.readline().split() for _ in range(el['n'])]
                    arr = np.array(rows, dtype=float)
                    verts = arr[:, [names.index(k) for k in 'xyz']]
                    if 'red' in names:
                        colors = arr[:, [names.index(k) for k in ('red', 'green', 'blue')]] / 255
                else:
                    dt = np.dtype([(n, '<' + tmap[t]) for n, t in zip(names, types)])
                    arr = np.frombuffer(f.read(dt.itemsize * el['n']), dtype=dt)
                    verts = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float)
                    if 'red' in names:
                        colors = np.column_stack([arr['red'], arr['green'], arr['blue']]) / 255
            elif el['name'] == 'face':
                if fmt == 'ascii':
                    faces = np.array([f.readline().split()[1:4] for _ in range(el['n'])], dtype=int)
                else:
                    cnt_t = el['props'][0][1]
                    idx_t = el['props'][0][2]
                    cs = {'uchar': 'B', 'uint8': 'B', 'int': 'i', 'uint': 'I'}[cnt_t]
                    isz = {'int': 'i', 'uint': 'I', 'int32': 'i', 'uint32': 'I'}[idx_t]
                    out = []
                    for _ in range(el['n']):
                        n = struct.unpack('<' + cs, f.read(struct.calcsize(cs)))[0]
                        out.append(struct.unpack('<' + isz * n, f.read(struct.calcsize(isz) * n))[:3])
                    faces = np.array(out, dtype=int)
    return verts, colors, faces


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ply')
    ap.add_argument('--out', default=None)
    ap.add_argument('--plan', action='append', default=[], help='plans/<label>.yaml (repeat)')
    ap.add_argument('--box', action='append', default=[], help='xmin,xmax,ymin,ymax,zmin,zmax')
    ap.add_argument('--zmin', type=float, default=-0.6)
    ap.add_argument('--zmax', type=float, default=2.0)
    ap.add_argument('--subsample', type=int, default=1)
    ap.add_argument('--view', default='ortho', choices=['ortho', '3d'])
    ap.add_argument('--title', default=None)
    a = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    V, C, F = read_ply(a.ply)
    keep = (V[:, 2] >= a.zmin) & (V[:, 2] <= a.zmax)
    V = V[keep][::a.subsample]
    C = C[keep][::a.subsample] if C is not None else None
    print(f'{a.ply}: {len(V)} vertices shown, bbox min {V.min(axis=0).round(2)} max {V.max(axis=0).round(2)}')
    colors = ['#0e7d86', '#b23a3a', '#a06c14', '#2f7d4f']
    out = a.out or os.path.splitext(a.ply)[0] + f'-{a.view}.png'

    if a.view == '3d':
        fig = plt.figure(figsize=(11, 8))
        ax = fig.add_subplot(111, projection='3d')
        order = np.argsort(V[:, 2])
        ax.scatter(V[order, 0], V[order, 1], V[order, 2], s=0.5,
                   c=C[order] if C is not None else V[order, 2], cmap='viridis', linewidths=0)
        for i, p in enumerate(a.plan):
            rec = yaml.safe_load(open(p))
            if 'tool_path' in rec:
                P = np.array(rec['tool_path'])
                ax.plot(P[:, 0], P[:, 1], P[:, 2], color=colors[i % 4], lw=2.5,
                        label=f'{rec["label"]} ({rec["pipeline"]})')
        ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
        ax.set_box_aspect((np.ptp(V[:, 0]), np.ptp(V[:, 1]), np.ptp(V[:, 2])))
        ax.view_init(elev=35, azim=-135)
        if a.plan:
            ax.legend()
        ax.set_title(a.title or os.path.basename(a.ply))
        fig.tight_layout(); fig.savefig(out, dpi=130)
        print('->', out)
        return

    fig, (ax, az) = plt.subplots(1, 2, figsize=(14, 6.5),
                                 gridspec_kw={'width_ratios': [1.1, 1]})
    # top-down: draw high points last so shelves cover the floor
    order = np.argsort(V[:, 2])
    sc = ax.scatter(V[order, 0], V[order, 1], s=0.4, c=V[order, 2], cmap='viridis',
                    vmin=a.zmin, vmax=min(a.zmax, 2.0), linewidths=0)
    ax.set_aspect('equal'); ax.set_xlabel('x [m] base_link'); ax.set_ylabel('y [m]')
    ax.set_title('top-down (colour = height)')
    ax.plot(0, 0, 'k+', ms=14, mew=2); ax.text(0.03, 0.03, 'base_link', fontsize=9)
    fig.colorbar(sc, ax=ax, shrink=0.7, label='z [m]')
    # side: y-z, colour by x
    order = np.argsort(-V[:, 0])
    az.scatter(V[order, 1], V[order, 2], s=0.4, c=V[order, 0], cmap='coolwarm', linewidths=0)
    az.set_aspect('equal'); az.set_xlabel('y [m]'); az.set_ylabel('z [m]')
    az.set_title('side view y-z (colour = x)')
    az.axhline(0, color='k', lw=0.8, ls=':')
    for i, p in enumerate(a.plan):
        rec = yaml.safe_load(open(p))
        if 'tool_path' not in rec:
            continue
        P = np.array(rec['tool_path'])
        lbl = f'{rec["label"]} ({rec["pipeline"]}): {rec["tool_path_m"]:.2f} m'
        if 'box_check' in rec:
            m = rec['box_check']['min_clearance_m']
            lbl += f', arm {"THROUGH box" if m < 0 else "clear"} ({m*100:+.0f} cm)'
        ax.plot(P[:, 0], P[:, 1], color=colors[i % 4], lw=2.5, label=lbl)
        ax.plot(P[0, 0], P[0, 1], 'o', color=colors[i % 4]); ax.plot(P[-1, 0], P[-1, 1], 'x', color=colors[i % 4], mew=2)
        az.plot(P[:, 1], P[:, 2], color=colors[i % 4], lw=2.5)
        az.plot(P[0, 1], P[0, 2], 'o', color=colors[i % 4]); az.plot(P[-1, 1], P[-1, 2], 'x', color=colors[i % 4], mew=2)
    for b in a.box:
        b = [float(v) for v in b.split(',')]
        ax.add_patch(Rectangle((b[0], b[2]), b[1]-b[0], b[3]-b[2], fill=False, ec='#b23a3a', lw=2, label='box'))
        az.add_patch(Rectangle((b[2], b[4]), b[3]-b[2], b[5]-b[4], fill=False, ec='#b23a3a', lw=2))
    if a.plan or a.box:
        ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.12))
    fig.suptitle(a.title or os.path.basename(a.ply))
    fig.tight_layout(); fig.savefig(out, dpi=130)
    print('->', out)


if __name__ == '__main__':
    main()
