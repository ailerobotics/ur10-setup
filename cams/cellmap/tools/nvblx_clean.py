#!/usr/bin/env python3
"""
Clean a saved nvblox map IN PLACE — crop out a region and/or drop isolated
speckle — writing a NEW .nvblx and leaving the original untouched.

A .nvblx is a SQLite database (`SQLite format 3` magic): one `<layer>_data`
table per layer, keyed by 8^3-voxel BLOCK index (index_x,index_y,index_z) with a
raw voxel BLOB. In THIS cell's maps only `tsdf_layer_data` and `color_layer_data`
carry blocks; esdf/occupancy/freespace/feature are empty and cuMotion recomputes
the ESDF it plans against from the TSDF at query time (nvblox_cell.yaml). So the
thing the planner actually sees is the TSDF, and that is what this tool edits.
There is no nvblox Python binding on host or in the container and no
remove-a-region service (map_ctl.py only save/load/ply/esdf), so we operate on
the file the planner loads, then verify the planner loads the result.

Two operations, composable in one pass:
  * region crop-out (--remove-box): every voxel whose CENTRE is inside an
    axis-aligned base_link box is set unobserved (weight 0) — the same state an
    unseen voxel has, which marching cubes and the ESDF integrator both ignore.
    Blocks left with no observed voxel are dropped. Voxel-precise (2 cm), so it
    takes only the pedestal and not the table voxel beside it.
  * speckle drop (--denoise): 26-connected components of occupied blocks smaller
    than --min-blocks are deleted. Isolated un-scanned specks float away from the
    real structure; the big connected cell (table+walls+shelf) is one component.
    Unobserved space reads as FREE to cuMotion, so every removed component is
    printed with its base_link centre, extent and block count — check each one
    is mid-air speckle and not a real small object (a carton spans 1–3 blocks),
    and protect known small objects with --keep-box.

The output file must not already exist (pass --force to overwrite a previous
cleaned map); src and dst may never be the same file.

Voxel layout (verified against the mesh, 8.9 mm median): nvblox VoxelBlock is
row-major voxels[x][y][z], so linear L -> vx=L//64, vy=(L//8)%8, vz=L%8, and the
voxel centre is ((block_index*8 + v + 0.5) * voxel_size) in base_link.

  nvblx_clean.py in.nvblx out.nvblx \
      --remove-box=-0.60,0.20,-0.32,0.50,-0.65,-0.10 --denoise --self-check
  nvblx_clean.py in.nvblx out.nvblx --remove-box=x0,x1,y0,y1,z0,z1   # repeatable
(use the --flag=value form: a box whose first number is negative is otherwise
read by argparse as an option and rejected with "expected one argument")

Runs anywhere Python + sqlite3 reach the file — no ROS, no rig (see
test_nvblx_clean.py). Confirm the cleaned map on the planner separately with
map_ctl.py load + verify_map.py.
"""
import argparse
import os
import shutil
import sqlite3
import sys
from collections import deque

import numpy as np

VPS = 8                     # voxels per block side (nvblox fixed)
NVOX = VPS ** 3             # 512

# Per-layer voxel struct. `weight_off` is the byte offset of the float weight
# whose zeroing marks a voxel unobserved; None means "no per-voxel clear, drop
# the whole block if it falls in the box". Any <name>_layer_data table not
# listed here gets UNKNOWN_LAYER, the same drop-the-block treatment, so no
# layer can keep a block the TSDF no longer has.
LAYERS = {
    'tsdf':      dict(bytes=8, weight_off=4),   # {f4 distance, f4 weight}
    'color':     dict(bytes=8, weight_off=4),   # {u1 r,g,b,a, f4 weight}
    'occupancy': dict(bytes=4, weight_off=0),   # {f4 log_odds} -> 0 = unknown
    'freespace': dict(bytes=None, weight_off=None),   # {timestamps, bool}: no weight
    'esdf':      dict(bytes=None, weight_off=None),
    'feature':   dict(bytes=None, weight_off=None),
}
UNKNOWN_LAYER = dict(bytes=None, weight_off=None)
_SUFFIX = '_layer_data'


def layer_tables(con):
    """Every layer name present as a <name>_layer_data table, known or not."""
    names = [r[0][:-len(_SUFFIX)] for r in con.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name LIKE ?",
        (f'%{_SUFFIX}',))]
    return sorted(names, key=lambda n: (n != 'tsdf', n))


def layer_spec(name):
    return LAYERS.get(name, UNKNOWN_LAYER)


def block_size(con, layer):
    row = con.execute(
        f"SELECT value_float FROM {layer}_layer_metadata WHERE param_name='block_size'"
    ).fetchone()
    if not row or row[0] is None:
        sys.exit(f'{layer}: no block_size in metadata')
    return float(row[0])


def voxel_centres(ix, iy, iz, bs):
    """(512,3) base_link centres for the voxels of one block, in BLOB order."""
    vs = bs / VPS
    L = np.arange(NVOX)
    vx, vy, vz = L // 64, (L // 8) % 8, L % 8
    return np.column_stack([(ix * VPS + vx + 0.5) * vs,
                            (iy * VPS + vy + 0.5) * vs,
                            (iz * VPS + vz + 0.5) * vs])


def in_any_box(centres, boxes):
    """Boolean mask: centre inside any [x0,x1,y0,y1,z0,z1]."""
    m = np.zeros(len(centres), bool)
    for x0, x1, y0, y1, z0, z1 in boxes:
        m |= ((centres[:, 0] >= x0) & (centres[:, 0] < x1) &
              (centres[:, 1] >= y0) & (centres[:, 1] < y1) &
              (centres[:, 2] >= z0) & (centres[:, 2] < z1))
    return m


def tsdf_occupied_mask(blob, surf_band):
    """Voxels of a TSDF block that read as observed surface (|dist|<=band, w>0)."""
    a = np.frombuffer(blob, '<f4').reshape(NVOX, 2)
    return (a[:, 1] > 0) & (np.abs(a[:, 0]) <= surf_band)


def crop_boxes(con, boxes, surf_band):
    """Zero the weight of every voxel inside `boxes`, drop emptied blocks.
    Returns (voxels_cleared, blocks_dropped)."""
    tables = layer_tables(con)
    bs = block_size(con, 'tsdf')
    # Which TSDF blocks touch a box, and the in-box voxel mask for each.
    cur = con.execute("SELECT index_x,index_y,index_z FROM tsdf_layer_data")
    touched = {}
    for ix, iy, iz in cur.fetchall():
        mask = in_any_box(voxel_centres(ix, iy, iz, bs), boxes)
        if mask.any():
            touched[(ix, iy, iz)] = mask
    cleared = 0
    emptied = []
    for (ix, iy, iz), mask in touched.items():
        # TSDF first, so we can decide whether the block is now fully unobserved.
        blob = con.execute(
            "SELECT data FROM tsdf_layer_data WHERE index_x=? AND index_y=? AND index_z=?",
            (ix, iy, iz)).fetchone()[0]
        a = np.frombuffer(blob, '<f4').reshape(NVOX, 2).copy()
        cleared += int((a[mask, 1] > 0).sum())
        a[mask, 1] = 0.0                       # weight 0 == unobserved
        con.execute("UPDATE tsdf_layer_data SET data=? WHERE index_x=? AND index_y=? AND index_z=?",
                    (a.tobytes(), ix, iy, iz))
        if not (a[:, 1] > 0).any():
            emptied.append((ix, iy, iz))
        # Every other present layer: same block, same voxel mask.
        for name in tables:
            if name == 'tsdf':
                continue
            spec = layer_spec(name)
            row = con.execute(
                f"SELECT data FROM {name}_layer_data WHERE index_x=? AND index_y=? AND index_z=?",
                (ix, iy, iz)).fetchone()
            if row is None:
                continue
            if spec['bytes'] is None:          # unknown layout -> drop in-box block
                con.execute(f"DELETE FROM {name}_layer_data WHERE index_x=? AND index_y=? AND index_z=?",
                            (ix, iy, iz))
                continue
            v = np.frombuffer(row[0], 'u1').reshape(NVOX, spec['bytes']).copy()
            wo = spec['weight_off']
            v[mask, wo:wo + 4] = 0
            con.execute(f"UPDATE {name}_layer_data SET data=? WHERE index_x=? AND index_y=? AND index_z=?",
                        (v.tobytes(), ix, iy, iz))
    for ix, iy, iz in emptied:
        for name in tables:
            con.execute(f"DELETE FROM {name}_layer_data WHERE index_x=? AND index_y=? AND index_z=?",
                        (ix, iy, iz))
    return cleared, len(emptied)


def denoise(con, min_blocks, surf_band, keep_boxes):
    """Delete 26-connected occupied-block components smaller than min_blocks.
    Blocks whose region overlaps a keep box are never removed. Returns one
    audit dict per removed component: blocks, voxels, centre (mean of its
    occupied voxel centres) and extent (their bounding box), all in base_link."""
    tables = layer_tables(con)
    bs = block_size(con, 'tsdf')
    vs = bs / VPS
    occ = {}
    for ix, iy, iz, blob in con.execute(
            "SELECT index_x,index_y,index_z,data FROM tsdf_layer_data").fetchall():
        m = tsdf_occupied_mask(blob, surf_band)
        if m.any():
            occ[(ix, iy, iz)] = voxel_centres(ix, iy, iz, bs)[m]
    nbr = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
           for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]
    seen, comps = set(), []
    for k in occ:
        if k in seen:
            continue
        q, comp = deque([k]), []
        seen.add(k)
        while q:
            cur = q.popleft()
            comp.append(cur)
            for d in nbr:
                n = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
                if n in occ and n not in seen:
                    seen.add(n)
                    q.append(n)
        comps.append(comp)

    def protected(b):
        ix, iy, iz = b
        c0 = np.array([ix, iy, iz]) * bs
        for x0, x1, y0, y1, z0, z1 in keep_boxes:
            if (c0[0] < x1 and c0[0] + bs > x0 and c0[1] < y1 and c0[1] + bs > y0
                    and c0[2] < z1 and c0[2] + bs > z0):
                return True
        return False

    removed = []
    for comp in comps:
        if len(comp) >= min_blocks:
            continue
        if any(protected(b) for b in comp):
            continue
        pts = np.vstack([occ[b] for b in comp])
        removed.append(dict(blocks=len(comp), voxels=len(pts),
                            centre=pts.mean(axis=0).tolist(),
                            extent=np.column_stack([pts.min(axis=0) - vs / 2,
                                                    pts.max(axis=0) + vs / 2]).tolist()))
        for ix, iy, iz in comp:
            for name in tables:
                con.execute(f"DELETE FROM {name}_layer_data WHERE index_x=? AND index_y=? AND index_z=?",
                            (ix, iy, iz))
    removed.sort(key=lambda r: (-r['blocks'], r['centre']))
    return removed


def format_audit(removed):
    """One line per removed component, in base_link metres."""
    lines = []
    for i, r in enumerate(removed, 1):
        cx, cy, cz = r['centre']
        (x0, x1), (y0, y1), (z0, z1) = r['extent']
        lines.append(f'    removed #{i}: {r["blocks"]} block(s), {r["voxels"]} voxel(s), '
                     f'centre ({cx:.3f}, {cy:.3f}, {cz:.3f}), '
                     f'extent x[{x0:.2f},{x1:.2f}] y[{y0:.2f},{y1:.2f}] z[{z0:.2f},{z1:.2f}]')
    return lines


def counts(con, surf_band):
    tot = con.execute("SELECT COUNT(*) FROM tsdf_layer_data").fetchone()[0]
    occ = 0
    for (blob,) in con.execute("SELECT data FROM tsdf_layer_data"):
        occ += int(tsdf_occupied_mask(blob, surf_band).sum())
    return tot, occ


def parse_box(s):
    v = [float(x) for x in s.split(',')]
    if len(v) != 6:
        raise argparse.ArgumentTypeError('box needs x0,x1,y0,y1,z0,z1')
    return (min(v[0], v[1]), max(v[0], v[1]), min(v[2], v[3]),
            max(v[2], v[3]), min(v[4], v[5]), max(v[4], v[5]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--remove-box', type=parse_box, action='append', default=[],
                    metavar='x0,x1,y0,y1,z0,z1', help='base_link box to clear (repeatable)')
    ap.add_argument('--denoise', action='store_true', help='drop isolated speckle')
    ap.add_argument('--min-blocks', type=int, default=4,
                    help='denoise: keep components with >= this many blocks (default 4)')
    ap.add_argument('--keep-box', type=parse_box, action='append', default=[],
                    metavar='x0,x1,y0,y1,z0,z1', help='denoise: never remove blocks here (repeatable)')
    ap.add_argument('--surf-band', type=float, default=0.02,
                    help='|TSDF distance| <= this (m) counts as observed surface')
    ap.add_argument('--self-check', action='store_true',
                    help='reload dst and assert no occupied voxel remains in any --remove-box')
    ap.add_argument('--force', action='store_true',
                    help='overwrite dst if it already exists (never the src file itself)')
    a = ap.parse_args(argv)
    if not a.remove_box and not a.denoise:
        ap.error('nothing to do: pass --remove-box and/or --denoise')
    if not os.path.isfile(a.src):
        ap.error(f'src not found: {a.src}')
    if os.path.exists(a.dst) and os.path.samefile(a.src, a.dst):
        ap.error(f'dst is the src file itself ({a.src}); the original is never modified')
    if os.path.exists(a.dst) and not a.force:
        ap.error(f'dst already exists: {a.dst} (pass --force to overwrite it)')

    shutil.copy2(a.src, a.dst)             # original stays untouched; edit the copy
    con = sqlite3.connect(a.dst)
    before = counts(con, a.surf_band)
    print(f'{a.src} -> {a.dst}')
    print(f'  before: {before[0]} tsdf blocks, {before[1]} occupied voxels')

    if a.remove_box:
        cleared, dropped = crop_boxes(con, a.remove_box, a.surf_band)
        print(f'  crop:   {len(a.remove_box)} box(es) -> cleared {cleared} voxels, dropped {dropped} emptied blocks')
    if a.denoise:
        removed = denoise(con, a.min_blocks, a.surf_band, a.keep_box)
        rb = sum(r['blocks'] for r in removed)
        print(f'  denoise:removed {len(removed)} components (< {a.min_blocks} blocks) = {rb} blocks'
              + (' — audit each one below (base_link m):' if removed else ''))
        for line in format_audit(removed):
            print(line)

    con.commit()
    con.execute('VACUUM')
    con.commit()
    after = counts(con, a.surf_band)
    print(f'  after:  {after[0]} tsdf blocks, {after[1]} occupied voxels '
          f'(-{before[0]-after[0]} blocks, -{before[1]-after[1]} voxels)')
    con.close()

    if a.self_check and a.remove_box:
        con = sqlite3.connect(a.dst)
        bs = block_size(con, 'tsdf')
        leaked = 0
        for ix, iy, iz, blob in con.execute(
                "SELECT index_x,index_y,index_z,data FROM tsdf_layer_data").fetchall():
            occ = tsdf_occupied_mask(blob, a.surf_band)
            if not occ.any():
                continue
            inb = in_any_box(voxel_centres(ix, iy, iz, bs), a.remove_box)
            leaked += int((occ & inb).sum())
        con.close()
        if leaked:
            sys.exit(f'SELF-CHECK FAILED: {leaked} occupied voxels remain inside a --remove-box')
        print('  self-check: OK — no occupied voxel remains in any --remove-box')
    return 0


if __name__ == '__main__':
    sys.exit(main())
