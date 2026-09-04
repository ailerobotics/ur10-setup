#!/usr/bin/env python3
"""
Offline unit test for nvblx_clean.py — no ROS, no rig, no real map.

Builds a tiny synthetic .nvblx (the real SQLite schema, block_size 0.16, TSDF +
color layers) with hand-placed blocks, runs the tool, and checks that:
  * a box crops voxels by CENTRE, to 2 cm — voxels just outside survive;
  * a block emptied by the crop is dropped from every layer;
  * --denoise removes a small isolated component but keeps a large one, and
    honours --keep-box;
  * --self-check passes on a good clean, and a voxel outside the box survives;
  * the source file is never modified and the output is valid loadable SQLite.

Run standalone (`python3 test_nvblx_clean.py`) or under pytest.
"""
import os
import sqlite3
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, 'nvblx_clean.py')
sys.path.insert(0, HERE)
import nvblx_clean as nc            # noqa: E402

BS = 0.16
VS = BS / nc.VPS                    # 0.02


def lin(vx, vy, vz):
    """Voxel (vx,vy,vz) -> BLOB linear index, matching nvblx_clean's layout."""
    return vx * 64 + vy * 8 + vz


def tsdf_blob(occupied):
    """occupied: iterable of (vx,vy,vz) -> that voxel gets weight 1, dist 0."""
    a = np.zeros((nc.NVOX, 2), '<f4')      # weight 0 everywhere = unobserved
    for v in occupied:
        a[lin(*v)] = (0.0, 1.0)
    return a.tobytes()


def color_blob(occupied):
    a = np.zeros(nc.NVOX, dtype=[('rgba', 'u1', 4), ('w', '<f4')])
    for v in occupied:
        a[lin(*v)] = ((200, 100, 50, 255), 1.0)
    return a.tobytes()


def make_map(path, tsdf_blocks):
    """tsdf_blocks: {(ix,iy,iz): [ (vx,vy,vz), ... ]}."""
    con = sqlite3.connect(path)
    for layer in ('tsdf', 'color', 'esdf', 'occupancy', 'freespace', 'feature'):
        con.execute(f"CREATE TABLE {layer}_layer_metadata(param_name TEXT PRIMARY KEY "
                    "UNIQUE NOT NULL,value_string TEXT,value_int INT,value_float FLOAT)")
        con.execute(f"CREATE TABLE {layer}_layer_data(index_x INT NOT NULL,index_y INT "
                    "NOT NULL,index_z INT NOT NULL,data BLOB,PRIMARY KEY(index_x,index_y,index_z))")
        con.execute(f"INSERT INTO {layer}_layer_metadata VALUES('type',?,NULL,NULL)", (f'{layer}_layer',))
        con.execute(f"INSERT INTO {layer}_layer_metadata VALUES('block_size',NULL,NULL,?)", (BS,))
    for (ix, iy, iz), occ in tsdf_blocks.items():
        con.execute("INSERT INTO tsdf_layer_data VALUES(?,?,?,?)", (ix, iy, iz, tsdf_blob(occ)))
        con.execute("INSERT INTO color_layer_data VALUES(?,?,?,?)", (ix, iy, iz, color_blob(occ)))
    con.commit()
    con.close()


def occupied_voxels(path):
    """{(ix,iy,iz): set of occupied (vx,vy,vz)} from a map's TSDF."""
    con = sqlite3.connect(path)
    out = {}
    for ix, iy, iz, blob in con.execute("SELECT index_x,index_y,index_z,data FROM tsdf_layer_data"):
        a = np.frombuffer(blob, '<f4').reshape(nc.NVOX, 2)
        occ = {(L // 64, (L // 8) % 8, L % 8) for L in np.where(a[:, 1] > 0)[0]}
        if occ:
            out[(ix, iy, iz)] = occ
    con.close()
    return out


def run(*args):
    return subprocess.run([sys.executable, TOOL, *args], capture_output=True, text=True)


def test_crop_is_voxel_precise_and_drops_emptied_blocks():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        # block (0,0,0) spans world [0,0.16)^3; centres are (v+0.5)*0.02.
        # box z<0.05 keeps only vz in {0,1} (centres .01,.03); vz=2 centre .05 is out.
        make_map(src, {
            (0, 0, 0): [(3, 3, 0), (3, 3, 1), (3, 3, 2)],   # straddles z=0.05 plane
            (5, 5, 5): [(4, 4, 4)],                          # far outside box: must survive
        })
        r = run(src, dst, '--remove-box', '0,0.16,0,0.16,0,0.05', '--self-check')
        assert r.returncode == 0, r.stderr + r.stdout
        occ = occupied_voxels(dst)
        # in-box voxels (vz 0,1) gone; vz=2 (centre 0.05, not < 0.05) kept
        assert occ.get((0, 0, 0)) == {(3, 3, 2)}, occ.get((0, 0, 0))
        # the untouched far block is exactly as built
        assert occ.get((5, 5, 5)) == {(4, 4, 4)}


def test_fully_cropped_block_is_removed_everywhere():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        make_map(src, {(0, 0, 0): [(1, 1, 1), (2, 2, 2)], (5, 5, 5): [(0, 0, 0)]})
        r = run(src, dst, '--remove-box', '0,0.16,0,0.16,0,0.16', '--self-check')
        assert r.returncode == 0, r.stderr + r.stdout
        con = sqlite3.connect(dst)
        for layer in ('tsdf', 'color'):
            row = con.execute(f"SELECT 1 FROM {layer}_layer_data WHERE index_x=0 AND index_y=0 AND index_z=0").fetchone()
            assert row is None, f'{layer} block (0,0,0) should be dropped'
        con.close()


def test_denoise_drops_small_keeps_large_and_keep_box():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        cluster = {(ix, 0, 0): [(0, 0, 0)] for ix in range(6)}        # 6-block line
        blocks = dict(cluster)
        blocks[(20, 20, 20)] = [(0, 0, 0)]                            # lone speck -> drop
        blocks[(-20, -20, -20)] = [(0, 0, 0)]                         # lone speck in keep-box
        make_map(src, blocks)
        keep = f'{-20*BS},{-20*BS+BS},{-20*BS},{-20*BS+BS},{-20*BS},{-20*BS+BS}'
        # negative-leading box values must use --flag=value so argparse does not
        # read them as options (the same holds for --remove-box in real use).
        r = run(src, dst, '--denoise', '--min-blocks', '4', f'--keep-box={keep}')
        assert r.returncode == 0, r.stderr + r.stdout
        occ = occupied_voxels(dst)
        assert (20, 20, 20) not in occ, 'isolated speck should be denoised'
        assert (-20, -20, -20) in occ, 'speck inside --keep-box must survive'
        for ix in range(6):
            assert (ix, 0, 0) in occ, 'large component must survive'


def test_self_check_passes_and_voxel_outside_box_survives():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        make_map(src, {(0, 0, 0): [(7, 7, 7)]})     # centre (0.15,0.15,0.15)
        # box misses the voxel; self-check is over the SAME box, so it must pass
        # and the voxel just outside must remain — proving the crop is bounded.
        r = run(src, dst, '--remove-box', '0,0.05,0,0.05,0,0.05', '--self-check')
        assert r.returncode == 0, r.stderr
        assert 'self-check: OK' in r.stdout
        assert (0, 0, 0) in occupied_voxels(dst), 'voxel outside the box must remain'


def test_source_untouched_and_output_valid():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        make_map(src, {(0, 0, 0): [(1, 1, 1)], (5, 5, 5): [(2, 2, 2)]})
        before = occupied_voxels(src)
        r = run(src, dst, '--remove-box', '0,0.16,0,0.16,0,0.16')
        assert r.returncode == 0, r.stderr
        assert occupied_voxels(src) == before, 'source map must be left untouched'
        # output opens as SQLite and keeps the schema
        con = sqlite3.connect(dst)
        tabs = {r[0] for r in con.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        assert 'tsdf_layer_data' in tabs and 'color_layer_metadata' in tabs
        con.close()


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    for t in tests:
        t()
        print(f'  ok  {t.__name__}')
    print(f'all {len(tests)} tests passed')


if __name__ == '__main__':
    main()
