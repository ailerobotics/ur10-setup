#!/usr/bin/env python3
"""
Offline unit test for nvblx_clean.py — no ROS, no rig, no real map.

Builds a tiny synthetic .nvblx (the real SQLite schema, block_size 0.16, TSDF +
color layers) with hand-placed blocks, runs the tool, and checks that:
  * a box crops voxels by CENTRE, to 2 cm — voxels just outside survive;
  * a block emptied by the crop is dropped from every layer, including a
    <name>_layer_data table the tool has no layout for, and --denoise drops
    its speckle blocks from that unknown layer too;
  * --denoise removes a small isolated component but keeps a large one, honours
    --keep-box, and prints an audit line (centre / extent / block count in
    base_link) for exactly the components it removed;
  * --self-check passes on a good clean, and a voxel outside the box survives;
  * the source file is never modified and the output is valid loadable SQLite;
  * an existing dst is refused without --force and overwritten with it, and
    dst == src is refused even with --force.

Run standalone (`python3 test_nvblx_clean.py`) or under pytest.
"""
import os
import re
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


AUDIT_RE = re.compile(r'removed #(\d+): (\d+) block\(s\), (\d+) voxel\(s\), '
                      r'centre \(([-\d.]+), ([-\d.]+), ([-\d.]+)\), '
                      r'extent x\[([-\d.]+),([-\d.]+)\] y\[([-\d.]+),([-\d.]+)\] z\[([-\d.]+),([-\d.]+)\]')


def audit_lines(stdout):
    """Parsed --denoise audit: [(blocks, voxels, centre(3), extent(3x2)), ...]."""
    out = []
    for m in AUDIT_RE.finditer(stdout):
        v = [float(x) for x in m.groups()[1:]]
        out.append((int(v[0]), int(v[1]), tuple(v[2:5]),
                    ((v[5], v[6]), (v[7], v[8]), (v[9], v[10]))))
    return out


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


def add_unknown_layer(path, name, blocks):
    """Add a <name>_layer_data table (no layout known to the tool) holding an
    opaque blob for each block in `blocks`."""
    con = sqlite3.connect(path)
    con.execute(f"CREATE TABLE {name}_layer_metadata(param_name TEXT PRIMARY KEY "
                "UNIQUE NOT NULL,value_string TEXT,value_int INT,value_float FLOAT)")
    con.execute(f"CREATE TABLE {name}_layer_data(index_x INT NOT NULL,index_y INT "
                "NOT NULL,index_z INT NOT NULL,data BLOB,PRIMARY KEY(index_x,index_y,index_z))")
    con.execute(f"INSERT INTO {name}_layer_metadata VALUES('block_size',NULL,NULL,?)", (BS,))
    for ix, iy, iz in blocks:
        con.execute(f"INSERT INTO {name}_layer_data VALUES(?,?,?,?)",
                    (ix, iy, iz, bytes(range(16))))
    con.commit()
    con.close()


def layer_blocks(path, name):
    con = sqlite3.connect(path)
    out = set(con.execute(f"SELECT index_x,index_y,index_z FROM {name}_layer_data").fetchall())
    con.close()
    return out


def test_unknown_layer_follows_tsdf_on_crop_and_denoise():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        # blocks (1..5,0,0) form a 5-block line that survives --min-blocks 4;
        # block (1,0,0) is only PARTLY in the box (its vx=0 slab, centre x=0.17)
        # so it stays in the TSDF, while block (0,0,0) is emptied by the crop.
        make_map(src, {
            (0, 0, 0): [(1, 1, 1)],                                   # fully in box -> emptied
            (1, 0, 0): [(0, 0, 0), (7, 7, 7)],                        # partly in box -> stays in tsdf
            (2, 0, 0): [(0, 0, 0)], (3, 0, 0): [(0, 0, 0)],
            (4, 0, 0): [(0, 0, 0)], (5, 0, 0): [(0, 0, 0)],
            (20, 20, 20): [(0, 0, 0)],                                # lone speck -> denoised
        })
        assert 'mystery' not in nc.LAYERS
        add_unknown_layer(src, 'mystery', [(0, 0, 0), (1, 0, 0), (3, 0, 0),
                                           (5, 0, 0), (20, 20, 20)])
        # box covers block (0,0,0) entirely and only the vx=0 slab of block (1,0,0)
        r = run(src, dst, '--remove-box', '0,0.18,0,0.16,0,0.16', '--denoise',
                '--min-blocks', '4', '--self-check')
        assert r.returncode == 0, r.stderr + r.stdout
        occ = occupied_voxels(dst)
        assert (0, 0, 0) not in occ and (20, 20, 20) not in occ, occ
        assert occ.get((1, 0, 0)) == {(7, 7, 7)}, occ.get((1, 0, 0))
        left = layer_blocks(dst, 'mystery')
        assert (0, 0, 0) not in left, 'emptied block must leave the unknown layer too'
        assert (20, 20, 20) not in left, 'denoised speck must leave the unknown layer too'
        assert (1, 0, 0) not in left, 'in-box block of an unknown layout is dropped whole'
        assert left == {(3, 0, 0), (5, 0, 0)}, left
        # the known layers agree with the unknown one on which blocks survive
        assert layer_blocks(dst, 'color') == set(occ), (layer_blocks(dst, 'color'), set(occ))
        assert layer_blocks(dst, 'tsdf') == set(occ)


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
        # the audit lists exactly the one removed component, where it really was:
        # voxel (0,0,0) of block (20,20,20) has centre (20*8+0.5)*0.02 = 3.21 m
        audit = audit_lines(r.stdout)
        assert len(audit) == 1, r.stdout
        blocks, voxels, centre, extent = audit[0]
        assert (blocks, voxels) == (1, 1), audit[0]
        assert all(abs(c - 3.21) < 1e-6 for c in centre), centre
        for lo, hi in extent:
            assert abs(lo - 3.20) < 1e-6 and abs(hi - 3.22) < 1e-6, extent


def test_denoise_audit_reports_multi_block_component_extent():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        # a 2-block speck along +x: block (10,0,0) voxel (0,0,0) and block
        # (11,0,0) voxel (7,7,7) -> centres (1.61,0.01,0.01) and (1.91,0.15,0.15)
        make_map(src, {(10, 0, 0): [(0, 0, 0)], (11, 0, 0): [(7, 7, 7)],
                       (30, 30, 30): [(0, 0, 0)], (31, 30, 30): [(0, 0, 0)],
                       (32, 30, 30): [(0, 0, 0)], (33, 30, 30): [(0, 0, 0)]})
        r = run(src, dst, '--denoise', '--min-blocks', '4')
        assert r.returncode == 0, r.stderr + r.stdout
        audit = audit_lines(r.stdout)
        assert len(audit) == 1, r.stdout
        blocks, voxels, centre, extent = audit[0]
        assert (blocks, voxels) == (2, 2), audit[0]
        assert abs(centre[0] - 1.76) < 1e-6 and abs(centre[1] - 0.08) < 1e-6, centre
        assert abs(extent[0][0] - 1.60) < 1e-6 and abs(extent[0][1] - 1.92) < 1e-6, extent
        assert abs(extent[1][0] - 0.00) < 1e-6 and abs(extent[1][1] - 0.16) < 1e-6, extent
        occ = occupied_voxels(dst)
        assert (10, 0, 0) not in occ and (11, 0, 0) not in occ
        assert all((30 + i, 30, 30) in occ for i in range(4)), '4-block component must survive'


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


def test_existing_dst_refused_without_force_and_overwritten_with_it():
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, 'in.nvblx'), os.path.join(d, 'out.nvblx')
        make_map(src, {(0, 0, 0): [(1, 1, 1)], (5, 5, 5): [(2, 2, 2)]})
        r = run(src, dst, '--remove-box', '0,0.16,0,0.16,0,0.16')
        assert r.returncode == 0, r.stderr
        first = open(dst, 'rb').read()
        # second run to the same dst: refused, and the verified output is untouched
        r = run(src, dst, '--remove-box', '0.8,0.96,0.8,0.96,0.8,0.96')
        assert r.returncode != 0, 'existing dst must be refused without --force'
        assert 'already exists' in r.stderr, r.stderr
        assert open(dst, 'rb').read() == first, 'refused run must not touch dst'
        # with --force the new clean replaces it
        r = run(src, dst, '--remove-box', '0.8,0.96,0.8,0.96,0.8,0.96', '--force')
        assert r.returncode == 0, r.stderr
        occ = occupied_voxels(dst)
        assert (0, 0, 0) in occ and (5, 5, 5) not in occ, occ


def test_dst_equal_to_src_refused_even_with_force():
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, 'in.nvblx')
        make_map(src, {(0, 0, 0): [(1, 1, 1)]})
        before = open(src, 'rb').read()
        r = run(src, src, '--remove-box', '0,0.16,0,0.16,0,0.16', '--force')
        assert r.returncode != 0, 'src as dst must be refused'
        assert open(src, 'rb').read() == before, 'source map must be left untouched'


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    for t in tests:
        t()
        print(f'  ok  {t.__name__}')
    print(f'all {len(tests)} tests passed')


if __name__ == '__main__':
    main()
