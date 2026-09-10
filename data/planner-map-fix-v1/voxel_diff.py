#!/usr/bin/env python3
"""Value-level diff of two nvblx TSDF layers.

Proves the despeckle removed EXACTLY ONE occupied voxel and changed nothing
else: every block index, and every voxel's (distance, weight) pair, must be
identical between the two maps except at the single culprit voxel, whose weight
must drop from occupied to unobserved. Reports the exact count of occupied
voxels that changed and confirms no other occupied voxel moved.

occupied := TSDF weight >= 0.1 AND distance <= 0  (the report's ESDF-relevant
definition, from nvblox_cell.yaml). Runs on recorded files only; no ROS, no rig.
"""
import sqlite3
import sys

import numpy as np

VPS = 8
NVOX = VPS ** 3
BS = 0.16
VS = BS / VPS  # 0.02


def occupied_mask(a):
    # a: (512,2) float32 -> {distance, weight}
    return (a[:, 1] >= 0.1) & (a[:, 0] <= 0.0)


def load_blocks(path):
    con = sqlite3.connect(path)
    blocks = {}
    for ix, iy, iz, blob in con.execute(
            "SELECT index_x,index_y,index_z,data FROM tsdf_layer_data").fetchall():
        blocks[(ix, iy, iz)] = np.frombuffer(blob, '<f4').reshape(NVOX, 2)
    con.close()
    return blocks


def centre(ix, iy, iz, L):
    vx, vy, vz = L // 64, (L // 8) % 8, L % 8
    return ((ix * VPS + vx + 0.5) * VS,
            (iy * VPS + vy + 0.5) * VS,
            (iz * VPS + vz + 0.5) * VS)


def main():
    a_path, b_path = sys.argv[1], sys.argv[2]
    A = load_blocks(a_path)   # original
    B = load_blocks(b_path)   # despeckled
    print(f'original:   {a_path}')
    print(f'despeckled: {b_path}')
    print(f'blocks: original={len(A)}  despeckled={len(B)}')

    only_a = set(A) - set(B)
    only_b = set(B) - set(A)
    print(f'blocks only in original: {len(only_a)}   only in despeckled: {len(only_b)}')

    occ_a_total = sum(int(occupied_mask(v).sum()) for v in A.values())
    occ_b_total = sum(int(occupied_mask(v).sum()) for v in B.values())
    print(f'occupied voxels (w>=0.1 & d<=0): original={occ_a_total}  despeckled={occ_b_total}  delta={occ_a_total-occ_b_total}')

    # Per-voxel diff over shared blocks.
    became_free = []      # occupied in A, not in B
    became_occ = []       # occupied in B, not in A
    other_value_changes = 0   # any voxel whose (d,w) changed but not covered above
    changed_voxels = 0
    for k in set(A) & set(B):
        a, b = A[k], B[k]
        # occupancy transitions
        ma, mb = occupied_mask(a), occupied_mask(b)
        for L in np.where(ma & ~mb)[0]:
            became_free.append((k, int(L), centre(*k, int(L)),
                                tuple(a[L]), tuple(b[L])))
        for L in np.where(mb & ~ma)[0]:
            became_occ.append((k, int(L), centre(*k, int(L)),
                               tuple(a[L]), tuple(b[L])))
        # count ALL voxels whose bytes differ at all
        diff = np.where((a[:, 0] != b[:, 0]) | (a[:, 1] != b[:, 1]))[0]
        changed_voxels += len(diff)

    # blocks present in only one map: any occupied voxel there is a change
    for k in only_a:
        became_free.extend((k, int(L), centre(*k, int(L)), tuple(A[k][L]), None)
                           for L in np.where(occupied_mask(A[k]))[0])
    for k in only_b:
        became_occ.extend((k, int(L), centre(*k, int(L)), None, tuple(B[k][L]))
                          for L in np.where(occupied_mask(B[k]))[0])

    print()
    print(f'TOTAL voxels whose (distance,weight) bytes changed at all: {changed_voxels}')
    print(f'occupied -> unobserved (removed): {len(became_free)}')
    print(f'unobserved -> occupied (added):   {len(became_occ)}')
    print()
    for k, L, c, va, vb in became_free:
        print(f'  REMOVED block{k} L={L} centre=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) '
              f'before(d,w)={va} after(d,w)={vb}')
    for k, L, c, va, vb in became_occ:
        print(f'  ADDED   block{k} L={L} centre=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) '
              f'before(d,w)={va} after(d,w)={vb}')

    ok = (len(became_free) == 1 and len(became_occ) == 0 and changed_voxels == 1)
    print()
    print('VERDICT:', 'PASS — exactly one occupied voxel removed, no other voxel changed'
          if ok else 'FAIL — more than one voxel changed')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
