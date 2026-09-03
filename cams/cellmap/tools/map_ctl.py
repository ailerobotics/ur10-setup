#!/usr/bin/env python3
"""
nvblox map services from inside the container (nvblox_msgs is not on the host):

  map_ctl.py save /maps/cell-2026-08-26.nvblx     # TSDF+colour+ESDF, whole map
  map_ctl.py load /maps/cell-2026-08-26.nvblx
  map_ctl.py ply  /maps/cell-2026-08-26.ply       # mesh, for viewers/screenshots
  map_ctl.py esdf                                  # one ESDF query, prints grid stats

Exit code 0 only when the service reports success — planner.launch.py gates
cuMotion on this.  From the host:
  docker exec ur10-cellmap-scan bash -lc 'source /opt/ros/jazzy/setup.bash && \
      python3 /cellmap/tools/map_ctl.py save /maps/cell-$(date +%F).nvblx'
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nvblox_msgs.srv import EsdfAndGradients, FilePath

SRV = {'save': '/nvblox_node/save_map', 'load': '/nvblox_node/load_map',
       'ply': '/nvblox_node/save_ply'}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['save', 'load', 'ply', 'esdf'])
    ap.add_argument('path', nargs='?')
    ap.add_argument('--wait', type=float, default=30, help='s to wait for the service')
    ap.add_argument('--node', default='/nvblox_node')
    ap.add_argument('--spheres', help='esdf: json [{state,link,c:[x,y,z],r}] -> ESDF clearance per sphere')
    ap.add_argument('--probe', action='append', default=[],
                    help='esdf: print the distance column above x,y (base_link m)')
    a = ap.parse_args()
    if a.cmd != 'esdf' and not a.path:
        sys.exit('path required')
    if a.cmd == 'load' and not os.path.exists(a.path):
        sys.exit(f'load: {a.path} does not exist (inside the container)')

    rclpy.init()
    node = Node('map_ctl')
    t0 = time.time()
    if a.cmd == 'esdf':
        cli = node.create_client(EsdfAndGradients, a.node + '/get_esdf_and_gradient')
    else:
        cli = node.create_client(FilePath, SRV[a.cmd].replace('/nvblox_node', a.node))
    while not cli.wait_for_service(timeout_sec=1.0):
        if time.time() - t0 > a.wait:
            sys.exit(f'service {cli.srv_name} not available after {a.wait:.0f} s')
        print(f'waiting for {cli.srv_name} ...', flush=True)

    if a.cmd == 'esdf':
        req = EsdfAndGradients.Request()
        req.update_esdf = True
        req.visualize_esdf = True
        req.use_aabb = False
        req.frame_id = 'base_link'
    else:
        req = FilePath.Request()
        req.file_path = a.path
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=300)
    res = fut.result()
    if res is None:
        sys.exit(f'{a.cmd}: no response')
    if a.cmd == 'esdf':
        dims = [(d.label, d.size) for d in res.esdf_and_gradients.layout.dim]
        data = np.asarray(res.esdf_and_gradients.data, np.float32)
        print(f'esdf: success={res.success} frame={res.header.frame_id} '
              f'origin=({res.origin_m.x:.2f},{res.origin_m.y:.2f},{res.origin_m.z:.2f}) '
              f'voxel={res.voxel_size_m} dims={dims} values={data.size}')
        if data.size:
            n = len(dims)
            if n >= 3:
                shape = [s for _, s in dims]
                grid = data.reshape(shape)
                dist = grid[..., 0] if grid.ndim == 4 else grid
                # COVERAGE CANNOT BE COUNTED HERE ANY MORE, and pretending it can is
                # dangerous. This used to test dist < -999 for nvblox's -1000
                # "never observed" sentinel, but nvblox_cell.yaml deliberately sets
                # esdf_and_gradients_unobserved_value: 2.0 so unseen space reads as
                # FREE to cuMotion (otherwise the arm's own unobservable base makes
                # every start state invalid). Unobserved voxels therefore come back
                # as +2.0 - and so does every genuinely clear voxel, because
                # esdf_integrator_max_distance_m is also 2.0. The two are
                # indistinguishable in this query.
                #
                # The old test silently reported "unobserved 0 (0 %)", which reads as
                # "the whole cell is mapped". That is the worst possible direction to
                # be wrong in: unobserved is treated as free, so a reader would trust
                # a map to protect them in space the camera never saw.
                unobs_val = 2.0
                at_max = np.isclose(dist, unobs_val)
                occupied = (dist <= 0).sum()
                print(f'  voxels {dist.size}: occupied(<=0) {occupied}, '
                      f'at the +{unobs_val:g} ceiling {at_max.sum()} '
                      f'({at_max.mean()*100:.0f} %) = unobserved OR >= {unobs_val:g} m clear '
                      f'(indistinguishable - see the comment in map_ctl.py), '
                      f'distance range {dist.min():.3f}..{dist.max():.3f} m')
                print('  NOTE: coverage is not measurable from this query; judge it from the '
                      'mesh (tools/verify_map.py) and remember unobserved = FREE to the planner.')
                o = np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z])
                vs = res.voxel_size_m
                if a.spheres:
                    import json
                    for sp in json.load(open(a.spheres)):
                        idx = ((np.array(sp['c']) - o) / vs).astype(int)
                        inside = all(0 <= idx[k] < dist.shape[k] for k in range(3))
                        d = float(dist[tuple(idx)]) if inside else float('nan')
                        clr = d - sp['r']
                        flag = '' if (d < -999 or clr >= 0) else '  <-- COLLISION'
                        if d < -999:
                            flag = '  (unobserved)'
                        print(f"  {sp['state']:5s} {sp['link']:15s} c={sp['c']} r+buf={sp['r']:.3f} "
                              f"esdf={d:+.3f} clearance={clr:+.3f}{flag}")
                for pr in a.probe:
                    x, y = (float(v) for v in pr.split(','))
                    i, j = int((x - o[0]) / vs), int((y - o[1]) / vs)
                    col = dist[i, j, :]
                    print(f'  column x={x} y={y}: ' + ' '.join(
                        f'z{o[2]+(k+.5)*vs:+.2f}:{col[k]:+.3f}' for k in range(len(col))
                        if -0.3 < o[2] + (k + .5) * vs < 0.8))
        sys.exit(0 if res.success else 1)
    ok = bool(res.success)
    size = os.path.getsize(a.path) if ok and os.path.exists(a.path) else None
    print(f'{a.cmd} {a.path}: {"OK" if ok else "FAILED"}'
          + (f' ({size/1e6:.1f} MB)' if size else '') + f' in {time.time()-t0:.1f} s')
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
