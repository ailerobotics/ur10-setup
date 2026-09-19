#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Query nvblox's ``get_esdf_and_gradient`` service over an AABB and report how
much of that box the planner currently sees as obstacle.

This is the measurement tool behind the ESDF proof: it asks nvblox for exactly
the ESDF cuMotion would consult, restricted to a small box around where an
obstacle is (or was), and counts the voxels whose signed distance is at/below a
threshold (<= 0 is inside an obstacle; a small positive band catches the surface
shell). Because it reads the SAME service cuMotion uses, "the obstacle is in the
ESDF" here means "cuMotion will route around it".

Used both as a library (``query_occupied``) and as a CLI that prints one JSON
line per poll, so ``proof/run_dynamic_esdf_proof.sh`` can assert appear/decay.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.node import Node

# nvblox fills unobserved ESDF voxels with esdf_and_gradients_unobserved_value
# (2.0 in our config). Anything at/below this is "observed"; at/below occ_thresh
# is obstacle.
UNOBSERVED_VALUE = 2.0


@dataclass
class EsdfQuery:
    occupied_voxels: int
    observed_voxels: int
    total_voxels: int
    min_distance_m: float          # most-inside-obstacle value seen (min esdf)
    voxel_size_m: float
    call_latency_s: float
    success: bool


def _parse_grid(resp) -> np.ndarray:
    """Return the ESDF distance field as a flat float array (one value per voxel).

    nvblox's EsdfAndGradients returns a dense grid whose ``layout.dim`` sizes give
    the grid shape. In this build the grid is 3D single-channel — ``dim = [x,y,z]``
    and ``len(data) == x*y*z`` — so EVERY value is an ESDF distance. Only when the
    layout carries an explicit small trailing channel dim (3/4/7, i.e. gradients
    packed with the distance) do we stride out channel 0. Getting this wrong reads
    a single z-layer and misses obstacles off that layer."""
    arr = np.asarray(resp.esdf_and_gradients.data, dtype=np.float32)
    dims = [d.size for d in resp.esdf_and_gradients.layout.dim]
    if arr.size == 0 or not dims:
        return arr
    prod = 1
    for d in dims:
        prod *= max(d, 1)
    if arr.size == prod:
        # dims fully describe the array. A small trailing dim = packed channels.
        if len(dims) >= 4 and dims[-1] in (3, 4, 7):
            return arr.reshape(-1, dims[-1])[:, 0]
        return arr                       # single ESDF value per voxel (our case)
    # data longer than the grid => channels not in dim; infer and take channel 0.
    channels = max(arr.size // prod, 1)
    if channels > 1 and arr.size % prod == 0:
        return arr.reshape(-1, channels)[:, 0]
    return arr


def query_occupied(node: Node, client, aabb_min, aabb_size,
                   occ_thresh_m: float = 0.0, frame_id: str = "base_link",
                   timeout_s: float = 5.0) -> Optional[EsdfQuery]:
    """Call the service for one AABB and summarise occupancy. Returns None if the
    service did not answer in time."""
    req = EsdfAndGradients.Request()
    req.update_esdf = True
    req.visualize_esdf = False
    req.use_aabb = True
    req.frame_id = frame_id
    req.aabb_min_m = Point(x=float(aabb_min[0]), y=float(aabb_min[1]), z=float(aabb_min[2]))
    req.aabb_size_m = Vector3(x=float(aabb_size[0]), y=float(aabb_size[1]), z=float(aabb_size[2]))
    t0 = time.monotonic()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    dt = time.monotonic() - t0
    if not future.done() or future.result() is None:
        return None
    resp = future.result()
    dims_dbg = [(d.label, d.size, d.stride) for d in resp.esdf_and_gradients.layout.dim]
    if os.environ.get("FM_DYN_ESDF_DEBUG"):
        print(json.dumps({"layout_dim": dims_dbg,
                          "data_len": len(resp.esdf_and_gradients.data),
                          "origin": [resp.origin_m.x, resp.origin_m.y, resp.origin_m.z],
                          "voxel": resp.voxel_size_m}), flush=True)
    grid = _parse_grid(resp)
    if grid.size == 0:
        return EsdfQuery(0, 0, 0, math.nan, float(resp.voxel_size_m), dt, bool(resp.success))
    observed = grid[grid < (UNOBSERVED_VALUE - 1e-3)]
    occupied = int(np.count_nonzero(grid <= occ_thresh_m))
    return EsdfQuery(
        occupied_voxels=occupied,
        observed_voxels=int(observed.size),
        total_voxels=int(grid.size),
        min_distance_m=float(np.min(grid)) if grid.size else math.nan,
        voxel_size_m=float(resp.voxel_size_m),
        call_latency_s=dt,
        success=bool(resp.success),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--service", default="/nvblox_node/get_esdf_and_gradient")
    ap.add_argument("--aabb-min", type=float, nargs=3, default=[-0.4, 0.4, -0.1])
    ap.add_argument("--aabb-size", type=float, nargs=3, default=[0.8, 0.6, 0.6])
    ap.add_argument("--occ-thresh", type=float, default=0.0)
    ap.add_argument("--frame", default="base_link")
    ap.add_argument("--poll-hz", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to poll; 0 = single shot")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("fm_dyn_esdf_probe")
    client = node.create_client(EsdfAndGradients, args.service)
    if not client.wait_for_service(timeout_sec=30.0):
        print(json.dumps({"error": f"service {args.service} not available"}), flush=True)
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    end = time.monotonic() + args.duration
    period = 1.0 / args.poll_hz if args.poll_hz > 0 else 0.5
    first = True
    while first or time.monotonic() < end:
        first = False
        q = query_occupied(node, client, args.aabb_min, args.aabb_size,
                           args.occ_thresh, args.frame)
        rec = {"t_wall": round(time.time(), 3)}
        rec.update({"error": "no response"} if q is None else {
            "occupied": q.occupied_voxels, "observed": q.observed_voxels,
            "total": q.total_voxels, "min_dist_m": round(q.min_distance_m, 4),
            "voxel_m": q.voxel_size_m, "latency_ms": round(q.call_latency_s * 1e3, 1),
            "success": q.success})
        print(json.dumps(rec), flush=True)
        if args.duration <= 0:
            break
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
