#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Live plumbing check for the human_safety graph.

Subscribes for a fixed window and reports, per camera: colour-frame rate and
newest-stamp age (fresh vs latched — the RealSense republishes its last frame
when dead), detector inference Hz + GPU memory (from the detector status topic),
whether it contributes a position or presence only; plus the end-to-end latency
from image stamp to published /human_safety/state, the state rate, and the
heartbeat advance. Exit 0 iff every configured camera produced fresh frames and
the state topic advanced.

Run on the isolated domain (77) with the graph already up (see run_live.sh):
  ROS_DOMAIN_ID=77 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    python3 cams/human_safety/tools/live_check.py --config <yaml> --secs 8
"""
import argparse
import json
import pathlib
import sys
import time

_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from human_safety.config import load_config  # noqa: E402


class Checker(Node):
    def __init__(self, cfg_path):
        super().__init__("human_safety_live_check")
        self.cfg = load_config(cfg_path)
        self.roles = [c.role for c in self.cfg.cameras]
        self.img_times = {r: [] for r in self.roles}
        self.img_stamp_age = {r: None for r in self.roles}
        self.status = {r: None for r in self.roles}
        self.state_times = []
        self.latencies = []
        self.heartbeats = []
        for c in self.cfg.cameras:
            self.create_subscription(Image, c.color_topic,
                                     self._img_cb(c.role), qos_profile_sensor_data)
            self.create_subscription(String, f"/human_safety/status/{c.role}",
                                     self._status_cb(c.role), 10)
        self.create_subscription(String, "/human_safety/state", self._state_cb, 10)

    def _img_cb(self, role):
        def cb(msg):
            now = time.time()
            self.img_times[role].append(now)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.img_stamp_age[role] = now - stamp
        return cb

    def _status_cb(self, role):
        def cb(msg):
            try:
                self.status[role] = json.loads(msg.data)
            except Exception:  # noqa: BLE001
                pass
        return cb

    def _state_cb(self, msg):
        now = time.time()
        self.state_times.append(now)
        try:
            s = json.loads(msg.data)
            if s.get("evidence_stamp"):
                self.latencies.append(now - s["evidence_stamp"])
            self.heartbeats.append(s.get("heartbeat_seq"))
        except Exception:  # noqa: BLE001
            pass


def _hz(times, window):
    return len(times) / window if window > 0 else 0.0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--secs", type=float, default=8.0)
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Checker(args.config)
    t0 = time.time()
    while time.time() - t0 < args.secs and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    window = time.time() - t0

    print(f"\n=== human_safety live check ({window:.1f}s window) ===")
    ok = True
    for r in node.roles:
        hz = _hz(node.img_times[r], window)
        age = node.img_stamp_age[r]
        st = node.status[r] or {}
        fresh = age is not None and age < 0.5
        cam_ok = hz > 0.5 and fresh
        ok = ok and cam_ok
        print(f"[{r:6}] colour {hz:5.1f} Hz  stamp_age "
              f"{'n/a' if age is None else f'{age:.3f}s'}  "
              f"{'FRESH' if fresh else 'STALE/absent'}  "
              f"infer {st.get('infer_hz', 0):.1f} Hz  "
              f"gpu {st.get('gpu_mem_mb', 0):.0f} MB  "
              f"{'POSITIONED' if st.get('positioned') else 'presence-only'}")
    st_hz = _hz(node.state_times, window)
    lat = node.latencies
    hbs = [h for h in node.heartbeats if h is not None]
    hb_ok = len(hbs) >= 2 and hbs[-1] > hbs[0]
    if lat:
        lat_s = sorted(lat)
        print(f"[state ] {st_hz:5.1f} Hz  latency img->state med "
              f"{lat_s[len(lat_s)//2]*1000:.0f} ms  max {max(lat)*1000:.0f} ms  "
              f"heartbeat {'advancing' if hb_ok else 'STUCK'}")
    else:
        print(f"[state ] {st_hz:5.1f} Hz  (no evidence_stamp / no state)")
        ok = False
    ok = ok and st_hz >= 1.0 and hb_ok
    print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
