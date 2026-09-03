#!/usr/bin/env python3
"""
32FC1 metres -> 16UC1 millimetres, for the UPRIGHT wrist depth stream.

WHY: the scan pipeline's depth contract is 16UC1 millimetres - that is what
scan.launch.py feeds the robot segmenter and what the 2026-08-28 scan ran on.
The camera bring-up publishes a 16UC1 mm topic only for the RAW orientation
(/wrist/aligned_depth/image_mm); the 180deg-corrected stream the map must now
be built on exists only as 32FC1 metres. This bridges that one gap.

WHY CONVERT THE FLIPPED TOPIC rather than flip the mm topic: flipping the mm
image here would mean re-deriving the rotation, and a second implementation of
the same 180deg flip is a second chance to get it wrong - and it would be
paired with a camera_info this node did not produce. Converting
/wrist/aligned_depth_upright/image keeps exactly one flip in the system, the
one whose camera_info (mirrored cx/cy) the calibration was solved against.

Purely additive: it starts no camera and touches no existing node.

NaN and Inf become 0, which is the 16UC1 convention for "no reading" that
nvblox and the segmenter already expect from the RealSense - not a spurious
surface at 0 mm (0 is treated as invalid, not as a hit 0 m away).

  python3 depth_to_mm.py                      # upright depth -> ..._mm
  python3 depth_to_mm.py --in X --out Y
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

IN = '/wrist/aligned_depth_upright/image'
OUT = '/wrist/aligned_depth_upright/image_mm'


class ToMM(Node):
    def __init__(self, a):
        super().__init__('depth_to_mm')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Image, a.out, qos)
        self.create_subscription(Image, getattr(a, 'in'), self.cb, qos)
        self.n = 0
        self.warned = False
        self.get_logger().info(f'{getattr(a, "in")} (32FC1 m) -> {a.out} (16UC1 mm)')

    def cb(self, m):
        if m.encoding == '16UC1':
            self.pub.publish(m)                     # already the wanted form
            return
        if m.encoding != '32FC1':
            if not self.warned:
                self.get_logger().error(f'unexpected encoding {m.encoding}; expected 32FC1')
                self.warned = True
            return
        d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        mm = np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
        # Clip into uint16 range BEFORE the cast: numpy wraps on overflow, so a
        # stray large value would silently become a near-zero "surface in your
        # face" rather than an out-of-range reading.
        mm = np.clip(mm, 0, 65535).astype(np.uint16)
        out = Image()
        out.header = m.header                       # same stamp AND same frame_id
        out.height, out.width = m.height, m.width
        out.encoding = '16UC1'
        out.is_bigendian = 0
        out.step = m.width * 2
        out.data = mm.tobytes()
        self.pub.publish(out)
        self.n += 1
        if self.n % 300 == 0:
            valid = float((mm > 0).mean()) * 100.0
            self.get_logger().info(f'{self.n} frames, {valid:.0f}% valid pixels')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', default=IN)
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()
    rclpy.init()
    n = ToMM(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
