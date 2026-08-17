#!/usr/bin/env python3
"""Grab one RGB frame + one metric depth frame from the rig; save PNG + stats."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame.png'


class Grab(Node):
    def __init__(self):
        super().__init__('frame_grab')
        self.rgb = None
        self.depth = None
        self.info = None
        self.create_subscription(Image, '/camera/rgb', self._rgb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image', self._depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self._info, qos_profile_sensor_data)

    def _rgb(self, m):
        self.rgb = m

    def _depth(self, m):
        self.depth = m

    def _info(self, m):
        self.info = m


def img_to_np(m, channels, dtype):
    itemsize = np.dtype(dtype).itemsize
    arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    arr = arr[:, :m.width * channels * itemsize].copy()
    return arr.view(dtype).reshape(m.height, m.width, channels)


def main():
    rclpy.init()
    n = Grab()
    end = time.time() + 20
    while rclpy.ok() and time.time() < end and (
            n.rgb is None or n.depth is None or n.info is None):
        rclpy.spin_once(n, timeout_sec=0.5)

    if n.rgb is None:
        print('NO RGB FRAME')
        sys.exit(1)
    rgb = img_to_np(n.rgb, 3, np.uint8)
    print(f'rgb: {n.rgb.width}x{n.rgb.height} {n.rgb.encoding}')

    saved = None
    try:
        import cv2
        cv2.imwrite(OUT, rgb[:, :, ::-1])
        saved = OUT
    except ImportError:
        try:
            from PIL import Image as PImage
            PImage.fromarray(rgb).save(OUT)
            saved = OUT
        except ImportError:
            saved = OUT.rsplit('.', 1)[0] + '.ppm'
            with open(saved, 'wb') as f:
                f.write(b'P6\n%d %d\n255\n' % (n.rgb.width, n.rgb.height))
                f.write(rgb.tobytes())
    print(f'SAVED {saved}')

    if n.depth is not None:
        d = img_to_np(n.depth, 1, np.float32)[:, :, 0]
        valid = d[np.isfinite(d) & (d > 0)]
        if valid.size:
            print(f'depth: {n.depth.encoding} valid={valid.size} '
                  f'min={valid.min():.2f}m med={np.median(valid):.2f}m '
                  f'max={valid.max():.2f}m')
        else:
            print(f'depth: {n.depth.encoding} NO VALID PIXELS')
    else:
        print('NO DEPTH FRAME (32FC1 converter output missing?)')

    if n.info is not None:
        k = n.info.k
        print(f'camera_info: fx={k[0]:.1f} fy={k[4]:.1f} '
              f'cx={k[2]:.1f} cy={k[5]:.1f}')
    else:
        print('NO CAMERA_INFO')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
