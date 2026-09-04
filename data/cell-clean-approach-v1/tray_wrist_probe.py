#!/usr/bin/env python3
"""
Background tray-carton tracker on the WRIST (hand-eye-calibrated) camera, for the
cell-clean approach demo. Adapts cams/approach/target_publisher.py: same YOLO +
"Obstacle-interior" cardboard filter (the eye-level viewpoint boxes cartons as
Obstacle, not tray), but retargeted to the live wrist upright stream and made a
TELEMETRY channel — it back-projects EVERY accepted carton to base_link via the
calibrated tool0->wrist_color_upright_optical_frame TF and logs class / conf /
3-D, so its detections can be cross-checked against the cleaned map's shelf.

It does NOT gate anything and moves nothing. Runs inside shelfbot_ros. This is
the exact invocation that produced tray-detections-base_link.json (model
tray_v5_best.pt; --base-tf is the base_link<-wrist_color_upright_optical_frame
snapshot of the parked arm, tool0_to_camera_2026-09-03.yaml + live TF):
  docker cp tray_wrist_probe.py shelfbot_ros:/root/
  docker exec -e ROS_DOMAIN_ID=0 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 shelfbot_ros \
    bash -lc 'source /opt/ros/jazzy/setup.bash && \
      /opt/yolo_venv/bin/python3 /root/tray_wrist_probe.py --secs 0 \
      --model /root/weights/tray_v5_best.pt \
      --base-tf=-0.776771,-0.154955,0.824915,-0.682359,0.197102,-0.105725,0.695960 \
      --json /root/tray_probe.json --annot /root/tray_annot.png'

--secs 0 runs forever (background); >0 runs a bounded probe then exits. Without
--base-tf the mount TF must be up (cams/cellmap/tools/publish_mount.py) so
optical->base_link resolves live; the JSON dump records which base_tf was used.
"""
import argparse
import collections
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

IMGSZ = 960
CONF = 0.25
PERIOD_S = 0.2
LOG_EVERY_S = 3.0
DEPTH_MIN_M, DEPTH_MAX_M = 0.15, 4.0
CORE_FRAC = 0.5
OBSTACLE_CLASS = 'Obstacle'
INTERIOR_MARGIN_PX = 6
MAX_AREA_FRAC = 0.40
BASE_FRAME = 'base_link'


class TrayProbe(Node):
    def __init__(self, model, accept, obstacle_fallback, args):
        super().__init__('tray_wrist_probe')
        self._model = model
        self._accept = accept
        self._obstacle_fallback = obstacle_fallback
        self._args = args
        self._last_run = self._last_log = 0.0
        self._class_counts = collections.Counter()
        self._depth = None
        self._info = None
        self._records = []                 # accepted dets over the run
        self._last_accept = []             # most recent frame's accepted dets
        self._tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf, self)
        self._pub_img = self.create_publisher(Image, '/cellmap/tray_annotated',
                                              qos_profile_sensor_data)
        self.create_subscription(Image, '/wrist/color_upright/image_raw',
                                 self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, '/wrist/aligned_depth_upright/image',
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/wrist/aligned_depth_upright/camera_info',
                                 self._on_info, qos_profile_sensor_data)

    def _on_depth(self, m):
        arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
        d = arr[:, :m.width * 4].copy().view(np.float32).reshape(m.height, m.width)
        self._depth = (time.time(), d)

    def _on_info(self, m):
        self._info = m

    @staticmethod
    def _interior(xyxy, w, h):
        x1, y1, x2, y2 = xyxy
        if (x1 < INTERIOR_MARGIN_PX or y1 < INTERIOR_MARGIN_PX
                or x2 > w - INTERIOR_MARGIN_PX or y2 > h - INTERIOR_MARGIN_PX):
            return False
        return (x2 - x1) * (y2 - y1) < MAX_AREA_FRAC * w * h

    def _back_project(self, xyxy, w, h):
        if self._depth is None or self._info is None:
            return None
        dt, depth = self._depth
        if time.time() - dt > 1.0:
            return None
        x1, y1, x2, y2 = xyxy
        sx, sy = depth.shape[1] / w, depth.shape[0] / h
        cxb, cyb = (x1 + x2) / 2 * sx, (y1 + y2) / 2 * sy
        hw, hh = (x2 - x1) / 2 * CORE_FRAC * sx, (y2 - y1) / 2 * CORE_FRAC * sy
        r0, r1 = max(0, int(cyb - hh)), min(depth.shape[0], int(cyb + hh) + 1)
        c0, c1 = max(0, int(cxb - hw)), min(depth.shape[1], int(cxb + hw) + 1)
        core = depth[r0:r1, c0:c1]
        good = core[np.isfinite(core) & (core > DEPTH_MIN_M) & (core < DEPTH_MAX_M)]
        if good.size < 20:
            return None
        z = float(np.median(good))
        k = self._info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        return np.array([(cxb - cx) * z / fx, (cyb - cy) * z / fy, z])

    def _to_base(self, xyz_opt):
        if self._args.base_tf:                 # fixed snapshot (arm parked, no motion)
            tx, ty, tz, x, y, z, w = self._args.base_tf
        else:
            try:
                tf = self._tf.lookup_transform(
                    BASE_FRAME, self._info.header.frame_id, rclpy.time.Time())
            except Exception:                                      # noqa: BLE001
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            tx, ty, tz = t.x, t.y, t.z
            x, y, z, w = q.x, q.y, q.z, q.w
        vx, vy, vz = xyz_opt
        # q * v * q^-1
        ax = 2 * (y * vz - z * vy)
        ay = 2 * (z * vx - x * vz)
        az = 2 * (x * vy - y * vx)
        rx = vx + w * ax + (y * az - z * ay)
        ry = vy + w * ay + (z * ax - x * az)
        rz = vz + w * az + (x * ay - y * ax)
        return np.array([rx + tx, ry + ty, rz + tz])

    def _on_rgb(self, m):
        now = time.time()
        if now - self._last_run < PERIOD_S:
            return
        self._last_run = now
        arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
        rgb = arr[:, :m.width * 3].copy().reshape(m.height, m.width, 3)
        r = self._model.predict(rgb[:, :, ::-1], imgsz=IMGSZ, conf=CONF, verbose=False)[0]
        accepted = []
        for b in r.boxes:
            name = r.names[int(b.cls)]
            self._class_counts[name] += 1
            xyxy = b.xyxy[0].tolist()
            ok = name in self._accept or (
                self._obstacle_fallback and name == OBSTACLE_CLASS
                and self._interior(xyxy, m.width, m.height))
            if not ok:
                continue
            xyz = self._back_project(xyxy, m.width, m.height)
            base = self._to_base(xyz) if xyz is not None else None
            accepted.append(dict(cls=name, conf=float(b.conf),
                                 bbox=[round(v, 1) for v in xyxy],
                                 area_frac=round((xyxy[2]-xyxy[0])*(xyxy[3]-xyxy[1])
                                                 / (m.width*m.height), 3),
                                 xyz_opt=None if xyz is None else [round(float(v), 3) for v in xyz],
                                 xyz_base=None if base is None else [round(float(v), 3) for v in base]))
        self._last_accept = accepted
        if accepted:
            self._records.append(dict(t=now, stamp=[m.header.stamp.sec, m.header.stamp.nanosec],
                                      dets=accepted))
        if now - self._last_log > LOG_EVERY_S:
            self._last_log = now
            base_pts = [d['xyz_base'] for d in accepted if d['xyz_base']]
            self.get_logger().info(
                f'accepted={len(accepted)} '
                + '; '.join(f"{d['cls']}:{d['conf']:.2f}"
                            + (f"@base{d['xyz_base']}" if d['xyz_base'] else "@base?")
                            for d in accepted)
                + f'  classes_seen={dict(self._class_counts)}')
            self._publish_annot(r, m)
            if self._args.annot:
                try:
                    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
                    plt.imsave(self._args.annot, np.ascontiguousarray(r.plot()[:, :, ::-1]))
                except Exception:                                  # noqa: BLE001
                    pass
            if self._args.json:
                self._dump()

    def _publish_annot(self, r, m):
        out = np.ascontiguousarray(r.plot()[:, :, ::-1])
        msg = Image()
        msg.header = m.header
        msg.height, msg.width = out.shape[0], out.shape[1]
        msg.encoding = 'rgb8'
        msg.step = out.shape[1] * 3
        msg.data = out.tobytes()
        self._pub_img.publish(msg)

    def _dump(self):
        # summary over recent records: cluster accepted dets by base_link xyz
        recent = [d for rec in self._records[-40:] for d in rec['dets'] if d['xyz_base']]
        clusters = []
        for d in recent:
            p = np.array(d['xyz_base'])
            for c in clusters:
                if np.linalg.norm(p - c['sum'] / c['n']) < 0.10:
                    c['sum'] += p; c['n'] += 1; c['conf'] = max(c['conf'], d['conf'])
                    c['cls'][d['cls']] += 1
                    break
            else:
                clusters.append(dict(sum=p.copy(), n=1, conf=d['conf'],
                                     cls=collections.Counter([d['cls']])))
        summary = [dict(xyz_base=[round(float(v), 3) for v in (c['sum'] / c['n'])],
                        hits=c['n'], max_conf=round(c['conf'], 2),
                        cls=dict(c['cls'])) for c in clusters]
        summary.sort(key=lambda s: -s['hits'])
        json.dump(dict(model=self._args.model, updated=time.time(),
                       base_tf=self._args.base_tf,
                       base_tf_note=('fixed base_link<-optical snapshot tx,ty,tz,qx,qy,qz,qw'
                                     if self._args.base_tf else 'live TF lookup per detection'),
                       classes_seen=dict(self._class_counts),
                       last_frame_accepted=self._last_accept,
                       carton_clusters=summary), open(self._args.json, 'w'), indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='/root/weights/tray_best_vm.pt')
    ap.add_argument('--classes', default='tray,broken_tray')
    ap.add_argument('--strict-classes', action='store_true')
    ap.add_argument('--secs', type=float, default=0.0, help='0 = run forever')
    ap.add_argument('--json', default='')
    ap.add_argument('--annot', default='')
    ap.add_argument('--base-tf', default='',
                    help='fixed base_link<-optical tx,ty,tz,qx,qy,qz,qw (arm parked); '
                         'snapshot it with system-python TF since the venv rmw misses /tf_static')
    a = ap.parse_args()
    a.base_tf = [float(v) for v in a.base_tf.split(',')] if a.base_tf else None
    from ultralytics import YOLO
    model = YOLO(a.model)
    rclpy.init()
    node = TrayProbe(model, set(a.classes.split(',')),
                     obstacle_fallback=not a.strict_classes, args=a)
    node.get_logger().info(f'tray_wrist_probe up: {a.model} on /wrist/color_upright/image_raw '
                           f'(Obstacle-interior fallback={not a.strict_classes})')
    t0 = time.time()
    try:
        while rclpy.ok() and (a.secs == 0 or time.time() - t0 < a.secs):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    if a.json:
        node._dump()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
