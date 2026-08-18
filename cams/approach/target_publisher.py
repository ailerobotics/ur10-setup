#!/usr/bin/env python3
"""
Detector bridge for the eye-in-hand approach demo (v1).

Runs INSIDE the shelfbot_ros container (ultralytics venv + ROS overlay),
adapting cams/tools/tray_live_loop.py into a target source:

  /camera/rgb (rgb8)  --YOLO tray_best_vm.pt-->  best cardboard bbox
  /camera/depth/image (32FC1, aligned to RGB)  --median over bbox core-->  Z
  /camera/depth/camera_info  --back-projection-->  (X, Y, Z) optical frame

Publishes:
  /approach/target_camera   geometry_msgs/PointStamped, CAMERA OPTICAL frame,
                            only while the target is VALID (see below)
  /detect_new/annotated     annotated frames, always (RViz bench view)

VALIDITY — a moving arm must never yield a target. A target is published only
when ALL hold:
  * the last N_CONSEC (5) accepted detections are each < MAX_GAP_S apart,
  * their 3-D spread (max distance from the window mean) < SPREAD_M (3 cm),
  * gyro stillness: every /camera/imu sample in the last STILL_WIN_S had
    |angular velocity| < GYRO_STILL_RAD_S (0.03 rad/s ~ 1.7 deg/s; D435i
    gyro noise at rest is ~0.005-0.01 rad/s, real arm motion is >> 0.1),
  * IMU data is FRESH (< IMU_STALE_S old). No IMU => no target, by design;
    --no-imu exists for bench-only debugging and prints a warning banner.

Class filter: the model's classes are {0 tray(=cardboard), 1 broken_tray,
2 Grabable, 3 Obstacle, 4 Items}. LIVE FINDING (2026-08-18, eye-level wrist
view of the gondola at ~1.6 m): the model NEVER fires tray/Grabable from
this viewpoint — even at conf 0.05 — but boxes the blue cardboard yogurt
tray as `Obstacle` (0.39-0.58). It was trained on near-overhead tray views;
frontal cardboard reads as Obstacle. The other Obstacle hits are shelf
structure whose bboxes are truncated at the image border (uprights, board
strips). Workable filter, chosen from that evidence:
  * tray / broken_tray: accepted unconditionally (cardboard by definition),
  * Obstacle: accepted ONLY if its bbox is fully interior to the frame
    (INTERIOR_MARGIN_PX) and covers < MAX_AREA_FRAC of the image — a
    free-standing cardboard target sits fully in view; truncated or huge
    boxes are shelf structure and are rejected.
Every class seen live is logged with counts so the choice stays auditable.
--strict-classes disables the Obstacle fallback; --classes overrides the
unconditional set.

Run (host side):
  docker start shelfbot_ros
  docker cp cams/approach/target_publisher.py shelfbot_ros:/root/
  docker exec -e ROS_DOMAIN_ID=0 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 shelfbot_ros \
    bash -c "source /opt/ros/jazzy/setup.bash && source /ros_ws/install_container/setup.bash \
             && /opt/yolo_venv/bin/python3 /root/target_publisher.py"
"""
import argparse
import collections
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, Imu

DEFAULT_MODEL = '/root/weights/tray_best_vm.pt'
IMGSZ = 960
CONF = 0.25
PERIOD_S = 0.15            # ~6.7 Hz inference (GPU loafs; more shots at the
                           # threshold-hovering tray detection)
LOG_EVERY_S = 2.0

N_CONSEC = 5               # consecutive detections needed
MAX_GAP_S = 2.0            # max age gap between consecutive detections
                           # (live conf hovers around 0.25 -> hits are
                           # intermittent; stillness + spread still gate)
SPREAD_M = 0.03            # 3 cm max spread across the window
GYRO_STILL_RAD_S = 0.03    # stillness threshold, documented above
STILL_WIN_S = 1.0          # gyro must be still over this whole window
IMU_STALE_S = 0.5          # IMU silence longer than this invalidates

DEPTH_MIN_M = 0.15         # D435i min range; below is garbage
DEPTH_MAX_M = 4.0          # beyond the cell; reject
CORE_FRAC = 0.5            # central fraction of the bbox used for depth

OBSTACLE_CLASS = 'Obstacle'
INTERIOR_MARGIN_PX = 6     # Obstacle bbox must clear the frame border by this
MAX_AREA_FRAC = 0.40       # ... and cover less than this much of the image


class TargetPublisher(Node):
    def __init__(self, model, classes, use_imu, obstacle_fallback):
        super().__init__('approach_target_publisher')
        self._model = model
        self._accept = classes            # unconditionally accepted names
        self._obstacle_fallback = obstacle_fallback
        self._use_imu = use_imu
        self._last_run = 0.0
        self._last_log = 0.0
        self._class_counts = collections.Counter()
        self._depth = None                # (stamp_s, np float32 HxW)
        self._info = None                 # CameraInfo
        self._gyro = collections.deque()  # (t_recv, |w|)
        self._window = collections.deque(maxlen=N_CONSEC)  # (t, xyz)

        self._pub_target = self.create_publisher(
            PointStamped, '/approach/target_camera', 10)
        self._pub_img = self.create_publisher(
            Image, '/detect_new/annotated', qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/rgb', self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image', self._on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._on_info,
            qos_profile_sensor_data)
        if use_imu:
            self.create_subscription(
                Imu, '/camera/imu', self._on_imu, qos_profile_sensor_data)

    # -- sensor caches -------------------------------------------------------
    def _on_depth(self, m):
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
        d = arr[:, :m.width * 4].copy().view(np.float32)
        self._depth = (time.time(), d.reshape(m.height, m.width))

    def _on_info(self, m):
        self._info = m

    def _on_imu(self, m):
        w = np.linalg.norm([m.angular_velocity.x, m.angular_velocity.y,
                            m.angular_velocity.z])
        now = time.time()
        self._gyro.append((now, w))
        while self._gyro and now - self._gyro[0][0] > STILL_WIN_S + 0.5:
            self._gyro.popleft()

    # -- validity gates ------------------------------------------------------
    def _gyro_still(self):
        if not self._use_imu:
            return True                      # bench-only escape hatch
        now = time.time()
        recent = [(t, w) for t, w in self._gyro if now - t <= STILL_WIN_S]
        if not recent or now - self._gyro[-1][0] > IMU_STALE_S:
            return False                     # stale/absent IMU => never valid
        return max(w for _, w in recent) < GYRO_STILL_RAD_S

    def _window_stable(self):
        if len(self._window) < N_CONSEC:
            return False
        ts = [t for t, _ in self._window]
        if any(b - a > MAX_GAP_S for a, b in zip(ts, ts[1:])):
            return False
        pts = np.array([p for _, p in self._window])
        return np.linalg.norm(pts - pts.mean(axis=0),
                              axis=1).max() < SPREAD_M

    # -- main path -----------------------------------------------------------
    def _on_rgb(self, m):
        now = time.time()
        if now - self._last_run < PERIOD_S:
            return
        self._last_run = now

        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
        rgb = arr[:, :m.width * 3].copy().reshape(m.height, m.width, 3)
        res = self._model.predict(rgb[:, :, ::-1], imgsz=IMGSZ, conf=CONF,
                                  verbose=False)
        r = res[0]

        best = None                          # (conf, name, xyxy)
        for b in r.boxes:
            name = r.names[int(b.cls)]
            self._class_counts[name] += 1
            xyxy = b.xyxy[0].tolist()
            if name in self._accept:
                pass                                    # cardboard by class
            elif (self._obstacle_fallback and name == OBSTACLE_CLASS
                    and self._interior(xyxy, m)):
                pass                                    # cardboard by shape
            else:
                continue
            c = float(b.conf)
            if best is None or c > best[0]:
                best = (c, name, xyxy)

        xyz = None
        depth_note = 'no cardboard det'
        if best is not None:
            xyz, depth_note = self._back_project(best[2], m)
            if xyz is not None:
                if self._window and now - self._window[-1][0] > MAX_GAP_S:
                    self._window.clear()     # continuity broken => restart
                self._window.append((now, xyz))

        still = self._gyro_still()
        valid = xyz is not None and still and self._window_stable()
        if valid:
            pts = np.array([p for _, p in self._window])
            mean = pts.mean(axis=0)
            out = PointStamped()
            out.header.stamp = m.header.stamp
            out.header.frame_id = self._info.header.frame_id
            out.point.x, out.point.y, out.point.z = mean.tolist()
            self._pub_target.publish(out)

        if now - self._last_log > LOG_EVERY_S:
            self._last_log = now
            det = (f'{best[1]} {best[0]:.2f} xyz={xyz}' if best else 'none')
            self.get_logger().info(
                f'det={det} [{depth_note}] window={len(self._window)}/'
                f'{N_CONSEC} still={still} VALID={valid} '
                f'classes_seen={dict(self._class_counts)}')

        self._publish_annotated(r, m, xyz, valid, still)

    @staticmethod
    def _interior(xyxy, m):
        """Obstacle fallback gate: fully in view and not huge (docstring)."""
        x1, y1, x2, y2 = xyxy
        if (x1 < INTERIOR_MARGIN_PX or y1 < INTERIOR_MARGIN_PX
                or x2 > m.width - INTERIOR_MARGIN_PX
                or y2 > m.height - INTERIOR_MARGIN_PX):
            return False
        return (x2 - x1) * (y2 - y1) < MAX_AREA_FRAC * m.width * m.height

    def _back_project(self, xyxy, rgb_msg):
        """Median aligned depth over the bbox core -> optical-frame XYZ."""
        if self._depth is None or self._info is None:
            return None, 'no depth/info yet'
        dt, depth = self._depth
        if time.time() - dt > 1.0:
            return None, 'depth stale'
        x1, y1, x2, y2 = xyxy
        # aligned depth is registered to RGB but may differ in resolution
        sx = depth.shape[1] / rgb_msg.width
        sy = depth.shape[0] / rgb_msg.height
        cx_box, cy_box = (x1 + x2) / 2 * sx, (y1 + y2) / 2 * sy
        hw = (x2 - x1) / 2 * CORE_FRAC * sx
        hh = (y2 - y1) / 2 * CORE_FRAC * sy
        r0 = max(0, int(cy_box - hh)); r1 = min(depth.shape[0], int(cy_box + hh) + 1)
        c0 = max(0, int(cx_box - hw)); c1 = min(depth.shape[1], int(cx_box + hw) + 1)
        core = depth[r0:r1, c0:c1]
        good = core[np.isfinite(core) & (core > DEPTH_MIN_M)
                    & (core < DEPTH_MAX_M)]
        if good.size < 20:
            return None, f'only {good.size} valid depth px in core'
        z = float(np.median(good))
        k = self._info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        # camera_info accompanies the aligned depth => same pixel grid
        x = (cx_box - cx) * z / fx
        y = (cy_box - cy) * z / fy
        return np.array([x, y, z]), f'z_med={z:.3f}m n={good.size}'

    def _publish_annotated(self, r, m, xyz, valid, still):
        out = np.ascontiguousarray(r.plot()[:, :, ::-1])   # BGR -> RGB
        try:
            import cv2
            state = ('VALID' if valid else
                     'unstable' if still else 'ARM MOVING / no IMU')
            txt = (f'{state}  xyz='
                   + (np.array2string(xyz, precision=3) if xyz is not None
                      else '---'))
            cv2.putText(out, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if valid else (255, 80, 80), 2)
        except ImportError:
            pass
        msg = Image()
        msg.header = m.header
        msg.height, msg.width = out.shape[0], out.shape[1]
        msg.encoding = 'rgb8'
        msg.step = out.shape[1] * 3
        msg.data = out.tobytes()
        self._pub_img.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--classes', default='tray,broken_tray',
                    help='comma-separated accepted class names')
    ap.add_argument('--no-imu', action='store_true',
                    help='BENCH ONLY: skip the gyro-stillness gate')
    ap.add_argument('--strict-classes', action='store_true',
                    help='disable the interior-Obstacle cardboard fallback')
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    rclpy.init()
    node = TargetPublisher(model, set(args.classes.split(',')),
                           use_imu=not args.no_imu,
                           obstacle_fallback=not args.strict_classes)
    if args.no_imu:
        node.get_logger().warning(
            'IMU GATE DISABLED (--no-imu): bench debugging only; '
            'NEVER run the approach demo against this configuration')
    node.get_logger().info(
        f'target publisher up: {args.model} imgsz={IMGSZ} conf={CONF} '
        f'classes={sorted(set(args.classes.split(",")))}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
