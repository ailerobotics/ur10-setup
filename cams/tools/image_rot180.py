#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Republish one camera stream rotated 180 degrees, as a CONSISTENT camera model
(image + CameraInfo + optical frame), not a display hack.

Why (2026-09-03): the wrist D435i is mounted UPSIDE DOWN on the tool flange, so
its raw colour image is inverted for humans. realsense2_camera has no rotation
option, so rig_multicam.launch.py runs this node next to the wrist driver and
publishes an upright twin of the colour (and aligned-depth) stream:

    /wrist/color/image_raw            raw   frame wrist_color_optical_frame
    /wrist/color_upright/image_raw    rot180, frame wrist_color_upright_optical_frame
    (+ camera_info beside each, + aligned_depth / aligned_depth_upright)

RAW stays the calibration / perception truth (cams/README.md, "Wrist camera is
mounted upside down"). The upright twin exists for eyes: the montage, RViz.

Camera model of a 180 degree rotation, pixel-centre convention (ROS/OpenCV: the
centre of pixel (0,0) is at (0,0)):

    u' = (W - 1) - u        v' = (H - 1) - v
    fx, fy   unchanged
    cx'      = (W - 1) - cx          cy' = (H - 1) - cy
    plumb_bob / rational_polynomial: radial k1,k2,k3(,k4..k6) unchanged (they
    are even in r); tangential p1, p2 NEGATED (odd under (x,y) -> (-x,-y)).
    P[0,2], P[1,2] get the same cx/cy update; R stays identity; roi/binning
    are copied through (RealSense publishes zeros).

The upright image's optical frame is the raw optical frame rotated 180 degrees
about its own z (optical) axis: frame_out = frame_in * Rz(pi), quaternion
(x,y,z,w) = (0,0,1,0). This node broadcasts that static TF itself (parent =
the frame_id seen on the first CameraInfo) so the twin is self-contained.

Run standalone (domain 0), e.g. for the wrist colour stream:
  python3 cams/tools/image_rot180.py --ros-args \
    -p image_in:=/wrist/color/image_raw -p info_in:=/wrist/color/camera_info \
    -p image_out:=/wrist/color_upright/image_raw \
    -p info_out:=/wrist/color_upright/camera_info \
    -p frame_out:=wrist_color_upright_optical_frame
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


def rotate_camera_info(msg: CameraInfo, frame_out: str) -> CameraInfo:
    """CameraInfo of the same camera after a 180 deg image rotation."""
    out = CameraInfo()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = frame_out
    out.height, out.width = msg.height, msg.width
    out.distortion_model = msg.distortion_model
    w1, h1 = float(msg.width - 1), float(msg.height - 1)

    k = list(msg.k)
    k[2] = w1 - k[2]                    # cx
    k[5] = h1 - k[5]                    # cy
    out.k = k

    p = list(msg.p)
    p[2] = w1 - p[2]                    # cx (projection)
    p[6] = h1 - p[6]                    # cy (projection)
    out.p = p

    d = list(msg.d)
    if len(d) >= 4:                     # [k1 k2 p1 p2 k3 ...]: negate p1, p2
        d[2] = -d[2]
        d[3] = -d[3]
    out.d = d

    out.r = list(msg.r)                 # identity for a mono camera
    out.binning_x, out.binning_y = msg.binning_x, msg.binning_y
    out.roi = msg.roi
    return out


class Rot180(Node):
    def __init__(self):
        super().__init__('image_rot180')
        p = self.declare_parameters('', [
            ('image_in', ''), ('info_in', ''),
            ('image_out', ''), ('info_out', ''),
            ('frame_out', ''),
        ])
        self.image_in, self.info_in, self.image_out, self.info_out, self.frame_out = \
            [x.value for x in p]
        if not all([self.image_in, self.info_in, self.image_out,
                    self.info_out, self.frame_out]):
            raise SystemExit('[image_rot180] all of image_in, info_in, image_out, '
                             'info_out, frame_out must be set')
        self.bridge = CvBridge()
        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_sent_for = None
        self.n = 0

        # Images: sensor-data QoS both ways (what RViz / the montage expect).
        # CameraInfo: reliable out (compatible with every subscriber policy).
        info_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_img = self.create_publisher(Image, self.image_out, qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, self.info_out, info_qos)
        self.create_subscription(Image, self.image_in, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_in, self._on_info, qos_profile_sensor_data)
        self.get_logger().info(
            f'{self.image_in} -> {self.image_out}  (rot180; frame {self.frame_out})')

    def _publish_static_tf(self, parent: str):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = self.frame_out
        t.transform.rotation.z = 1.0        # Rz(pi): (x,y,z,w) = (0,0,1,0)
        t.transform.rotation.w = 0.0
        self.tf_static.sendTransform(t)
        self.tf_sent_for = parent
        self.get_logger().info(f'static TF {parent} -> {self.frame_out} = Rz(180 deg)')

    def _on_info(self, msg: CameraInfo):
        if self.tf_sent_for != msg.header.frame_id:
            self._publish_static_tf(msg.header.frame_id)
        self.pub_info.publish(rotate_camera_info(msg, self.frame_out))

    def _on_image(self, msg: Image):
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        out = self.bridge.cv2_to_imgmsg(cv2.rotate(cv, cv2.ROTATE_180), encoding=msg.encoding)
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_out
        self.pub_img.publish(out)
        self.n += 1


def main():
    rclpy.init()
    node = Rot180()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
