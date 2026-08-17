#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Real-rig camera bring-up for the shelfbot cell (vision-rig plan, phase 3).

Single-camera cut for now: the one connected D435i serves the MAIN perception
role, i.e. the exact topic contract the Isaac twin's /World/PerceptionCamera
publishes in sim. Shelf (D435 #2) and wrist channels get added here when the
hardware lands.

Contract produced (consumers: tray_pose_node & friends in shelfbot_ros):
  /camera/rgb                 sensor_msgs/Image  rgb8   1280x720@30
  /camera/depth/image         sensor_msgs/Image  32FC1  metres, aligned to RGB
  /camera/depth/camera_info   sensor_msgs/CameraInfo aligned-depth intrinsics
  TF world -> PerceptionCamera  (the RGB *optical* frame)

The world->PerceptionCamera TF is a LOOK-AT approximation from cam_pos/cam_aim
(cell world frame, metres) good enough for the region prior + smoke tests.
Phase-4 hand-eye calibration replaces it with measured truth; until then poses
are approximately-world, not measurement-grade.

Why a mm->m converter: the RealSense publishes aligned depth as 16UC1 mm, but
the sim contract (and nvblox's input) is 32FC1 metres; depth_image_proc's
convert_metric bridges /camera/depth/image_mm -> /camera/depth/image.

Run (domain 42 + UDPv4 so the shelfbot_ros container sees it — same pattern
as the HIL bridge):

  ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    ros2 launch ~/ur10/cams/rig.launch.py \
      cam_pos:=-0.70,-0.25,2.2 cam_aim:=0.45,-1.0,0.8

Defaults below are the plan's proposed MAIN pose; override per actual mount.
"""
import numpy as np

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

D435I_SERIAL = '018322070619'   # rs-enumerate-devices -s, recorded 2026-08-17


def _look_at_quat(pos, aim):
    """
    Quaternion (x, y, z, w) of the OPTICAL frame at ``pos`` looking at ``aim``.

    Optical convention (what pose_from_depth back-projects into): +z along the
    view ray, +x image-right, +y image-down. Assumes an upright (roll-free)
    camera; a straight-down view falls back to world +x as image-right.
    """
    z = np.asarray(aim, dtype=float) - np.asarray(pos, dtype=float)
    norm = np.linalg.norm(z)
    if norm < 1e-6:
        raise ValueError('cam_aim must differ from cam_pos')
    z = z / norm
    x = np.cross(z, [0.0, 0.0, 1.0])
    xnorm = np.linalg.norm(x)
    x = np.array([1.0, 0.0, 0.0]) if xnorm < 1e-6 else x / xnorm
    y = np.cross(z, x)
    m = np.column_stack([x, y, z])          # camera -> world rotation
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        qw, qx = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        qy, qz = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        qw, qx = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        qy, qz = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        qw, qx = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        qy, qz = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        qw, qx = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        qy, qz = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return qx, qy, qz, qw


def _setup(context, *args, **kwargs):
    serial = LaunchConfiguration('serial').perform(context)
    pos = [float(v) for v in
           LaunchConfiguration('cam_pos').perform(context).split(',')]
    aim = [float(v) for v in
           LaunchConfiguration('cam_aim').perform(context).split(',')]
    qx, qy, qz, qw = _look_at_quat(pos, aim)

    params = {
        'align_depth.enable': True,
        'depth_module.depth_profile': '848x480x30',
        'rgb_camera.color_profile': '1280x720x30',
        'enable_gyro': False,
        'enable_accel': False,
        'pointcloud.enable': True,   # RViz 3D view; harmless for the contract
    }
    if serial:
        # KNOWN TRAP (2026-08-17): launch_ros re-serializes parameters
        # through YAML without quoting numeric-looking strings — the params
        # file gets `serial_no: 018322070619`, rcl types it as double, and
        # the node refuses. ParameterValue(value_type=str) does NOT survive
        # the round-trip either. Leave `serial` empty while one camera is
        # connected; for the multi-camera rig, pin serials via a
        # hand-written (properly quoted) params YAML instead of this arg.
        params['serial_no'] = ParameterValue(serial, value_type=str)

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[params],
        remappings=[
            ('/camera/camera/color/image_raw', '/camera/rgb'),
            ('/camera/camera/aligned_depth_to_color/image_raw',
             '/camera/depth/image_mm'),
            ('/camera/camera/aligned_depth_to_color/camera_info',
             '/camera/depth/camera_info'),
        ],
    )

    convert = Node(
        package='depth_image_proc',
        executable='convert_metric_node',
        name='convert_metric',
        namespace='camera',
        output='screen',
        remappings=[
            ('image_raw', '/camera/depth/image_mm'),
            ('image', '/camera/depth/image'),
        ],
    )

    rig_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rig_tf_main',
        output='screen',
        arguments=[
            '--x', str(pos[0]), '--y', str(pos[1]), '--z', str(pos[2]),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
            '--frame-id', 'world', '--child-frame-id', 'PerceptionCamera',
        ],
    )

    return [camera, convert, rig_tf]


def generate_launch_description():
    """MAIN-role D435i + mm->m converter + look-at world TF."""
    return LaunchDescription([
        DeclareLaunchArgument('serial', default_value='',
                              description='Camera serial to pin; empty = first '
                                          'device (see KNOWN TRAP in _setup).'),
        DeclareLaunchArgument('cam_pos', default_value='-0.70,-0.25,2.2',
                              description='Camera position x,y,z (cell world frame, m).'),
        DeclareLaunchArgument('cam_aim', default_value='0.45,-1.0,0.8',
                              description='Point the camera looks at (cell world frame, m).'),
        OpaqueFunction(function=_setup),
    ])
