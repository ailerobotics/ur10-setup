#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Multi-camera bring-up for the expanded shelfbot cell (rig v2, 2026-08-31).

The rig grew from one camera to three:
  * WRIST  — D435i on the tool flange (REPOSITIONED 2026-08-31; eye-in-hand)
  * SHELF  — D415 high, near-frontal at the gondola shelf
  * TABLE  — D415 on the table, wide view of table + arm + transit workspace

Each camera runs its OWN realsense2_camera_node in its own namespace
(/wrist, /shelf, /table) pinned to its serial via a hand-written, quoted
params YAML in cams/params/. This is mandatory here, not stylistic:

  serial_no DEGRADES TO A DOUBLE through every launch-argument path — rcl types
  the bare digits as a number and the node refuses it, and
  ParameterValue(value_type=str) does not survive launch_ros' YAML round-trip
  (cams/README.md "Traps"). A --params-file is passed straight through with no
  re-serialisation, so the quoted serial survives. With >1 RealSense on the host
  the "first device" default is a coin toss, so every camera MUST be pinned.

Topics are remapped OFF the realsense default /<ns>/camera/... onto clean
per-role names so the three cameras never collide:
  /<role>/color/image_raw          rgb8    color
  /<role>/color/camera_info
  /<role>/aligned_depth/image_mm   16UC1   aligned depth, millimetres (raw)
  /<role>/aligned_depth/image      32FC1   aligned depth, metres (convert_metric)
  /<role>/aligned_depth/camera_info
  /<role>/points                           organised coloured pointcloud
  /<role>/imu                              (wrist only, imu:=true)

The WRIST D435i is mounted UPSIDE DOWN on the flange (2026-09-03). Its raw
topics above stay exactly as the driver publishes them (calibration and
perception truth); for humans an upright twin is added by tools/image_rot180.py
(upright:=true, default) — image + matching CameraInfo + its own optical frame:
  /wrist/color_upright/image_raw           rot180  frame wrist_color_upright_optical_frame
  /wrist/color_upright/camera_info
  /wrist/aligned_depth_upright/image       rot180  (metres)
  /wrist/aligned_depth_upright/camera_info
  TF wrist_color_optical_frame -> wrist_color_upright_optical_frame = Rz(180 deg)
See cams/README.md "Wrist camera is mounted upside down" before calibrating.

Extrinsics (where each camera sits in the world / on the arm) are NOT set here —
that is Step 2 (AprilTag world-alignment for the two static D415s; hand-eye
re-calibration for the repositioned wrist D435i, whose old
cams/cellmap/calib/tool0_to_camera_2026-08-27.yaml is now STALE). Step 1 only
proves all three feeds are live; the RGB Image displays in rig_multicam.rviz
need no TF, so they show immediately.

Run (local RViz bring-up = domain 0 + UDPv4, project default per AGENTS.md; the
running UR driver already publishes the UR10 robot_description + TF on domain 0):

  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    ros2 launch cams/rig_multicam.launch.py

  # bring cameras up incrementally as they are plugged in / mounted:
  ros2 launch cams/rig_multicam.launch.py cams:=wrist
  ros2 launch cams/rig_multicam.launch.py cams:=shelf,table

Then, in another shell on domain 0:
  rviz2 -d cams/rig_multicam.rviz
"""
import pathlib
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = pathlib.Path(__file__).resolve().parent
PARAMS = HERE / 'params'
ROT180 = HERE / 'tools' / 'image_rot180.py'

# role -> (params_file, is_d435i, upside_down).
# D435i has an IMU; the D415s do not. upside_down: the camera is physically
# mounted inverted, so an upright twin of its image topics is published
# (upright:=true). Only the wrist is (flange bracket, 2026-09-03).
CAMERAS = {
    'wrist': ('wrist_d435i.yaml', True, True),
    'shelf': ('shelf_d415.yaml', False, False),
    'table': ('table_d415.yaml', False, False),
}

# Marker left in a params YAML until its real serial is filled in from
# `rs-enumerate-devices -s`. We refuse to launch such a camera with a clear
# message rather than letting the node fail cryptically on a bad serial.
_PLACEHOLDER = 'FILL_'


def _upright_nodes(role):
    """Upright (rot180) twins of the colour + aligned-depth streams, each with
    a correctly transformed CameraInfo and the upright optical frame + its
    static TF (see tools/image_rot180.py for the camera model)."""
    frame = f'{role}_color_upright_optical_frame'   # aligned depth shares it
    pairs = [
        (f'/{role}/color/image_raw', f'/{role}/color/camera_info',
         f'/{role}/color_upright/image_raw', f'/{role}/color_upright/camera_info',
         'color_upright'),
        (f'/{role}/aligned_depth/image', f'/{role}/aligned_depth/camera_info',
         f'/{role}/aligned_depth_upright/image', f'/{role}/aligned_depth_upright/camera_info',
         'aligned_depth_upright'),
    ]
    procs = []
    for img_in, info_in, img_out, info_out, name in pairs:
        procs.append(ExecuteProcess(
            cmd=[sys.executable, str(ROT180), '--ros-args',
                 '-r', f'__ns:=/{role}', '-r', f'__node:={name}',
                 '-p', f'image_in:={img_in}', '-p', f'info_in:={info_in}',
                 '-p', f'image_out:={img_out}', '-p', f'info_out:={info_out}',
                 '-p', f'frame_out:={frame}'],
            output='screen', name=f'{role}_{name}'))
    return procs


def _nodes_for(role, imu_on, color_profile, depth_profile, upright, reset):
    params_file = PARAMS / CAMERAS[role][0]
    is_d435i = CAMERAS[role][1]
    upside_down = CAMERAS[role][2]
    if not params_file.is_file():
        raise RuntimeError(f'[rig_multicam] missing params file: {params_file}')
    if _PLACEHOLDER in params_file.read_text():
        raise RuntimeError(
            f'[rig_multicam] {params_file.name} still has a placeholder serial. '
            f'Run `rs-enumerate-devices -s`, then set serial_no to the {role} '
            f"camera's real serial (quoted) before launching cams:={role}.")

    imu = imu_on and is_d435i   # only the D435i has an IMU
    stream = {
        # camera_name is the realsense TF PREFIX (not the topic namespace, which
        # comes from node namespace+name). Without a unique prefix all three
        # cameras publish the SAME frames (camera_link, camera_color_optical_
        # frame, ...) — a TF collision that scrambles RViz's TF view and makes
        # every pointcloud claim one frame. Prefix per role -> wrist_link,
        # shelf_color_optical_frame, table_depth_optical_frame, etc. Step-2
        # extrinsics then attach to distinct, unambiguous frames.
        'camera_name': role,
        'align_depth.enable': True,
        'pointcloud.enable': True,          # organised coloured cloud for RViz 3D
        'depth_module.depth_profile': depth_profile,
        'rgb_camera.color_profile': color_profile,
        # Disable the two infrared streams. The node opens them by default at
        # the depth_module's native profile (1280x720x30 Y8 on the D415) — two
        # extra streams we never use (we only need colour + aligned depth +
        # pointcloud). Left on, they ~double each camera's USB payload and, with
        # both D415s sharing one USB3 controller (bus 004 here), starve it:
        # "Incomplete video frame detected" + colour dropping to ~25 Hz
        # (measured 2026-08-31). Off, all three hold ~30 Hz.
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_gyro': imu,
        'enable_accel': imu,
        # reset:=true -> SDK hardware_reset() of THIS camera before streaming:
        # the software reset for a wedged unit (cams/README.md, "RGB modules
        # inconsistency"). The device re-enumerates on USB (a few seconds).
        'initial_reset': reset,
    }
    if imu:
        stream['unite_imu_method'] = 2      # accel+gyro united onto one /imu msg

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace=role,
        output='screen',
        # params FILE first (pins the quoted serial), then the streaming dict;
        # the node merges both. Keep the serial in the FILE — see module docstring.
        parameters=[str(params_file), stream],
        remappings=[
            (f'/{role}/camera/color/image_raw', f'/{role}/color/image_raw'),
            (f'/{role}/camera/color/camera_info', f'/{role}/color/camera_info'),
            (f'/{role}/camera/aligned_depth_to_color/image_raw',
             f'/{role}/aligned_depth/image_mm'),
            (f'/{role}/camera/aligned_depth_to_color/camera_info',
             f'/{role}/aligned_depth/camera_info'),
            (f'/{role}/camera/depth/color/points', f'/{role}/points'),
            (f'/{role}/camera/imu', f'/{role}/imu'),
        ],
    )

    # RealSense publishes aligned depth as 16UC1 millimetres; nvblox and the sim
    # contract want 32FC1 metres. convert_metric bridges mm -> m per camera.
    convert = Node(
        package='depth_image_proc',
        executable='convert_metric_node',
        name='convert_metric',
        namespace=role,
        output='screen',
        remappings=[
            ('image_raw', f'/{role}/aligned_depth/image_mm'),
            ('image', f'/{role}/aligned_depth/image'),
        ],
    )
    nodes = [camera, convert]
    if upright and upside_down:
        nodes += _upright_nodes(role)
    return nodes


def _setup(context, *args, **kwargs):
    requested = [c.strip() for c in
                 LaunchConfiguration('cams').perform(context).split(',')
                 if c.strip()]
    unknown = [c for c in requested if c not in CAMERAS]
    if unknown:
        raise RuntimeError(f'[rig_multicam] unknown camera role(s) {unknown}; '
                           f'known: {sorted(CAMERAS)}')
    imu_on = LaunchConfiguration('imu').perform(context).lower() == 'true'
    color_profile = LaunchConfiguration('color_profile').perform(context)
    depth_profile = LaunchConfiguration('depth_profile').perform(context)
    upright = LaunchConfiguration('upright').perform(context).lower() == 'true'
    reset = LaunchConfiguration('reset').perform(context).lower() == 'true'

    nodes = []
    for role in requested:
        nodes += _nodes_for(role, imu_on, color_profile, depth_profile, upright, reset)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cams', default_value='wrist,shelf,table',
            description='Comma list of camera roles to bring up: any of '
                        'wrist,shelf,table. Add cameras incrementally as they '
                        'are plugged in.'),
        DeclareLaunchArgument(
            'color_profile', default_value='1280x720x30',
            description='RGB profile WxHxFPS for every camera. Drop to '
                        '848x480x15 if three cameras on one USB tree drop '
                        'frames ("No frames received").'),
        DeclareLaunchArgument(
            'depth_profile', default_value='848x480x30',
            description='Depth profile WxHxFPS for every camera.'),
        DeclareLaunchArgument(
            'upright', default_value='true',
            description='Publish the rot180 upright twin (/<role>/color_upright, '
                        '/<role>/aligned_depth_upright + TF) for cameras that '
                        'are mounted upside down (the WRIST). Raw topics are '
                        'never altered.'),
        DeclareLaunchArgument(
            'reset', default_value='false',
            description='SDK hardware_reset() of each listed camera before '
                        'streaming (initial_reset). Software reset for a unit '
                        'whose colour module failed to enumerate; combine with '
                        'cams:=<role> to reset only that camera.'),
        DeclareLaunchArgument(
            'imu', default_value='false',
            description='Enable accel+gyro on the WRIST D435i (united on '
                        '/wrist/imu). D415s have no IMU. Off for plain feeds.'),
        OpaqueFunction(function=_setup),
    ])
