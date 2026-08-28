#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Eye-in-hand (wrist) bring-up of the tool-flange D435i for the approach demo.

Differences from ../rig.launch.py (the static table/pole rig):
  * Runs on the ARM's domain:  ROS_DOMAIN_ID=0  FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  * IMU ON by default (accel + gyro, united into /camera/imu) — used for the
    gravity-vector rotation sanity check and the gyro-stillness gate in
    target_publisher.py. `imu:=false` turns it back off.
  * Publishes the static  tool0 -> camera_link  mount guess (see below); the
    RealSense wrapper's own calibrated static TF tree then provides
    camera_link -> camera_color_optical_frame etc., so image frame_ids
    resolve against the arm's base via the robot's own TF chain.
  * The table rig's world->PerceptionCamera look-at TF is MEANINGLESS here
    (the camera rides the wrist; a fixed world pose would fight the arm TF),
    so it is NOT published unless you explicitly ask with lookat_tf:=true
    (only useful when the camera is physically off the arm again).

Mount guess (rough by design — the demo stops >= 0.25 m short of the target):
  The D435i is strapped to the tool flange with its optical axis along the
  tool's forward axis (tool0 +Z, out of the flange) and its "up" side toward
  tool0 +X (which points up in the parked shelf-facing pose). In camera_link
  body convention (x fwd, y left, z up) that is:
      x_cl = tool0 +Z,   y_cl = tool0 -Y,   z_cl = tool0 +X
  `cam_roll_deg` spins the camera about its optical axis on top of this base
  mapping. Default 28.6: the gravity check (tools/imu_gravity_check.py, run
  2026-08-18, arm parked) showed the camera strapped level with the WORLD
  while tool0 +X is 28.7 deg off vertical in the park pose — i.e. the camera
  is really rolled ~+28.6 deg w.r.t. tool0. Residual after refinement:
  4.2 deg (README). `cam_xyz` is the lens position in tool0 (default: 5 cm
  toward tool0 +X — the strap offset — and 1.5 cm forward; translation is a
  pure guess, unverifiable from gravity).

Calibrated mount (2026-08-26, cell map): `calib:=auto` (default) loads the
newest cams/cellmap/calib/tool0_to_camera_*.yaml (hand-eye calibration,
cams/cellmap/handeye_calibrate.py) and publishes THAT tool0 -> camera_link;
the guess above is used only when no calibration exists, or explicitly with
`calib:=guess`. Why the default changed: a map built on the ±5 cm / 4 deg
guess is off by several cm at 1 m — more than the planner's collision
buffer — while the approach demo's 0.25 m standoff never cared.
`calib:=/path/to.yaml` pins a specific file.

Run:
  ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    ros2 launch ~/ur10/cams/approach/wrist_rig.launch.py
"""
import glob
import math
import pathlib

import numpy as np

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

D435I_SERIAL = '018322070619'   # tool-flange unit; single camera => not pinned
                                # (serial_no YAML trap, see ../README.md)


def _mat_to_quat(m):
    """Rotation matrix (3x3, columns = child axes in parent) -> (qx,qy,qz,qw)."""
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


def mount_guess_quat(roll_deg):
    """tool0 -> camera_link quaternion for the documented mount guess."""
    x_cl = np.array([0.0, 0.0, 1.0])    # camera forward = tool0 +Z
    y_cl = np.array([0.0, -1.0, 0.0])   # camera left    = tool0 -Y
    z_cl = np.array([1.0, 0.0, 0.0])    # camera up      = tool0 +X
    base = np.column_stack([x_cl, y_cl, z_cl])
    a = math.radians(roll_deg)
    # roll about the camera's own forward (x_cl) axis
    roll = np.array([[1, 0, 0],
                     [0, math.cos(a), -math.sin(a)],
                     [0, math.sin(a), math.cos(a)]])
    return _mat_to_quat(base @ roll)


def _setup(context, *args, **kwargs):
    imu = LaunchConfiguration('imu').perform(context).lower() == 'true'
    lookat = LaunchConfiguration('lookat_tf').perform(context).lower() == 'true'
    xyz = [float(v) for v in
           LaunchConfiguration('cam_xyz').perform(context).split(',')]
    roll = float(LaunchConfiguration('cam_roll_deg').perform(context))
    qx, qy, qz, qw = mount_guess_quat(roll)
    calib = LaunchConfiguration('calib').perform(context).strip()
    calib_file = None
    if calib == 'auto':
        found = sorted(glob.glob(str(pathlib.Path(__file__).resolve().parent.parent
                                     / 'cellmap' / 'calib' / 'tool0_to_camera_*.yaml')))
        calib_file = found[-1] if found else None
    elif calib not in ('', 'guess'):
        calib_file = calib
    if calib_file:
        import yaml
        d = yaml.safe_load(open(calib_file))['tool0_to_camera_link']
        assert d['parent'] == 'tool0' and d['child'] == 'camera_link', calib_file
        xyz = [float(v) for v in d['xyz']]
        qx, qy, qz, qw = (float(v) for v in d['quaternion_xyzw'])
        print(f'[wrist_rig] tool0->camera_link from CALIBRATION {calib_file}: '
              f'xyz {xyz} rpy {d.get("rpy_deg")}')
    else:
        print('[wrist_rig] tool0->camera_link = mount GUESS (no calibration file'
              + (' found)' if calib == 'auto' else ', calib:=guess)'))

    params = {
        'align_depth.enable': True,
        'depth_module.depth_profile': '848x480x30',
        'rgb_camera.color_profile': '1280x720x30',
        'enable_gyro': imu,
        'enable_accel': imu,
        'pointcloud.enable': True,   # RViz 3D sanity view; harmless
    }
    if imu:
        params['unite_imu_method'] = 2   # linear_interpolation -> one Imu msg
                                         # with both accel + gyro on /camera/imu

    # camera_params: OPT-IN path to a hand-written params YAML, used to pin
    # serial_no (see cams/cellmap/d435i_wrist_serial.yaml for why a file and
    # not a launch arg). Default '' = unchanged first-device behaviour, so the
    # approach demo keeps working exactly as before.
    cam_params = LaunchConfiguration('camera_params').perform(context).strip()
    param_list = ([cam_params] if cam_params else []) + [params]
    if cam_params:
        print(f'[wrist_rig] extra camera params file: {cam_params}')

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=param_list,
        remappings=[
            ('/camera/camera/color/image_raw', '/camera/rgb'),
            ('/camera/camera/aligned_depth_to_color/image_raw',
             '/camera/depth/image_mm'),
            ('/camera/camera/aligned_depth_to_color/camera_info',
             '/camera/depth/camera_info'),
            ('/camera/camera/imu', '/camera/imu'),
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

    mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wrist_cam_tf',
        output='screen',
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
            '--frame-id', 'tool0', '--child-frame-id', 'camera_link',
        ],
    )

    nodes = [camera, convert, mount_tf]

    if lookat:
        # Off-arm fallback only. In eye-in-hand mode a fixed world pose for
        # the camera is wrong by construction — never enable while the camera
        # rides the wrist.
        pos = [float(v) for v in
               LaunchConfiguration('cam_pos').perform(context).split(',')]
        aim = [float(v) for v in
               LaunchConfiguration('cam_aim').perform(context).split(',')]
        import importlib.util   # NB: do NOT re-import pathlib here — a
                                # function-local import would shadow the
                                # module-level one for the WHOLE function and
                                # break the calib lookup above (UnboundLocalError)
        spec = importlib.util.spec_from_file_location(
            'table_rig', pathlib.Path(__file__).resolve().parent.parent
            / 'rig.launch.py')
        table_rig = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(table_rig)
        lqx, lqy, lqz, lqw = table_rig._look_at_quat(pos, aim)
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rig_tf_main',
            output='screen',
            arguments=[
                '--x', str(pos[0]), '--y', str(pos[1]), '--z', str(pos[2]),
                '--qx', str(lqx), '--qy', str(lqy), '--qz', str(lqz),
                '--qw', str(lqw),
                '--frame-id', 'world', '--child-frame-id', 'PerceptionCamera',
            ],
        ))

    return nodes


def generate_launch_description():
    """Wrist D435i + mm->m converter + tool0->camera_link mount guess."""
    return LaunchDescription([
        DeclareLaunchArgument('imu', default_value='true',
                              description='Enable accel+gyro, united on /camera/imu.'),
        DeclareLaunchArgument('cam_xyz', default_value='0.05,0.0,0.015',
                              description='Camera position in tool0 (m): x,y,z. '
                                          'Rough mount guess; see module docstring.'),
        DeclareLaunchArgument('cam_roll_deg', default_value='28.6',
                              description='Roll about the optical axis on top of '
                                          'the base mount mapping (deg). Default '
                                          'fitted to the IMU gravity vector, '
                                          '2026-08-18 (4.2 deg residual).'),
        DeclareLaunchArgument('calib', default_value='auto',
                              description='auto = newest cams/cellmap/calib/'
                                          'tool0_to_camera_*.yaml; guess = the mount '
                                          'guess above; or a yaml path.'),
        DeclareLaunchArgument('camera_params', default_value='',
                              description='Path to an extra realsense2_camera '
                                          'params YAML, e.g. cams/cellmap/'
                                          'd435i_wrist_serial.yaml to pin the '
                                          'wrist D435i by serial when a second '
                                          'RealSense is connected. Empty '
                                          '(default) = first device, unchanged.'),
        DeclareLaunchArgument('lookat_tf', default_value='false',
                              description='Publish the table-rig world->PerceptionCamera '
                                          'look-at TF. MUST stay false in eye-in-hand '
                                          'mode (camera on the wrist).'),
        DeclareLaunchArgument('cam_pos', default_value='-0.70,-0.25,2.2',
                              description='Only with lookat_tf:=true.'),
        DeclareLaunchArgument('cam_aim', default_value='0.45,-1.0,0.8',
                              description='Only with lookat_tf:=true.'),
        OpaqueFunction(function=_setup),
    ])
