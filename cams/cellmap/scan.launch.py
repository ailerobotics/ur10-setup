#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Cell scan from the WRIST camera — runs INSIDE the Isaac ROS container
(see run_scan.sh). The camera stays on the tool flange; the captain freedrives
the arm; the camera pose is pure kinematics:

  base_link -> tool0 (UR driver, /joint_states) -> camera_link (CALIBRATED,
  cams/cellmap/calib/*.yaml via cams/approach/wrist_rig.launch.py) ->
  camera_color_optical_frame (RealSense static TF)

so the map is built directly in base_link, no SLAM, no registration step.
(Decision 2026-08-26, replacing the handheld cuVSLAM design: FK is sub-mm and
drift-free; the only error left is the hand-eye transform, hence the
calibration gate in handeye_calibrate.py.)

  wrist D435i (host, wrist_rig.launch.py)
     /camera/depth/image_mm (16UC1, aligned to colour) --> robot_segmenter --> /cumotion/camera_1/world_depth
     /camera/rgb, camera_info                                                   `-> nvblox (2 cm, base_link)

Args:
  segmenter:=true    mask the arm out of the depth (needs live /joint_states)
  nvblox:=true
  color_image / color_info / depth_image / depth_info   wrist_rig contract by default
  cfg_dir:=/cfg      ur10_calibrated.urdf / ur10.xrdf (segmenter robot model)
  nvblox_config:=/cellmap/nvblox_cell.yaml
  container:=cellmap_container
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CONTAINER = 'cellmap_container'
WORLD_DEPTH = '/cumotion/camera_1/world_depth'      # segmenter output


def _true(ctx, name):
    return LaunchConfiguration(name).perform(ctx).strip().lower() in ('1', 'true', 'yes')


def _setup(ctx, *args, **kwargs):
    lc = lambda k: LaunchConfiguration(k).perform(ctx)  # noqa: E731
    cfg_dir = lc('cfg_dir')
    use_seg = _true(ctx, 'segmenter')
    color_image, color_info = lc('color_image'), lc('color_info')
    depth_image, depth_info = lc('depth_image'), lc('depth_info')
    composables = []

    if use_seg:
        # Masks every depth pixel within (collision sphere radius + buffer)
        # of the arm as described by OUR calibrated URDF + live joint states.
        # Without it the arm maps itself (it is always in the wrist camera's
        # near field) and every plan starts "in collision".
        composables.append(ComposableNode(
            name='robot_segmenter_1', package='isaac_ros_cumotion_robot_segmenter',
            plugin='nvidia::isaac_ros::manipulator::RobotSegmenter',
            parameters=[{
                'urdf_path': os.path.join(cfg_dir, 'ur10_calibrated.urdf'),
                'xrdf_path': os.path.join(cfg_dir, 'ur10.xrdf'),
                'robot_base_frame': 'base_link',
                'input_qos': 'SENSOR_DATA',
                'output_qos': 'SENSOR_DATA',
                'input_qos_depth': 1,
                'output_qos_depth': 1,
                # 0.10 m on top of the XRDF spheres: the wrist camera sees the
                # arm from very close, where the spheres are a coarse model
                # of the real links (and the strapped camera + cable are not
                # in the model at all). The masked halo stays "unobserved",
                # which cuMotion treats as free (bench-verified), so it costs
                # nothing.
                'additional_buffer_distance': 0.10,
                'reload_robot_description_topic': '/cumotion/reload_robot_description',
                'robot_description_service_name': '/cumotion/get_robot_description',
                'enable_performance_logging': False,
            }],
            remappings=[
                ('depth_image', depth_image),
                ('camera_info_depth', depth_info),
                ('joint_states', '/joint_states'),
                ('robot_mask', '/cumotion/camera_1/robot_mask'),
                ('robot_depth', WORLD_DEPTH),
            ],
        ))

    if _true(ctx, 'nvblox'):
        ex = get_package_share_directory('nvblox_examples_bringup')
        mb = get_package_share_directory('isaac_ros_manipulation_bringup')
        composables.append(ComposableNode(
            name='nvblox_node', package='nvblox_ros', plugin='nvblox::NvbloxNode',
            parameters=[
                os.path.join(ex, 'config/nvblox/nvblox_base.yaml'),
                os.path.join(mb, 'config/nvblox/nvblox_manipulator_base.yaml'),
                os.path.join(mb, 'config/nvblox/specializations/'
                                 'nvblox_manipulator_realsense.yaml'),
                lc('nvblox_config'),
            ],
            remappings=[
                ('/camera_0/color/image', color_image),
                ('/camera_0/color/camera_info', color_info),
                ('/camera_0/depth/camera_info', depth_info),
                ('/camera_0/depth/image', WORLD_DEPTH if use_seg else depth_image),
            ],
        ))

    if not composables:
        return []
    return [ComposableNodeContainer(
        name=lc('container'), namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=composables,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('segmenter', default_value='true'),
        DeclareLaunchArgument('nvblox', default_value='true'),
        DeclareLaunchArgument('color_image', default_value='/camera/rgb'),
        DeclareLaunchArgument('color_info', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('depth_image', default_value='/camera/depth/image_mm'),
        DeclareLaunchArgument('depth_info', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('cfg_dir', default_value='/cfg'),
        DeclareLaunchArgument('nvblox_config',
                              default_value='/cellmap/nvblox_cell.yaml'),
        DeclareLaunchArgument('container', default_value=CONTAINER),
        OpaqueFunction(function=_setup),
    ])
