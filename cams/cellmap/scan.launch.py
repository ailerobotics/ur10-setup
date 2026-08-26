#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Handheld cell scan — runs INSIDE the Isaac ROS container (see run_scan.sh).

Pipeline (all switchable, so step 1 "tracking proof" runs with just the first
two):

  D435i (RGBD profile) --> cuVSLAM RGBD (tracking_mode 2)  TF map->odom->camera_link
                       `-> robot_segmenter (arm masked)    /cumotion/camera_1/world_depth
                                                  `-> nvblox (2 cm, base_link)

Why RGBD mode and not NVIDIA's stereo-IR+IMU demo: stereo mode needs the
projector flashing on/off through the `realsense_splitter` node (not in this
image) and per-frame emitter metadata the camera's 5.12.8.200 firmware may not
emit (Isaac 4.x asks for 5.16.0.1). RGBD mode tracks on the colour image and
takes aligned depth with the projector simply ON, i.e. none of that. Cost: no
IMU fusion and a rolling-shutter colour sensor -> scan at walking pace, no
fast turns (documented limit; stereo+IMU stays the fallback).

Frame chain the whole thing hinges on (all one TF tree):
  world -> base_link -> map -> odom -> camera_link -> camera_color_optical_frame
          (robot)     (static: tools/tag_anchor.py)  (cuVSLAM)   (RealSense static)
nvblox and the segmenter look up base_link -> camera_color_optical_frame through
that chain, which is why the anchor MUST be published before nvblox is
started (integration into a wrong frame cannot be undone; restart the scan).

Args:
  camera:=true      RealSense driver in the RGBD profile (in-container, USB
                    passed through by run_scan.sh). false = driver elsewhere.
  vslam:=true       cuVSLAM RGBD.
  segmenter:=false  robot segmenter; needs live /joint_states + the anchor TF.
  nvblox:=false     nvblox mapper; needs the anchor TF (see above).
  fps:=30           colour+depth rate (D435i: 30 or 60 at 640x360).
  cfg_dir:=/cfg     where ur10_calibrated.urdf / ur10.xrdf are mounted.
  nvblox_config:=/cellmap/nvblox_cell.yaml
  container:=cellmap_container   component container name; use a different one
                    (e.g. cellmap_mapper) when adding segmenter+nvblox to an
                    already running camera+cuVSLAM instance via docker exec.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CONTAINER = 'cellmap_container'

# RealSense topic/frame contract used by every consumer below and by the host
# tools (tools/tag_anchor.py, tools/anchor_check.py). Node name 'camera',
# namespace '' -> /camera/<stream>/... and frames camera_link,
# camera_color_optical_frame (same layout NVIDIA's RGBD launch assumes).
COLOR_IMAGE = '/camera/color/image_raw'
COLOR_INFO = '/camera/color/camera_info'
DEPTH_IMAGE = '/camera/aligned_depth_to_color/image_raw'
DEPTH_INFO = '/camera/aligned_depth_to_color/camera_info'
WORLD_DEPTH = '/cumotion/camera_1/world_depth'      # segmenter output


def _true(ctx, name):
    return LaunchConfiguration(name).perform(ctx).strip().lower() in ('1', 'true', 'yes')


def _setup(ctx, *args, **kwargs):
    fps = int(LaunchConfiguration('fps').perform(ctx))
    cfg_dir = LaunchConfiguration('cfg_dir').perform(ctx)
    nvblox_cfg = LaunchConfiguration('nvblox_config').perform(ctx)
    use_seg = _true(ctx, 'segmenter')
    container = LaunchConfiguration('container').perform(ctx)
    actions = []

    if _true(ctx, 'camera'):
        actions.append(Node(
            name='camera', namespace='',
            package='realsense2_camera', executable='realsense2_camera_node',
            output='screen',
            parameters=[{
                # serial NOT pinned on purpose: one camera on the bus, and the
                # launch-arg path types '018322070619' as a double (cams/README.md
                # "Traps"). Pin via a quoted params YAML if a 2nd camera appears.
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,          # depth in the colour frame
                'enable_sync': True,                 # colour+depth pairs share a stamp
                'depth_module.emitter_enabled': 1,   # projector ON: depth quality first
                'depth_module.depth_profile': f'640,360,{fps}',
                'rgb_camera.color_profile': f'640,360,{fps}',
                'depth_module.enable_auto_exposure': True,
                'rgb_camera.enable_auto_exposure': True,
                'enable_gyro': False,                # RGBD mode fuses no IMU
                'enable_accel': False,
                # Post-filters add latency and smear edges into the map — off.
                'decimation_filter.enable': False,
                'spatial_filter.enable': False,
                'temporal_filter.enable': False,
                'hole_filling_filter.enable': False,
            }],
        ))

    composables = []

    if _true(ctx, 'vslam'):
        composables.append(ComposableNode(
            name='visual_slam_node', package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            parameters=[{
                'tracking_mode': 2,                  # RGBD
                'depth_scale_factor': 1000.0,        # 16UC1 mm
                'enable_image_denoising': False,
                'rectified_images': False,
                'image_jitter_threshold_ms': 20.0,
                'sync_matching_threshold_ms': 10.0,
                'num_cameras': 1,
                'min_num_images': 1,
                'depth_camera_id': 0,
                'camera_optical_frames': ['camera_color_optical_frame'],
                'base_frame': 'camera_link',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'publish_map_to_odom_tf': True,
                'publish_odom_to_base_tf': True,
                # Loop closure ON: the captain walks a loop and ends at the
                # tag; localisation-and-mapping is what pulls the end of the
                # loop back onto its start instead of leaving pure-odometry
                # drift in the map.
                'enable_localization_n_mapping': True,
                'enable_ground_constraint_in_odometry': False,   # handheld, 6-DoF
                'enable_ground_constraint_in_slam': False,
                'enable_slam_visualization': True,
                'enable_landmarks_view': True,
                'enable_observations_view': True,
                'path_max_size': 4096,               # whole walk for loop_check.py
                'verbosity': 0,
            }],
            remappings=[
                ('visual_slam/image_0', COLOR_IMAGE),
                ('visual_slam/camera_info_0', COLOR_INFO),
                ('visual_slam/depth_0', DEPTH_IMAGE),
            ],
        ))

    if use_seg:
        # Masks every depth pixel within (collision sphere radius + buffer)
        # of the arm as described by OUR calibrated URDF + live joint states.
        # Without it the parked arm maps itself and the first plan from the
        # park pose is "start state in collision".
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
                # 0.10 m on top of the XRDF spheres: the handheld camera pose
                # is a SLAM estimate (cm-level), a wider mask hides that
                # error; the cost is a 10 cm halo of unmapped space around
                # the arm, which is free space anyway once the arm moves.
                'additional_buffer_distance': 0.10,
                'reload_robot_description_topic': '/cumotion/reload_robot_description',
                'robot_description_service_name': '/cumotion/get_robot_description',
                'enable_performance_logging': False,
            }],
            remappings=[
                ('depth_image', DEPTH_IMAGE),
                ('camera_info_depth', DEPTH_INFO),
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
                nvblox_cfg,
            ],
            remappings=[
                ('/camera_0/color/image', COLOR_IMAGE),
                ('/camera_0/color/camera_info', COLOR_INFO),
                ('/camera_0/depth/camera_info', DEPTH_INFO),
                ('/camera_0/depth/image', WORLD_DEPTH if use_seg else DEPTH_IMAGE),
            ],
        ))

    if composables:
        actions.append(ComposableNodeContainer(
            name=container, namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen',
            composable_node_descriptions=composables,
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('vslam', default_value='true'),
        DeclareLaunchArgument('segmenter', default_value='false'),
        DeclareLaunchArgument('nvblox', default_value='false'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('cfg_dir', default_value='/cfg'),
        DeclareLaunchArgument('nvblox_config',
                              default_value='/cellmap/nvblox_cell.yaml'),
        DeclareLaunchArgument('container', default_value=CONTAINER),
        OpaqueFunction(function=_setup),
    ])
