#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
cuMotion WITH the cell map — runs INSIDE the Isaac container
(cumotion/start_cumotion_map.sh). Strict order, enforced by launch events:

  1. nvblox_node (manipulator config, NO camera inputs, global_frame base_link)
  2. tools/map_ctl.py load <map.nvblx>   -- waits for the service, loads, exits
  3. only if (2) exited 0: isaac_ros_cumotion with read_esdf_world:=true
     otherwise: log the failure and shut the launch down (exit non-zero).

Why the ordering is hard-wired and not "start everything and hope": cuMotion
with read_esdf_world:=true blocks forever waiting for
/nvblox_node/get_esdf_and_gradient (start_cumotion.sh's own warning), and an
nvblox that is up but EMPTY answers happily — the planner would then run with
no obstacles while everyone believes the map is on. Gating on the loader's
exit code makes "map not loaded" a loud failure instead of a silent one.

Args:
  map:=/maps/cell-YYYY-MM-DD.nvblx   (required)
  xrdf:=/cfg/ur10_map.xrdf           0.04 m world buffer (ur10.xrdf = 0.02)
  urdf:=/cfg/ur10_calibrated.urdf
  voxels:=true                       publish_cumotion_world_as_voxels ->
                                     /curobo/voxels (Marker) so host RViz shows
                                     exactly what the planner sees. Costs a
                                     little GPU; turn off once validated.
  time_dilation:=0.5                 same as start_cumotion.sh
  nvblox_config:=/cellmap/nvblox_cell.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _setup(ctx, *args, **kwargs):
    lc = lambda k: LaunchConfiguration(k).perform(ctx)  # noqa: E731
    map_path = lc('map')
    if not map_path:
        raise RuntimeError('map:=<path.nvblx> is required')
    ex = get_package_share_directory('nvblox_examples_bringup')
    mb = get_package_share_directory('isaac_ros_manipulation_bringup')
    here = os.path.dirname(os.path.abspath(__file__))

    nvblox = ComposableNode(
        name='nvblox_node', package='nvblox_ros', plugin='nvblox::NvbloxNode',
        parameters=[
            os.path.join(ex, 'config/nvblox/nvblox_base.yaml'),
            os.path.join(mb, 'config/nvblox/nvblox_manipulator_base.yaml'),
            os.path.join(mb, 'config/nvblox/specializations/nvblox_manipulator_realsense.yaml'),
            lc('nvblox_config'),
            {
                # Serve the SAVED map only: no depth/colour subscriptions, so a
                # stray camera topic can never integrate into the served map.
                'use_depth': False,
                'use_color': False,
                'integrate_depth_rate_hz': 0.0,
                'integrate_color_rate_hz': 0.0,
                # 1 Hz mesh so run_rviz.sh can show the loaded map for the
                # captain's "is this my cell?" check; nothing changes, so cheap.
                'update_mesh_rate_hz': 1.0,
            },
        ],
    )
    container = ComposableNodeContainer(
        name='cellmap_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[nvblox],
    )

    loader = ExecuteProcess(
        cmd=['python3', os.path.join(here, 'tools', 'map_ctl.py'), 'load', map_path,
             '--wait', '90'],
        name='load_map', output='screen',
    )

    cumotion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('isaac_ros_cumotion'),
            'launch', 'isaac_ros_cumotion.launch.py')),
        launch_arguments={
            'cumotion_action_server.urdf_file_path': lc('urdf'),
            'cumotion_action_server.xrdf_file_path': lc('xrdf'),
            'cumotion_action_server.read_esdf_world': 'true',
            'cumotion_action_server.update_esdf_on_request': 'true',
            'cumotion_action_server.esdf_service_name': '/nvblox_node/get_esdf_and_gradient',
            'cumotion_action_server.add_ground_plane': 'true',    # keep the free guard
            'cumotion_action_server.publish_cumotion_world_as_voxels': lc('voxels'),
            'cumotion_action_server.publish_voxel_size': '0.02',
            'cumotion_action_server.max_publish_voxels': '500000',
            'cumotion_action_server.time_dilation_factor': lc('time_dilation'),
        }.items(),
    )

    def after_load(event, context):
        if event.returncode == 0:
            return [LogInfo(msg=f'cell map loaded: {map_path} -> starting cuMotion '
                                '(read_esdf_world=true, xrdf ' + lc('xrdf') + ')'),
                    cumotion]
        return [LogInfo(msg=f'!!! load_map FAILED (rc {event.returncode}) for {map_path} — '
                            'NOT starting cuMotion (it would block on the ESDF service '
                            'or plan against an empty world). Check the path / nvblox log.'),
                EmitEvent(event=Shutdown(reason='cell map load failed'))]

    return [container, loader,
            RegisterEventHandler(OnProcessExit(target_action=loader, on_exit=after_load))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('xrdf', default_value='/cfg/ur10_map.xrdf'),
        DeclareLaunchArgument('urdf', default_value='/cfg/ur10_calibrated.urdf'),
        DeclareLaunchArgument('voxels', default_value='true'),
        DeclareLaunchArgument('time_dilation', default_value='0.5'),
        DeclareLaunchArgument('nvblox_config', default_value='/cellmap/nvblox_cell.yaml'),
        OpaqueFunction(function=_setup),
    ])
