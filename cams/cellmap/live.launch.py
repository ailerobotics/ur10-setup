"""
LIVE collision avoidance: saved static map as the base world, live wrist-camera
depth integrated on top, one ESDF feeding cuMotion.

  segmenter (masks the MOVING arm) -> nvblox -> ESDF -> cuMotion

Why one nvblox and not two: cuMotion consumes exactly one ESDF service. To
avoid BOTH the pre-scanned geometry AND anything new, the same nvblox has to
hold both — so it starts with depth integration live, the saved .nvblx is
loaded over the top a moment later (load REPLACES the layers, so the handful
of frames integrated before it are discarded, which is why loading must
happen at startup and not mid-run), and everything the camera sees from then
on accumulates on that base.

Ordering is the same gate as planner.launch.py: cuMotion starts ONLY after
load_map exits 0, because read_esdf_world:=true against a missing service
blocks forever and against an empty world refuses every request.

DECAY IS OFF, deliberately. The manipulator defaults decay TSDF weights so
moved objects disappear; here that would also erase the static base wherever
the camera happens to look, replacing scanned geometry with whatever the
current view contains. Off means the world only ever GAINS obstacles — stale
ones linger, which is the conservative failure direction for collision
avoidance. Re-scan to clear them.

  cams/cellmap/run_live.sh                       # defaults below
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

WORLD_DEPTH = '/cumotion/camera_1/world_depth'


def _setup(ctx, *args, **kwargs):
    lc = lambda k: LaunchConfiguration(k).perform(ctx)  # noqa: E731
    cfg_dir = lc('cfg_dir')
    here = os.path.dirname(os.path.abspath(__file__))
    map_path = lc('map')
    if not map_path:
        raise RuntimeError('map:=<path.nvblx> is required (the static base world)')
    ex = get_package_share_directory('nvblox_examples_bringup')
    mb = get_package_share_directory('isaac_ros_manipulation_bringup')
    color_image, color_info = lc('color_image'), lc('color_info')
    depth_image, depth_info = lc('depth_image'), lc('depth_info')

    segmenter = ComposableNode(
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
            # The arm is ALWAYS in the wrist camera's near field and it is now
            # MOVING, so this mask is what keeps the arm from mapping itself
            # into the world it is trying to avoid. 0.10 m on top of the XRDF
            # spheres covers the coarse sphere model plus the strapped camera
            # and cable, which are not in the model at all.
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
    )

    nvblox = ComposableNode(
        name='nvblox_node', package='nvblox_ros', plugin='nvblox::NvbloxNode',
        parameters=[
            os.path.join(ex, 'config/nvblox/nvblox_base.yaml'),
            os.path.join(mb, 'config/nvblox/nvblox_manipulator_base.yaml'),
            os.path.join(mb, 'config/nvblox/specializations/'
                             'nvblox_manipulator_realsense.yaml'),
            lc('nvblox_config'),
            {
                'use_depth': True,
                'integrate_depth_rate_hz': float(lc('integrate_hz')),
                'update_mesh_rate_hz': 2.0,      # captain watches it fill in
            },
        ],
        remappings=[
            ('/camera_0/color/image', color_image),
            ('/camera_0/color/camera_info', color_info),
            ('/camera_0/depth/camera_info', depth_info),
            ('/camera_0/depth/image', WORLD_DEPTH),
        ],
    )

    container = ComposableNodeContainer(
        name='cellmap_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[segmenter, nvblox],
    )

    loader = ExecuteProcess(
        cmd=['python3', os.path.join(here, 'tools', 'map_ctl.py'), 'load', map_path,
             '--wait', '90'],
        name='load_static_base', output='screen',
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
            'cumotion_action_server.add_ground_plane': lc('ground_plane'),
            'cumotion_action_server.publish_cumotion_world_as_voxels': lc('voxels'),
            'cumotion_action_server.publish_voxel_size': '0.02',
            'cumotion_action_server.max_publish_voxels': '500000',
            'cumotion_action_server.time_dilation_factor': lc('time_dilation'),
        }.items(),
    )

    def after_load(event, context):
        if event.returncode == 0:
            return [LogInfo(msg=f'static base loaded: {map_path}; live depth integrating '
                                f'on top at {lc("integrate_hz")} Hz -> starting cuMotion'),
                    cumotion]
        return [LogInfo(msg=f'!!! load of the static base FAILED (rc {event.returncode}) — '
                            'NOT starting cuMotion. It would plan against live depth ONLY, '
                            'i.e. with no memory of anything the camera is not looking at.'),
                EmitEvent(event=Shutdown(reason='static base load failed'))]

    return [container, loader,
            RegisterEventHandler(OnProcessExit(target_action=loader, on_exit=after_load))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('cfg_dir', default_value='/cfg'),
        DeclareLaunchArgument('nvblox_config', default_value='/cellmap/nvblox_cell.yaml'),
        DeclareLaunchArgument('urdf', default_value='/cfg/ur10_pneumatic.urdf'),
        DeclareLaunchArgument('xrdf', default_value='/cfg/ur10_pneumatic.xrdf'),
        DeclareLaunchArgument('ground_plane', default_value='true'),
        DeclareLaunchArgument('voxels', default_value='true'),
        DeclareLaunchArgument('time_dilation', default_value='0.05'),
        DeclareLaunchArgument('integrate_hz', default_value='10.0'),
        DeclareLaunchArgument('color_image', default_value='/camera/rgb'),
        DeclareLaunchArgument('color_info', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('depth_image', default_value='/camera/depth/image_mm'),
        DeclareLaunchArgument('depth_info', default_value='/camera/depth/camera_info'),
        OpaqueFunction(function=_setup),
    ])
