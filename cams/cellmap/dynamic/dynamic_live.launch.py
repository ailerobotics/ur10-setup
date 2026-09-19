#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""nvblox DYNAMIC layer feeding cuMotion — runs INSIDE the task container.

Unlike cams/cellmap/live.launch.py (a saved snapshot + live wrist depth, decay
OFF), this runs nvblox's `dynamic` multi-mapper (mapping_type overridable) fed by
a depth topic — synthetic (synth_depth_pub.py) or a replayed bag from the fixed
D415s — so obstacles that APPEAR or MOVE enter the ESDF and, when they leave,
DECAY back out. There is no robot in the depth here (sim/replay of fixed cameras),
so there is no robot_segmenter: nothing to mask.

  1. nvblox_node (dynamic config, base_link global frame, depth from --depth-*)
  2. optionally cuMotion with read_esdf_world:=true querying nvblox's ESDF.

cuMotion is optional (with_cumotion:=false for the ESDF-only proof). When on, it
starts immediately: unlike the snapshot launch there is no map to load first, and
nvblox answers the ESDF service as soon as it is up (empty until depth arrives,
which is the honest state — no obstacles seen yet).

NOTE: this launch NEVER commands motion. cuMotion here only PLANS (its action
server); whether any plan reaches a driver is decided elsewhere by the merged
human-safety gate (routing_gate.py / humangate). Simulation only.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _true(ctx, name):
    return LaunchConfiguration(name).perform(ctx).strip().lower() in ("1", "true", "yes")


def _setup(ctx, *args, **kwargs):
    lc = lambda k: LaunchConfiguration(k).perform(ctx)  # noqa: E731
    ex = get_package_share_directory("nvblox_examples_bringup")
    mb = get_package_share_directory("isaac_ros_manipulation_bringup")
    depth_image, depth_info = lc("depth_image"), lc("depth_info")

    nvblox = ComposableNode(
        name="nvblox_node", package="nvblox_ros", plugin="nvblox::NvbloxNode",
        parameters=[
            os.path.join(ex, "config/nvblox/nvblox_base.yaml"),
            os.path.join(mb, "config/nvblox/nvblox_manipulator_base.yaml"),
            os.path.join(mb, "config/nvblox/specializations/"
                             "nvblox_manipulator_realsense.yaml"),
            lc("nvblox_config"),
            {
                "use_sim_time": _true(ctx, "use_sim_time"),
                "mapping_type": lc("mapping_type"),
                "use_depth": True,
                "use_segmentation": _true(ctx, "use_segmentation"),
                "integrate_depth_rate_hz": float(lc("integrate_hz")),
                "update_esdf_rate_hz": float(lc("esdf_hz")),
                "update_mesh_rate_hz": 2.0 if _true(ctx, "viz") else 0.0,
                "publish_layer_rate_hz": 2.0 if _true(ctx, "viz") else 0.0,
            },
        ],
        remappings=[
            ("/camera_0/depth/image", depth_image),
            ("/camera_0/depth/camera_info", depth_info),
            ("/camera_0/color/image", lc("color_image")),
            ("/camera_0/color/camera_info", lc("color_info")),
            # segmentation mask (only used when use_segmentation:=true)
            ("/camera_0/mask/image", lc("mask_image")),
            ("/camera_0/mask/camera_info", depth_info),
        ],
    )

    container = ComposableNodeContainer(
        name="fm_dyn_container", namespace="", package="rclcpp_components",
        executable="component_container_mt", output="screen",
        composable_node_descriptions=[nvblox],
    )
    out = [LogInfo(msg=f"nvblox dynamic layer: mapping_type={lc('mapping_type')}, "
                       f"depth={depth_image}, esdf {lc('esdf_hz')} Hz"),
           container]

    if _true(ctx, "with_cumotion"):
        cumotion = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("isaac_ros_cumotion"),
                "launch", "isaac_ros_cumotion.launch.py")),
            launch_arguments={
                "cumotion_action_server.urdf_file_path": lc("urdf"),
                "cumotion_action_server.xrdf_file_path": lc("xrdf"),
                "cumotion_action_server.read_esdf_world": "true",
                "cumotion_action_server.update_esdf_on_request": "true",
                "cumotion_action_server.esdf_service_name":
                    "/nvblox_node/get_esdf_and_gradient",
                "cumotion_action_server.add_ground_plane": lc("ground_plane"),
                "cumotion_action_server.publish_cumotion_world_as_voxels": lc("voxels"),
                "cumotion_action_server.publish_voxel_size": "0.02",
                "cumotion_action_server.max_publish_voxels": "500000",
                "cumotion_action_server.time_dilation_factor": lc("time_dilation"),
            }.items(),
        )
        out += [LogInfo(msg="starting cuMotion (read_esdf_world=true) against the "
                            "live dynamic ESDF"), cumotion]
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mapping_type", default_value="dynamic"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nvblox_config",
                              default_value="/dynamic/nvblox_dynamic.yaml"),
        DeclareLaunchArgument("depth_image", default_value="/fm_dyn/depth/image_mm"),
        DeclareLaunchArgument("depth_info", default_value="/fm_dyn/depth/camera_info"),
        DeclareLaunchArgument("color_image", default_value="/fm_dyn/color/image"),
        DeclareLaunchArgument("color_info", default_value="/fm_dyn/color/camera_info"),
        DeclareLaunchArgument("mask_image", default_value="/fm_dyn/mask/image"),
        DeclareLaunchArgument("use_segmentation", default_value="false"),
        DeclareLaunchArgument("integrate_hz", default_value="10.0"),
        DeclareLaunchArgument("esdf_hz", default_value="10.0"),
        DeclareLaunchArgument("viz", default_value="false"),
        DeclareLaunchArgument("with_cumotion", default_value="false"),
        DeclareLaunchArgument("urdf", default_value="/cfg/ur10_calibrated.urdf"),
        DeclareLaunchArgument("xrdf", default_value="/cfg/ur10_map.xrdf"),
        DeclareLaunchArgument("ground_plane", default_value="true"),
        DeclareLaunchArgument("voxels", default_value="true"),
        DeclareLaunchArgument("time_dilation", default_value="0.5"),
        OpaqueFunction(function=_setup),
    ])
