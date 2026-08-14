#!/usr/bin/env bash
# Start MoveIt + RViz for the UR10, self-healing the RViz kinematics params.
#
# Why this dance (Jazzy + ur_moveit_config 3.8.0): ANY kinematics/joint_limits/
# planning params pre-declared on the RViz node make the MotionPlanning
# display's loadRobotModel die with "Wrong parameter type ... setting it to
# {string} is not allowed" -> RViz shows no robot at all. Starting bare renders
# the robot but leaves the display without an IK solver -> no drag marker.
# Post-init parameter writes ARE accepted, so we start bare and inject the
# solver once the display has self-declared its params.
#
# After injection, the display reloads its model on the next "Reset" click
# (bottom-left in RViz) or MotionPlanning display off/on toggle — then the
# orange goal arm + drag ball appear.
source /opt/ros/jazzy/setup.bash
# Overlay with the host-built cuMotion MoveIt plugin (adds the isaac_ros_cumotion
# planning pipeline). Harmless if the workspace is absent.
[ -f "$HOME/workspaces/cumotion_src/install/setup.bash" ] && \
  source "$HOME/workspaces/cumotion_src/install/setup.bash"
(
  # Inject the IK solver and KEEP verifying it stuck: the display's first
  # loadRobotModel can land AFTER our write and overwrite the solver back to
  # "" (observed twice). Re-assert until the read-back is stable for two
  # consecutive checks, then announce.
  stable=0
  for _ in $(seq 1 90); do
    cur=$(timeout 5 ros2 param get /rviz2_moveit \
            robot_description_kinematics.ur_manipulator.kinematics_solver 2>/dev/null \
          | grep -o "KDLKinematicsPlugin")
    if [ -n "$cur" ]; then
      stable=$((stable + 1))
      [ "$stable" -ge 2 ] && { echo "[start_moveit] IK solver injected and stable — click Reset in RViz once"; break; }
    else
      stable=0
      ros2 param set /rviz2_moveit \
        robot_description_kinematics.ur_manipulator.kinematics_solver \
        kdl_kinematics_plugin/KDLKinematicsPlugin >/dev/null 2>&1
      ros2 param set /rviz2_moveit \
        robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution \
        0.005 >/dev/null 2>&1
    fi
    sleep 2
  done
) &
exec ros2 launch "$HOME/ur10/moveit/ur10_moveit.launch.py" ur_type:=ur10 launch_rviz:=true "$@"
