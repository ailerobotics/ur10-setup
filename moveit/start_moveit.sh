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
(
  for _ in $(seq 1 60); do
    # succeeds only once the display has declared the param (post first load)
    if ros2 param set /rviz2_moveit \
         robot_description_kinematics.ur_manipulator.kinematics_solver \
         kdl_kinematics_plugin/KDLKinematicsPlugin >/dev/null 2>&1; then
      ros2 param set /rviz2_moveit \
        robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution \
        0.005 >/dev/null 2>&1 || true
      echo "[start_moveit] IK solver injected into RViz node — click Reset in RViz once"
      break
    fi
    sleep 1
  done
) &
exec ros2 launch "$HOME/ur10/moveit/ur10_moveit.launch.py" ur_type:=ur10 launch_rviz:=true "$@"
