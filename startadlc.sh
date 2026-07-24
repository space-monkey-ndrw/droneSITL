#!/bin/bash
source /home/ndrwldr/ros2_ws/install/setup.bash
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="/home/ndrwldr/droneSITL/install/aruco_detector:$AMENT_PREFIX_PATH"
ros2 run aruco_detector aruco_detector_node --ros-args -p use_sim_time:=true &
ros2 run landing_controller landing_controller --ros-args -p use_sim_time:=true
