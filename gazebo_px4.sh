#! /usr/bin/env bash
set -e
source devel/setup.bash
export FIRMWARE_DIR=/home/flag/tools/PX4-Autopilot
#export FIRMWARE_DIR=/home/flag/tools/old_px4/PX4-Autopilot
echo $FIRMWARE_DIR
source $FIRMWARE_DIR/Tools/setup_gazebo.bash $FIRMWARE_DIR $FIRMWARE_DIR/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FIRMWARE_DIR
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FIRMWARE_DIR/Tools/sitl_gazebo
roslaunch fsm iris_realsense_camera_px4_mavros_vo.launch
