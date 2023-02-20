#! /usr/bin/env bash
set -e
RUN_AFTER_BASHRC="~/QGroundControl.AppImage" gnome-terminal --title="QGC" --tab;
RUN_AFTER_BASHRC="rosrun remap_node message_remap" gnome-terminal --title="remap" --tab &
sleep 2 &&
RUN_AFTER_BASHRC="./single.sh" gnome-terminal --title="PX4" --tab & 
sleep 2 && 
RUN_AFTER_BASHRC="./estimater.sh" gnome-terminal --title="PX4" --tab & 
sleep 2 && 
RUN_AFTER_BASHRC="./swarm.sh" gnome-terminal --title="control" --window & 
sleep 2;
RUN_AFTER_BASHRC="./mapping.sh" gnome-terminal --title="mapping" --tab & 
sleep 2 && 
RUN_AFTER_BASHRC="rviz -d demo.rviz" gnome-terminal --title="rviz" --tab;
RUN_AFTER_BASHRC="./astar.sh" gnome-terminal --title="astar" --tab;
RUN_AFTER_BASHRC="./bspline.sh" gnome-terminal --title="planning" --tab;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch perception_pkg perception.launch" gnome-terminal --title="perception" --tab;
RUN_AFTER_BASHRC="python2 ../catkin_ws_SLAM/src/circle_recognition/src/circle_recognition.py" gnome-terminal --title="circle" --tab;
RUN_AFTER_BASHRC="python2 ../catkin_ws_SLAM/src/circle_recognition_bule/src/circle_recognition.py" gnome-terminal --title="circle_blue" --tab;
RUN_AFTER_BASHRC="python2 ../catkin_ws_SLAM/src/detect_walls/src/detec_walls.py" gnome-terminal --title="wall" --tab;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch apriltag_ros continuous_detection.launch" gnome-terminal --title="wall" --tab;

wait
exit 0
