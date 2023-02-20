#! /usr/bin/env bash
set -e
RUN_AFTER_BASHRC="./gazebo_px4.sh" gnome-terminal --title="Gazebo Sim" --tab &
sleep 5 &&
# RUN_AFTER_BASHRC="~/QGroundControl.AppImage" gnome-terminal --title="QGC" --tab;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch fsm px4_pos_estimator.launch" gnome-terminal --title="est" --tab;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch model_state iris_gazebo.launch" gnome-terminal --title="remap" --tab &
sleep 5 &&
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch fsm UAV2_single.launch" gnome-terminal --title="connection" --tab;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch fsm swarm.launch" gnome-terminal --title="control" --window & 
sleep 1 ;
# RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch mapping mapping.launch" gnome-terminal --title="fiesta" --tab ;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch grid_path_searcher astar_node_3d.launch" gnome-terminal --title="astar" --tab & 
sleep 1 &&
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch bspline_race traj_testing.launch" gnome-terminal --title="bspline" --tab &
sleep 1;
#RUN_AFTER_BASHRC="rosbag record /topic1_name /topic2_name /xxx" gnome-terminal --title="rosbag" --tab;
RUN_AFTER_BASHRC="rviz -d ~/ascup_ws/demo.rviz" gnome-terminal --title="rviz" --tab;

#RUN_AFTER_BASHRC="bash ~/FLAG_ws/shell/topic.sh" gnome-terminal --title="topic" --window & 
#sleep 1
wait
exit 0