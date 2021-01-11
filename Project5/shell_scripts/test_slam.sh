#!/bin/sh

export
TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/catkin_ws/src/worlds/new_world.world

xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash " & 
sleep 5

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/worlds/new_world.world" &
sleep 5

xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
