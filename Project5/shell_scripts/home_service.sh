#!/bin/sh

export
TURTLEBOT_GAZEBO_WORLD_FILE=/home/robond/catkin_ws/src/worlds/new_world.world
TURTLEBOT_GAZEBO_MAP_FILE=/home/robond/catkin_ws/src/maps/map.yaml

xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash " & 
sleep 10

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/worlds/new_world.world " &
sleep 10


xterm -e " roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/robond/catkin_ws/src/maps/map.yaml " &
sleep 10


xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10

xterm -e " rosrun pick_objects pick_objects " &
sleep 5

xterm -e " rosrun add_markers add_markers"

