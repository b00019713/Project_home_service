#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/ubuntu/workspaces/catkin_ws/src/Project_home_service/map/Project2_extended.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/ubuntu/workspaces/catkin_ws/src/Project_home_service/map/map2.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" 
sleep 5


