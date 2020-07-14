#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/ubuntu/workspaces/catkin_ws/src/Project_home_service/map/Project2_extended.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/ubuntu/workspaces/catkin_ws/src/Project_home_service/map/map2.yaml" &
sleep 5
xterm -e "rosrun rviz rviz -d /home/ubuntu/workspaces/catkin_ws/src/Project_home_service/rvizConfig/rvizConfig.rviz" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" 
