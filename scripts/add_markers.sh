#!/bin/sh
xterm  -e  " source ../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find add_markers)/../worlds/UdacityOffice.world" & 
sleep 5
xterm  -e  " source ../../devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch world_file:=$(rospack find add_markers)/../worlds/UdacityOffice.world map_file:=$(rospack find add_markers)/../maps/map.yaml" & 
sleep 5
xterm  -e  " source ../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " source ../../devel/setup.bash; roslaunch add_markers add_markers.launch"