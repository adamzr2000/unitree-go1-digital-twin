#!/bin/bash

# Source ROS and workspace setup scripts
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

roslaunch vizanti server.launch --wait

