#!/bin/bash

# Debugging: Print out the ROS network configuration to verify.
echo "Using ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "Using ROS_IP: ${ROS_IP}"

# Source ROS and workspace setup scripts
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash 

roslaunch go1_description go1_rviz.launch --wait
# rosrun rviz rviz
