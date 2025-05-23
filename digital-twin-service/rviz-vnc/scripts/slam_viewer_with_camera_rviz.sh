#!/bin/bash

# Debugging: Print out the ROS network configuration to verify.
echo "Using ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "Using ROS_IP: ${ROS_IP}"

# Source ROS and workspace setup scripts
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

rosrun rviz rviz -d /home/ubuntu/scripts/slam_with_camera.rviz
