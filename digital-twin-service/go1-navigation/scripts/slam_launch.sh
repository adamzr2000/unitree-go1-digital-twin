#!/bin/bash

# Source ROS and workspace setup scripts
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

# Read the SLAM algorithm from an environment variable, default to hector if not set
slam_algorithm=${SLAM_ALGORITHM:-"cartographer"}

echo "Selected SLAM Algorithm: $slam_algorithm"

case $slam_algorithm in
    "gmapping")
        roslaunch unitree_navigation slam.launch rviz:=false algorithm:=gmapping --wait
        ;;
    "hector")
        roslaunch unitree_navigation slam.launch rviz:=false algorithm:=hector --wait
        ;;
    "cartographer")
        roslaunch go1_navigation_cartographer slam.launch rviz:=false --wait
        ;;
    *)
        echo "Invalid SLAM algorithm specified: $slam_algorithm"
        echo "Using default SLAM algorithm 'hector'."
        roslaunch unitree_navigation slam.launch rviz:=true algorithm:=hector --wait
        ;;
esac

# roslaunch go1_navigation_cartographer slam.launch rviz:=true algorithm:=hector
