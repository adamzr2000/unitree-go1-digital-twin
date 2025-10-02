#!/bin/bash

# Source ROS and workspace setup scripts
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

# Read the SLAM algorithm from an environment variable, default to hector if not set
slam_algorithm=${SLAM_ALGORITHM:-"cartographer"}

use_odom=${USE_ODOM:-"false"}
use_imu=${USE_IMU:-"false"}

echo "Selected SLAM Algorithm: $slam_algorithm"
echo "Use odom: $use_odom"
echo "Use imu: $use_imu"

case $slam_algorithm in
    "gmapping")
        roslaunch unitree_navigation slam.launch rviz:=false algorithm:=gmapping --wait
        ;;
    "hector")
        roslaunch unitree_navigation slam.launch rviz:=false algorithm:=hector --wait
        ;;
    "cartographer")
        roslaunch go1_navigation_cartographer slam.launch rviz:=false use_odom:=$use_odom use_imu:=$use_imu --wait
        ;;
    *)
        echo "Invalid SLAM algorithm specified: $slam_algorithm"
        echo "Using default SLAM algorithm 'hector'."
        roslaunch unitree_navigation slam.launch rviz:=true algorithm:=hector --wait
        ;;
esac

# roslaunch go1_navigation map_navi.launch