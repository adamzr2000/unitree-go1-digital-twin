#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash 

roslaunch rplidar_ros rplidar_go1.launch --wait 
