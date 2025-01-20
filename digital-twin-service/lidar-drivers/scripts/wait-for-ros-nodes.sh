#!/bin/bash

# Source ROS and workspace setup files
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash 

# Check if RF2O_LASER_ODOMETRY is set to true, default to false if not set
if [ "${RF2O_LASER_ODOMETRY,,}" == "true" ]; then
  LAUNCH_FILE="rplidar_go1_rf2o.launch"
else
  LAUNCH_FILE="rplidar_go1.launch"
fi

# Check if ODOM_TOPIC is set, otherwise default to /odom_rf2o
if [ -z "$ODOM_TOPIC" ]; then
  ODOM_TOPIC="/odom_rf2o"
fi

echo "Using ODOM_TOPIC: $ODOM_TOPIC"
echo "Launching file: $LAUNCH_FILE"

# Launch the selected file with the ODOM_TOPIC argument
roslaunch rplidar_ros $LAUNCH_FILE --wait odom_topic:=$ODOM_TOPIC
