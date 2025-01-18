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

echo "Launching file: $LAUNCH_FILE"

# Launch the selected file
roslaunch rplidar_ros $LAUNCH_FILE --wait
