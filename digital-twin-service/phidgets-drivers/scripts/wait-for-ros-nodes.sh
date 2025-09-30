#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

FRAME_ID="${FRAME_ID:-phidgets_imu_link}"
FIXED_FRAME="${FIXED_FRAME:-phidgets_odom}"

roslaunch phidgets_spatial spatial.launch \
  frame_id:="${FRAME_ID}" \
  fixed_frame:="${FIXED_FRAME}" \
  --wait
