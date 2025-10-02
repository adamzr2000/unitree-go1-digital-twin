#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

# rosrun teleop_twist_keyboard teleop_twist_keyboard.py

rosrun teleop_twist_keyboard teleop_twist_keyboard.py _stamped:=True _frame_id:=cmd_vel_frame _key_timeout:=0.6 cmd_vel:=go1_controller/cmd_vel