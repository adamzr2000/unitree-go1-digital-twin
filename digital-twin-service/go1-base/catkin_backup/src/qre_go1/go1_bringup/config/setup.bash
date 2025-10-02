# Pass through to the main ROS workspace of the system.
source /opt/ros/noetic/setup.bash
source /home/unitree/catkin_ws/devel/setup.bash

# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

######
export GO1_LOGITECH=1
export GO1_GPS_ODOM=false
# ROS Environment Variables
# export ROS_MASTER_URI=http://192.168.123.13:11311/
# export ROS_IP=192.168.123.13
# export ROS_HOSTNAME=192.168.123.13
