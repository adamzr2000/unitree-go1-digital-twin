#!/bin/bash

# Default values
ros_master_uri="http://ros-master:11311"
container_image="rviz-vnc"

# Run docker container
echo "Running $container_image docker image."

host_ros_bags_dir="$(pwd)/ros_bags"
rviz_dir="$(pwd)/scripts"
catkin_ws_dir="$(pwd)/catkin_ws/src"

docker run \
    --name $container_image \
    --rm \
    -p 6080:80 \
    --net digital-twin-service \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e DISABLE_ROS1_EOL_WARNINGS=1 \
    -v ${catkin_ws_dir}:/home/ubuntu/Desktop/catkin_ws/src \
    -v ${host_ros_bags_dir}:/home/ubuntu/Desktop/ros_bags \
    -v ${rviz_dir}:/home/ubuntu/Desktop/scripts \
    --privileged \
    --runtime=nvidia \
    $container_image:latest \

echo "Done."

