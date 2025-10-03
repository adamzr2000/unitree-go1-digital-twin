#!/bin/bash

# Default values
container_image="vizanti"
container_name="vizanti"

ros_master_uri="http://ros-master:11311"

echo "==========================="
echo " Parameters in Use"
echo "==========================="
echo "ROS_MASTER_URI: $ros_master_uri"
echo "==========================="

echo "Running $container_image docker image."

docker run \
    -it \
    --name $container_name \
    --hostname $container_name \
    --rm \
    --net digital-twin-service \
    -p 5000:5000 \
    -p 5001:5001 \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_HOSTNAME="$container_name" \
    -e IP_ADDRESS_ROSBRIDGE="0.0.0.0" \
    $container_image:latest