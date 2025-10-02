#!/bin/bash

# Default values
ros_master_ip="127.0.0.1"
container_image="ros-master"

# Parse arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --ros-master-ip)
            ros_master_ip="$2"
            shift # past argument
            shift # past value
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 --ros-master-ip <ROS_MASTER_IP>"
            exit 1
            ;;
    esac
done

# Validate ROS_MASTER_IP
if [[ -z "$ros_master_ip" ]]; then
    echo "ROS_MASTER_IP is required. Use --ros-master-ip to specify it."
    exit 1
fi

# Construct ROS_MASTER_URI
ros_master_uri="http://${ros_master_ip}:11311"

echo "==========================="
echo " Parameters in Use"
echo "==========================="
echo "ROS_MASTER_IP: $ros_master_ip"
echo "ROS_MASTER_URI: $ros_master_uri"
echo "==========================="

echo "Running $container_image docker image."

docker run \
    -itd \
    --name $container_image \
    --rm \
    --net host \
    -v ./scripts:/home/go1/scripts \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_master_ip" \
    $container_image:latest

echo "Done."
