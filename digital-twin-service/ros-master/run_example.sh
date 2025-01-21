#!/bin/bash

# Default values
ros_master_ip="127.0.0.1"

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

# Display the selected parameters
echo "==========================="
echo " Parameters in Use"
echo "==========================="
echo "ROS_MASTER_IP: $ros_master_ip"
echo "ROS_MASTER_URI: $ros_master_uri"
echo "==========================="

# Run docker container with selected ROS_MASTER_URI and ROS_IP
echo 'Running go1-robot docker image.'

docker run \
    -itd \
    --name roscore-edge \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_master_ip" \
    -v ./scripts:/home/go1/scripts \
    go1-roscore:latest

echo "Done."
