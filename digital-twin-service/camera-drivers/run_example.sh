#!/bin/bash

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"

# Function to validate IP address format
validate_ip() {
    local ip=$1
    if [[ ! $ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "Invalid IP address format: $ip"
        exit 1
    fi
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --ros-master-uri)
            ros_master_uri="$2"
            shift # past argument
            shift # past value
            ;;
        --ros-ip)
            ros_ip="$2"
            shift # past argument
            shift # past value
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--ros-master-uri <ROS_MASTER_URI>] [--ros-ip <ROS_IP>]"
            exit 1
            ;;
    esac
done

# Validate ROS_IP
validate_ip "$ros_ip"

# Display parameters in use
echo "==============================="
echo " Parameters in Use             "
echo "==============================="
echo "ROS_MASTER_URI: $ros_master_uri"
echo "ROS_IP: $ros_ip"
echo "==============================="

# Run docker container
echo "Running astra-camera docker image."

# Directory on host to map to the container's catkin_ws directory
host_catkin_ws_dir="$(pwd)/catkin_ws/src"

docker run \
    -it \
    --name astra-camera \
    --rm \
    --net host \
    --privileged \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    --device=/dev/astra \
    astra-camera:latest
