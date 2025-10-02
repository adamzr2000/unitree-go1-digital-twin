#!/bin/bash

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
container_image="phidgets-imu:latest"

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

# Validate inputs
validate_ip() {
    local ip=$1
    if [[ ! $ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "Invalid IP address format: $ip"
        exit 1
    fi
}

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
echo "Running $container_image docker image."

host_catkin_ws_dir="$(pwd)/catkin_ws/src"

docker run \
    -it \
    --privileged \
    --name phidgets-imu \
    --rm \
    --net host \
    -v $host_catkin_ws_dir:/home/phidgets-imu/catkin_ws/src \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -e FRAME_ID="phidgets_imu_link" \
    -e FIXED_FRAME="base" \
    $container_image