#!/bin/bash

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
rf2o_odometry="true"
odom_topic="/odom"

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
        --rf2o-laser-odometry)
            rf2o_odometry="$2"
            shift # past argument
            shift # past value
            ;;
        --odom-topic)
            odom_topic="$2"
            shift # past argument
            shift # past value
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--ros-master-uri <ROS_MASTER_URI>] [--ros-ip <ROS_IP>] [--rf2o-laser-odometry <true/false>] [--odom-topic <ODOM_TOPIC>]"
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
echo "RF2O_LASER_ODOMETRY: $rf2o_odometry"
echo "ODOM_TOPIC: $odom_topic"
echo "==============================="

# Run docker container
echo "Running rplidar docker image."

host_catkin_ws_dir="$(pwd)/catkin_ws/src"

docker run \
    -it \
    --name lidar \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -e RF2O_LASER_ODOMETRY="$rf2o_odometry" \
    -e ODOM_TOPIC="$odom_topic" \
    --device=/dev/rplidar \
    rplidar-lidar:latest