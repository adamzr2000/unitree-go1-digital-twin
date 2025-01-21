#!/bin/bash

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
target_ip="192.168.123.161"
state_loop_rate="100"
use_ekf_odom="true"
udp_send_dt="0.01"
udp_recv_dt="0.01"

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
        --target-ip)
            target_ip="$2"
            shift # past argument
            shift # past value
            ;;
        --state-loop-rate)
            state_loop_rate="$2"
            shift # past argument
            shift # past value
            ;;
        --use-ekf-odom)
            use_ekf_odom="$2"
            shift # past argument
            shift # past value
            ;;
        --udp-send-dt)
            udp_send_dt="$2"
            shift # past argument
            shift # past value
            ;;
        --udp-recv-dt)
            udp_recv_dt="$2"
            shift # past argument
            shift # past value
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--ros-master-uri <ROS_MASTER_URI>] [--ros-ip <ROS_IP>] [--target-ip <TARGET_IP>] [--state-loop-rate <RATE>] [--use-ekf-odom <true/false>] [--udp-send-dt <VALUE>] [--udp-recv-dt <VALUE>]"
            exit 1
            ;;
    esac
done

# Validate ROS_IP and TARGET_IP
validate_ip "$ros_ip"
validate_ip "$target_ip"

# Display parameters in use
echo "==============================="
echo " Parameters in Use             "
echo "==============================="
echo "ROS_MASTER_URI: $ros_master_uri"
echo "ROS_IP: $ros_ip"
echo "TARGET_IP: $target_ip"
echo "STATE_LOOP_RATE: $state_loop_rate"
echo "USE_EKF_ODOM: $use_ekf_odom"
echo "UDP_SEND_DT: $udp_send_dt"
echo "UDP_RECV_DT: $udp_recv_dt"
echo "==============================="

# Run docker container
echo "Running go1 docker image."

docker run \
    -it \
    --name go1-base \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -e TARGET_IP="$target_ip" \
    -e STATE_LOOP_RATE="$state_loop_rate" \
    -e USE_EKF_ODOM="$use_ekf_odom" \
    -e UDP_SEND_DT="$udp_send_dt" \
    -e UDP_RECV_DT="$udp_recv_dt" \
    --privileged \
    go1-base:latest
