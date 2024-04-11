#!/bin/bash

# Source ROS and workspace setup scripts
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

# Function to validate IP address format
validate_ip() {
    if [[ $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        IFS='.' read -r -a ip_array <<< "$1"
        for segment in "${ip_array[@]}"; do
            if ((segment < 0 || segment > 255)); then
                echo "Invalid IP address: $1"
                return 1
            fi
        done
        return 0
    else
        echo "Invalid IP address format: $1"
        return 1
    fi
}

# Read the target IP address from an environment variable
target_ip=${TARGET_IP:-"192.168.123.161"} # Default IP if TARGET_IP is not set

# Validate the IP address format
if ! validate_ip "$target_ip"; then
    echo "Invalid or no TARGET_IP environment variable set, using default IP address."
    target_ip="192.168.123.161" # Default IP if TARGET_IP has an invalid format
fi

# Debug: Echo the target_ip to ensure it's being set correctly
echo "Using target_ip: $target_ip"

# Launch with the selected or entered target IP address
roslaunch go1_bringup bringup.launch target_ip:=$target_ip --wait
