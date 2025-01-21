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

# Read the EKF odometry usage flag from an environment variable
use_ekf_odom=${USE_EKF_ODOM:-"true"} # Default to true if USE_EKF_ODOM is not set

# Validate the EKF odometry flag
if [[ "$use_ekf_odom" != "true" && "$use_ekf_odom" != "false" ]]; then
    echo "Invalid value for USE_EKF_ODOM: $use_ekf_odom. Defaulting to true."
    use_ekf_odom="true"
fi

state_loop_rate=${STATEE_LOOP_RATE:-"100"}
udp_send_dt=${UDP_SEND_DT:-"0.005"}
udp_recv_dt=${UDP_RECV_DT:-"0.005"}

# Debug: Echo the variables to ensure they are being set correctly
echo "Using target_ip: $target_ip"
echo "Using use_ekf_odom: $use_ekf_odom"
echo "Using state_loop_rate: $state_loop_rate"
echo "Using udp_send_dt: $udp_send_dt"
echo "Using udp_recv_dt: $udp_recv_dt"

# Launch with the selected or entered target IP address and EKF odometry flag
roslaunch go1_bringup bringup.launch target_ip:=$target_ip use_ekf_odom:=$use_ekf_odom state_loop_rate:=$state_loop_rate udp_send_dt:=$udp_send_dt udp_recv_dt:=$udp_recv_dt --wait