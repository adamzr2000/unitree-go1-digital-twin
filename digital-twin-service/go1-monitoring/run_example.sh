#!/bin/bash

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
topics="/scan /joint_states /go1_controller/odom /go1_controller/cmd_vel"
influxdb_url="http://localhost:8086"
influxdb_token="desire6g2024;"
influxdb_org="desire6g"
influxdb_bucket="infrastructure-monitoring"
window_size="50"

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
            shift
            shift
            ;;
        --ros-ip)
            ros_ip="$2"
            shift
            shift
            ;;
        --topics)
            topics="$2"
            shift
            shift
            ;;
        --influxdb-url)
            influxdb_url="$2"
            shift
            shift
            ;;
        --influxdb-token)
            influxdb_token="$2"
            shift
            shift
            ;;
        --influxdb-org)
            influxdb_org="$2"
            shift
            shift
            ;;
        --influxdb-bucket)
            influxdb_bucket="$2"
            shift
            shift
            ;;
        --window-size)
            window_size="$2"
            shift
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo ""
            echo "Usage: $0 [--ros-master-uri <URI>] [--ros-ip <IP>] [--topics \"<TOPIC LIST>\"]"
            echo "          [--influxdb-url <URL>] [--influxdb-token <TOKEN>]"
            echo "          [--influxdb-org <ORG>] [--influxdb-bucket <BUCKET>]"
            echo "          [--window-size <SIZE>]"
            exit 1
            ;;
    esac
done

# Validate required IPs
validate_ip "$ros_ip"


# Display parameters in use
echo "==============================="
echo " Parameters in Use             "
echo "==============================="
echo "ROS_MASTER_URI : $ros_master_uri"
echo "ROS_IP         : $ros_ip"
echo "TOPICS         : $topics"
echo "INFLUXDB_URL   : $influxdb_url"
echo "INFLUXDB_TOKEN : $influxdb_token"
echo "INFLUXDB_ORG   : $influxdb_org"
echo "INFLUXDB_BUCKET: $influxdb_bucket"
echo "WINDOW_SIZE    : $window_size"
echo "==============================="

# Run docker container
echo "Running go1 docker image..."

# Directory on host to map to the container's app directory
host_app_dir="$(pwd)/app"

# Run docker container with selected ROS MASTER URI and ROS_IP
echo 'Running go1 docker image.'

docker run \
    -it \
    --name go1-monitoring \
    --hostname go1-monitoring \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -e TOPICS="$topics" \
    -e INFLUXDB_URL="$influxdb_url" \
    -e INFLUXDB_TOKEN="$influxdb_token" \
    -e INFLUXDB_ORG="$influxdb_org" \
    -e INFLUXDB_BUCKET="$influxdb_bucket" \
    -e WINDOW_SIZE="$window_size" \
    -v ${host_app_dir}:/home/go1/app \
    --privileged \
    go1-monitoring:latest
