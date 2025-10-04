#!/bin/bash

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
topic="chatter"
rate="1.0"
message="Hello from publisher!"
container_image="dummy-ros-app"

# Parse options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --ros-master-uri)
            ros_master_uri="$2"
            shift 2
            ;;
        --ros-ip)
            ros_ip="$2"
            shift 2
            ;;    
        --topic)
            topic="$2"
            shift 2
            ;;
        --rate)
            rate="$2"
            shift 2
            ;;
        --message)
            message="$2"
            shift 2
            ;;            
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Display parameters in use
echo "==============================="
echo " Parameters in Use             "
echo "==============================="
echo "ROS_MASTER_URI: $ros_master_uri"
echo "ROS_IP: $ros_ip"
echo "Topic: $topic"
echo "Rate: $rate"
echo "Message: $message"
echo "==============================="

# Run the container
docker run \
    -it \
    --name ros-publisher \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -e ROS_ROLE="publisher" \
    -e ROS_TOPIC="$topic" \
    -e ROS_RATE="$rate" \
    -e ROS_MESSAGE="$message" \
    $container_image:latest
