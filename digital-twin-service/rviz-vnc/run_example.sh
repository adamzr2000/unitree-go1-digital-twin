#!/bin/bash

# Directory on host to map to the container's ros_bags directory
host_ros_bags_dir="$(pwd)/ros_bags"

# Directory on host to map to the container's rviz directory
rviz_dir="$(pwd)/scripts"

# Function to display the menu
display_menu() {
    echo "==========================="
    echo "   Select ROS MASTER IP    "
    echo "==========================="
    echo "  1 -> localhost"
    echo "  2 -> edge (10.5.1.21)"
    echo "  3 -> custom"
    echo "==========================="
}

# Function to get user choice
get_choice() {
    read -p "Enter your choice (1/2/3): " choice
    case $choice in
        1) ros_master_ip="127.0.0.1"; return;;
        2) ros_master_ip="10.5.1.21"; return;;
        3) read -p "Enter custom ROS MASTER IP: " custom_ip; ros_master_ip="$custom_ip"; return;;
        *) echo "Invalid choice. Please enter 1, 2, or 3."; get_choice;;
    esac
}

# Prompt the user to select ROS MASTER IP
display_menu
get_choice

# Construct ROS MASTER URI
ros_master_uri="http://${ros_master_ip}:11311"
ros_ip="127.0.0.1"

docker run \
    --name rviz-vnc \
    --rm \
    -p 6080:80 \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -v ${host_ros_bags_dir}:/home/ubuntu/ros_bags \
    -v ${rviz_dir}:/home/ubuntu/scripts \
    go1-rviz-vnc:latest \

echo "Done."

