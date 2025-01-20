#!/bin/bash

# Function to display the menu
display_menu() {
    echo "==========================="
    echo "   Select ROS MASTER IP    "
    echo "==========================="
    echo "  1 -> localhost"
    echo "  2 -> alienware (10.5.98.101)"
    echo "  3 -> edge (10.5.1.21)"
    echo "  4 -> custom"
    echo "==========================="
}

# Function to get user choice
get_choice() {
    read -p "Enter your choice (1/2/3/4): " choice
    case $choice in
        1) ros_master_ip="127.0.0.1"; return;;
        2) ros_master_ip="10.5.98.101"; return;;
        3) ros_master_ip="10.5.1.21"; return;;
        4) read -p "Enter custom ROS MASTER IP: " custom_ip; ros_master_ip="$custom_ip"; return;;
        *) echo "Invalid choice. Please enter 1, 2, 3, or 4."; get_choice;;
    esac
}

# Prompt the user to select ROS MASTER IP
display_menu
get_choice

# Construct ROS MASTER URI
ros_master_uri="http://${ros_master_ip}:11311"

# Run docker container with selected ROS MASTER URI and ROS_IP
echo 'Running go1-robot docker image.'

docker run \
    -itd \
    --name roscore-edge \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_master_ip" \
    --privileged \
    -v ./scripts:/home/go1/scripts \
    go1-roscore:latest \

echo "Done."
