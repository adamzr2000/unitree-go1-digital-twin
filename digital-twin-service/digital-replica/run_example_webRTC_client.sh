#!/bin/bash

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
        1) ros_master_ip="localhost"; return;;
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

docker run --name go1-digital-twin --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -v ./go1-environments:/isaac-sim/go1-environments \
    isaac-sim:2023.1.0-ubuntu22.04 \
    ./runheadless.webrtc.sh -v 
