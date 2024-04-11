#!/bin/bash

# Function to clean up and terminate both processes
cleanup() {
    # Kill the Python process
    kill "$python_pid"

    # Kill the screen session
    screen -S ros-gesture-control-cmd -X quit
    rosnode kill turtlesim

    exit 0
}

# Default to "webcam" if CAMERA_TYPE is not set or set incorrectly
camera_type=${CAMERA_TYPE,,} # Convert to lowercase to ensure compatibility
case "$camera_type" in
    "webcam")
        python_script="gesture_capture_webcam.py"
        camera_type="Webcam"
        ;;
    "intel")
        python_script="gesture_capture_d435i.py"
        camera_type="Intel D435i"
        ;;
    *)
        echo "Invalid or no CAMERA_TYPE environment variable set, defaulting to 'Webcam'."
        python_script="gesture_capture_webcam.py"
        camera_type="Webcam"
        ;;
esac

echo "Selected $camera_type"

# Trap Ctrl+C signal (SIGINT) and call the cleanup function
trap cleanup SIGINT

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash 

echo "Setting up processes..."

# Start the first process in the background
screen -S ros-gesture-control-cmd -dm roslaunch unitree_legged_real gesture_control_cmd.launch --wait

# Start the turtlesim node in the background
screen -S ros-turtlesim -dm rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel --wait

echo "Processes started successfully."

echo "Launching Gesture Capture using $camera_type..."

# Start the selected Python script in the foreground
python3 "$python_script" &
python_pid=$!

# Wait for the Python process to finish
wait $python_pid
