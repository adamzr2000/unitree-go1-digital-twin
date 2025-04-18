#!/bin/bash

# Function to clean up and terminate both processes
cleanup() {
    # Kill the Python process
    kill "$python_pid"

    # Kill the screen session
    screen -S ros-gesture-control-cmd -X quit
    
    exit 0
}

# Environment variables with default values using parameter expansion
camera_type="${CAMERA_TYPE,,}" 
web_server="${WEB_SERVER,,}"
control_loop_rate="${CONTROL_LOOP_RATE:-50}" 
stamped="${STAMPED:-true}"
cmd_vel="${CMD_VEL:-go1_controller/cmd_vel}"


# Determine the appropriate Python script based on CAMERA_TYPE and WEB_SERVER
case "$camera_type" in
    "webcam")
        if [[ "$web_server" == "yes" ]]; then
            python_script="gesture_capture_webcam_web.py"
            start_command="flask run --host=0.0.0.0 --port=8888"
        else
            python_script="gesture_capture_webcam.py"
            start_command="python3 $python_script"
        fi
        camera_type="Webcam"
        ;;
    "webcam_ip")
        python_script="gesture_capture_webcam_web_ip.py"
        start_command="flask run --host=0.0.0.0 --port=8888"
        camera_type="Webcam"
        ;;
    *)
        echo "Invalid or no CAMERA_TYPE environment variable set, defaulting to 'Webcam'."
        if [[ "$web_server" == "yes" ]]; then
            python_script="gesture_capture_webcam_web.py"
            start_command="flask run --host=0.0.0.0 --port=8888"
        else
            python_script="gesture_capture_webcam.py"
            start_command="python3 $python_script"
        fi
        camera_type="Webcam"
        ;;
esac

echo "Using camera type: $camera_type with script: $python_script and control loop rate: $control_loop_rate"
echo "cmd_vel topic: $cmd_vel, stamped: $stamped"

# Trap Ctrl+C signal (SIGINT) and call the cleanup function
trap cleanup SIGINT

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash 

# Launch the ROS node with the specified control loop rate
screen -S ros-gesture-control-cmd -dm roslaunch unitree_legged_real gesture_control_cmd.launch control_loop_rate:=$control_loop_rate stamped:=$stamped cmd_vel:=$cmd_vel --wait

# screen -S ros-gesture-control-cmd -dm roslaunch unitree_legged_real gesture_control_cmd.launch --wait

sleep 2

# Start the selected Python script in the foreground based on WEB_SERVER variable
if [[ "$web_server" == "yes" ]]; then
    export FLASK_APP="$python_script"
    $start_command &
else
    $start_command &
fi

python_pid=$!

# Wait for the Python process to finish
wait $python_pid