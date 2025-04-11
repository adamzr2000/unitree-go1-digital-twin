xhost local:root

XAUTH=/tmp/.docker.xauth

# Default values
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
camera_type="webcam_ip"
web_server="yes"
control_loop_rate="500"
cmd_vel="go1_controller/cmd_vel"
stamped="true"

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
        --camera-type)
            camera_type="$2"
            shift # past argument
            shift # past value
            ;;
        --web-server)
            web_server="$2"
            shift # past argument
            shift # past value
            ;;
        --control-loop-rate)
            control_loop_rate="$2"
            shift # past argument
            shift # past value
            ;;
        --cmd-vel)
            cmd_vel="$2"
            shift # past argument
            shift # past value
            ;;
        --stamped)
            stamped="$2"
            shift # past argument
            shift # past value
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--ros-master-uri <ROS_MASTER_URI>] [--ros-ip <ROS_IP>] [--camera-type <webcam/intel/webcam_ip>] [--web-server <yes/no>] [--control-loop-rate <RATE>] [--cmd-vel <TOPIC>] [--stamped <true/false>]"
            exit 1
            ;;
    esac
done

# Validate ROS_IP
validate_ip "$ros_ip"

# Display parameters in use
echo "==============================="
echo " Parameters in Use             "
echo "==============================="
echo "ROS_MASTER_URI: $ros_master_uri"
echo "ROS_IP: $ros_ip"
echo "CAMERA_TYPE: $camera_type"
echo "WEB_SERVER: $web_server"
echo "CONTROL_LOOP_RATE (Hz): $control_loop_rate"
echo "CMD_VEL: $cmd_vel"
echo "STAMPED: $stamped"
echo "==============================="

# Run docker container
echo "Running go1-gesture-control docker image."
# Directory on host to map to the container's app directory
host_app_dir="$(pwd)/app"

docker run \
    -it \
    --name gesture-control-app \
    --hostname gesture-control-app \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="QT_QPA_PLATFORM=xcb" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_ip" \
    -e CAMERA_TYPE="$camera_type" \
    -e WEB_SERVER="$web_server" \
    -e CMD_VEL="$cmd_vel" \
    -e STAMPED="$stamped" \
    -e CONTROL_LOOP_RATE="$control_loop_rate" \
    -v ${host_app_dir}:/home/go1/app \
    --rm \
    --net host \
    --privileged \
    --runtime=nvidia \
    --group-add video \
    go1-gesture-control:latest 

echo "Done."
