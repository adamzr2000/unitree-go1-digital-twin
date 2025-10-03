#!/bin/bash
set -euo pipefail

# Defaults
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
container_image="go1-navigation"
container_name="go1-navigation"
use_odom="false"
use_imu="false"
slam_algorithm="cartographer"
navigate="false" 

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --ros-master-uri <URI>   ROS master URI (default: ${ros_master_uri})
  --ros-ip <IP>            ROS_IP of this host (default: ${ros_ip})
  --use-odom               Enable odometry input for Cartographer (sets CARTOGRAPHER_USE_ODOM=true)
  --use-imu                Enable IMU input for Cartographer (sets CARTOGRAPHER_USE_IMU=true)
  --navigate               Run navigation instead of SLAM (sets NAVIGATE=true)
  --container-name <name>  Docker container name (default: ${container_name})
  --image <name>           Docker image name (default: ${container_image})
  -h, --help               Show this help
EOF
}

# IP format validator
validate_ip() {
  local ip=$1
  if [[ ! $ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid IP address format: $ip"
    exit 1
  fi
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-master-uri)
      ros_master_uri="$2"; shift 2 ;;
    --ros-ip)
      ros_ip="$2"; shift 2 ;;
    --use-odom)
      use_odom="true"; shift ;;
    --use-imu)
      use_imu="true"; shift ;;
    --navigate)
      navigate="true"; shift ;;
    --container-name)
      container_name="$2"; shift 2 ;;
    --image)
      container_image="$2"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

# Validate
validate_ip "$ros_ip"

# Prepare host bind mounts (create if missing)
host_catkin_ws_dir="$(pwd)/catkin_ws/src"
host_utils_dir="$(pwd)/scripts"
host_maps_dir="$(pwd)/maps"
mkdir -p "$host_ros_bags_dir" "$host_catkin_ws_dir" "$host_utils_dir" "$host_maps_dir"

# Show config
echo "==============================="
echo " Parameters in Use"
echo "==============================="
echo "ROS_MASTER_URI:           $ros_master_uri"
echo "ROS_IP:                   $ros_ip"
echo "USE_ODOM:                 $use_odom"
echo "USE_IMU:                  $use_imu"
echo "NAVIGATE:                 $navigate"
echo "SLAM_ALGORITHM:           $slam_algorithm"
echo "==============================="

# Run
echo "Running ${container_image} docker image..."
docker run \
  -it \
  --name "$container_name" \
  --rm \
  --net host \
  --privileged \
  -e ROS_MASTER_URI="$ros_master_uri" \
  -e ROS_IP="$ros_ip" \
  -e USE_ODOM="$use_odom" \
  -e USE_IMU="$use_imu" \
  -e SLAM_ALGORITHM="$slam_algorithm" \
  -e NAVIGATE="$navigate" \
  -v "${host_catkin_ws_dir}:/home/go1/catkin_ws/src" \
  -v "${host_utils_dir}:/home/go1/scripts" \
  -v "${host_maps_dir}:/home/go1/maps" \
  "${container_image}:latest"

echo "Done."
