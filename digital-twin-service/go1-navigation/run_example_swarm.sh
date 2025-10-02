#!/bin/bash
set -euo pipefail

# Defaults
ros_master_uri="http://ros-master:11311"
container_image="go1-navigation"
container_name="go1-navigation"
use_odom="true"
use_imu="false"
slam_algorithm="cartographer" # cartographer, hector, gmapping

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --use-odom               Enable odometry input for Cartographer (sets CARTOGRAPHER_USE_ODOM=true)
  --use-imu                Enable IMU input for Cartographer (sets CARTOGRAPHER_USE_IMU=true)
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
    --use-odom)
      use_odom="true"; shift ;;
    --use-imu)
      use_imu="true"; shift ;;
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


# Prepare host bind mounts (create if missing)
host_catkin_ws_dir="$(pwd)/catkin_ws/src"
host_utils_dir="$(pwd)/scripts"
host_maps_dir="$(pwd)/maps"
mkdir -p "$host_catkin_ws_dir" "$host_utils_dir" "$host_maps_dir"

# Show config
echo "==============================="
echo " Parameters in Use"
echo "==============================="
echo "USE_ODOM:                 $use_odom"
echo "USE_IMU:                  $use_imu"
echo "==============================="

# Run
echo "Running ${container_image} docker image..."
docker run \
  -it \
  --hostname "$container_name" \
  --name "$container_name" \
  --rm \
  --net digital-twin-service \
  --privileged \
  -e ROS_MASTER_URI="$ros_master_uri" \
  -e ROS_HOSTNAME="$container_name" \
  -e USE_ODOM="$use_odom" \
  -e USE_IMU="$use_imu" \
  -e SLAM_ALGORITHM="$slam_algorithm" \
  -v "${host_catkin_ws_dir}:/home/go1/catkin_ws/src" \
  -v "${host_utils_dir}:/home/go1/scripts" \
  -v "${host_maps_dir}:/home/go1/maps" \
  "${container_image}:latest"

echo "Done."
