#!/bin/bash

# Source ROS and workspace setup scripts
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

# Read env vars with defaults
slam_algorithm=${SLAM_ALGORITHM:-"hector"}
use_odom=${USE_ODOM:-"false"}
use_imu=${USE_IMU:-"false"}
navigate=${NAVIGATE:-"false"}     
map_file=${MAP_FILE:-"home/go1/maps/5tonic-map.yaml"}

echo "Selected SLAM Algorithm : $slam_algorithm"
echo "Use odom               : $use_odom"
echo "Use imu                : $use_imu"
echo "Navigate mode          : $navigate"

# Small helper for boolean-ish env vars
 Helper: interpret boolean-ish env vars
is_true() {
  case "${1,,}" in
    true|1|yes|y|on) return 0 ;;
    *)               return 1 ;;
  esac
}

if is_true "$navigate"; then
  # Pick the package that owns navigate.launch
  if [[ "${slam_algorithm,,}" == "cartographer" ]]; then
    nav_pkg="go1_navigation_cartographer"
  else
    nav_pkg="unitree_navigation"
  fi
  echo "Navigation package      : $nav_pkg"

  # Validate MAP_FILE (must be absolute and exist)
  if [[ -z "$map_file" ]]; then
    echo "ERROR: MAP_FILE is required when NAVIGATE=true." >&2
    exit 1
  fi
  if [[ $map_file != /* ]]; then
    echo "ERROR: MAP_FILE must be an absolute path (starts with '/'). Got: $map_file" >&2
    exit 1
  fi

  echo "Using map file          : $map_file"

  # Launch navigation (rviz off here; tweak if needed)
  exec roslaunch "$nav_pkg" navigate.launch rviz:=false map_file:="$map_file" --wait

else
  case "${slam_algorithm,,}" in
    gmapping)
      exec roslaunch unitree_navigation slam.launch rviz:=false algorithm:=gmapping --wait
      ;;
    hector)
      exec roslaunch unitree_navigation slam.launch rviz:=false algorithm:=hector --wait
      ;;
    cartographer)
      exec roslaunch go1_navigation_cartographer slam.launch \
        rviz:=false use_odom:="$use_odom" use_imu:="$use_imu" --wait
      ;;
    *)
      echo "Invalid SLAM algorithm specified: $slam_algorithm"
      echo "Using default SLAM algorithm 'hector'."
      exec roslaunch unitree_navigation slam.launch rviz:=true algorithm:=hector --wait
      ;;
  esac
fi