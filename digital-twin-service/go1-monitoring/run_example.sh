#!/bin/bash
set -euo pipefail

# Defaults
ros_master_uri="http://localhost:11311"
ros_ip="127.0.0.1"
topics="/scan,/joint_states,/odometry/filtered,/go1_controller/cmd_vel"
influxdb_url="http://localhost:8086"
influxdb_token="desire6g2024;"
influxdb_org="desire6g"
influxdb_bucket="monitoring"
window_size="50"
wait_for_master="true"
node_role="auto"

container_image="go1-monitoring"
container_name="go1-monitoring"

usage() {
  cat <<EOF
Usage: $0 [--ros-master-uri URI] [--ros-ip IP] [--topics "t1,t2,..."]
          [--influxdb-url URL] [--influxdb-token TOKEN]
          [--influxdb-org ORG] [--influxdb-bucket BUCKET]
          [--window-size N] [--wait-for-master true|false]
          [--node-role edge|robot|auto]
EOF
}

validate_ip() {
  local ip=$1
  if [[ ! $ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    echo "Invalid IP address format: $ip" >&2
    exit 1
  fi
}

validate_uri() {
  local uri=$1
  if [[ ! $uri =~ ^http://[^:]+:[0-9]+$ ]]; then
    echo "Invalid ROS_MASTER_URI (expected http://host:port): $uri" >&2
    exit 1
  fi
}

mask() {
  local s=${1:-}
  local n=${#s}
  if (( n == 0 )); then echo "<empty>"; return; fi
  if (( n <= 6 )); then printf '%*s\n' "$n" '' | tr ' ' '*'; return; fi
  echo "${s:0:3}***${s: -3}"
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-master-uri) ros_master_uri="$2"; shift 2;;
    --ros-ip)         ros_ip="$2";         shift 2;;
    --topics)         topics="$2";         shift 2;;
    --influxdb-url)   influxdb_url="$2";   shift 2;;
    --influxdb-token) influxdb_token="$2"; shift 2;;
    --influxdb-org)   influxdb_org="$2";   shift 2;;
    --influxdb-bucket) influxdb_bucket="$2"; shift 2;;
    --window-size)    window_size="$2";    shift 2;;
    --wait-for-master) wait_for_master="$2"; shift 2;;
    --node-role)      node_role="$2";      shift 2;;
    -h|--help)        usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage; exit 1;;
  esac
done

# Validate
validate_ip "$ros_ip"
validate_uri "$ros_master_uri"

# Normalize topics (strip spaces if any)
topics="${topics// /}"

# Display parameters in use
echo "==============================="
echo " Parameters in Use             "
echo "==============================="
echo "ROS_MASTER_URI : $ros_master_uri"
echo "ROS_IP         : $ros_ip"
echo "ROS_HOSTNAME   : $ros_ip"
echo "TOPICS         : $topics"
echo "INFLUXDB_URL   : $influxdb_url"
echo "INFLUXDB_TOKEN : $(mask "$influxdb_token")"
echo "INFLUXDB_ORG   : $influxdb_org"
echo "INFLUXDB_BUCKET: $influxdb_bucket"
echo "WINDOW_SIZE    : $window_size"
echo "WAIT_FOR_MASTER: $wait_for_master"
echo "NODE_ROLE      : $node_role"
echo "==============================="

# Host directory (optional mount)
host_app_dir="$(pwd)/app"

echo "Running $container_name Docker container…"
docker run \
  -it \
  --name $container_name \
  --rm \
  --net host \
  -e ROS_MASTER_URI="$ros_master_uri" \
  -e ROS_IP="$ros_ip" \
  -e ROS_HOSTNAME="$ros_ip" \
  -e TOPICS="$topics" \
  -e INFLUXDB_URL="$influxdb_url" \
  -e INFLUXDB_TOKEN="$influxdb_token" \
  -e INFLUXDB_ORG="$influxdb_org" \
  -e INFLUXDB_BUCKET="$influxdb_bucket" \
  -e WAIT_FOR_MASTER="$wait_for_master" \
  -e NODE_ROLE="$node_role" \
  -v "${host_app_dir}:/home/go1/app" \
  $container_image:latest