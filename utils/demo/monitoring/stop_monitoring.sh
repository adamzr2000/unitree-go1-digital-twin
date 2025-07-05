#!/bin/bash

echo "Stopping mon-kafka-producer Docker container..."
docker ps -q --filter "name=mon-kafka-producer" | xargs -r docker kill

echo "Stopping mon-topology-updater Docker container..."
docker ps -q --filter "name=mon-topology-updater" | xargs -r docker kill

echo "Stopping mon-kafka-to-influx Docker container..."
docker ps -q --filter "name=mon-kafka-to-influx" | xargs -r docker kill

echo "Stopping edge stack..."
EDGE_SCRIPT_DIR="$(dirname "$0")/edge"
cd "$EDGE_SCRIPT_DIR" || { echo "Failed to enter $EDGE_SCRIPT_DIR. Exiting."; exit 1; }
./stop_edge.sh

echo "Monitoring stopped."
