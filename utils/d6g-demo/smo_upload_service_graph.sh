#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 <service_graph_yaml>"
    echo "Example: $0 demo_nsd.sg.yml"
    exit 1
}

# Check if the user provided the service graph YAML filename
if [ "$#" -ne 1 ]; then
    echo "Error: Service graph YAML filename not provided."
    usage
fi

SERVICE_GRAPH_YAML=$1

# Construct the full path to the service graph YAML file
SERVICE_GRAPH_PATH="descriptors/$SERVICE_GRAPH_YAML"

# Check if the specified YAML file exists in the descriptors directory
if [ ! -f "$SERVICE_GRAPH_PATH" ]; then
    echo "Error: Service graph YAML file '$SERVICE_GRAPH_PATH' not found."
    exit 1
fi

# Upload service graph
curl -X POST -F "file=@$SERVICE_GRAPH_PATH" http://localhost:32006/upload/
