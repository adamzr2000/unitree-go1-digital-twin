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

# Upload service graph
curl -X POST -F "file=@$SERVICE_GRAPH_YAML" http://localhost:32006/upload/
