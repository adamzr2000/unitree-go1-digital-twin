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

# Deploy service
curl -X POST -H "Content-Type: application/json" \
    -d "{\"json_data\": {\"type\": \"service_graph\", \"name\": \"$SERVICE_GRAPH_YAML\", \"site_id\": \"desire6g-site\"}}" \
    http://localhost:32008/main_id/ | jq
