#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 <service_id>"
    echo "Example: $0 11"
    exit 1
}

# Check if the user provided the service ID
if [ "$#" -ne 1 ]; then
    echo "Error: Service ID not provided."
    usage
fi

SERVICE_ID=$1

# Delete service
curl -X DELETE "http://localhost:32008/main_id/$SERVICE_ID"
