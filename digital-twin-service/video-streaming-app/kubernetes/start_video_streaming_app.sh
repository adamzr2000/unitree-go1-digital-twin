#!/bin/bash

# Apply the necessary YAML configurations
kubectl apply -f config.yml
kubectl apply -f video-streaming-app-receiver.yml

# Function to wait until a deployment is ready
wait_for_deployment() {
    local deployment_name=$1
    local namespace=$2
    echo "Waiting for deployment $deployment_name to be ready..."
    kubectl rollout status deployment/$deployment_name -n $namespace --timeout=5m
}

# Wait until both mediamtx-deployment and video-receiver-transcoder-deployment are ready
wait_for_deployment "mediamtx-deployment" "default"
wait_for_deployment "video-receiver-transcoder-deployment" "default"

# Apply the video-streaming-app-streamer.yml
kubectl apply -f video-streaming-app-streamer.yml

echo "Deployment and services are successfully applied and ready."

