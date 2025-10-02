#!/bin/bash

# Delete the deployments and services
kubectl delete -f video-streaming-app-streamer.yml
kubectl delete -f video-streaming-app-receiver.yml
kubectl delete -f config.yml

echo "All deployments and services have been removed."

