#!/bin/bash

# Check if the deployment name is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <deployment-name>"
  exit 1
fi

# Get the deployment name from the first argument
DEPLOYMENT_NAME=$1

# Get the pod name associated with the deployment
POD_NAME=$(kubectl get pods --no-headers -o custom-columns=":metadata.name" | grep "^${DEPLOYMENT_NAME}-")

# Check if a pod name was found
if [ -z "$POD_NAME" ]; then
  echo "No pod found for deployment: $DEPLOYMENT_NAME"
  echo "Available deployments:"
  kubectl get deploy
  exit 1
fi

# Print the pod name found (for debugging purposes)
echo "Found pod: $POD_NAME"

# Execute a bash shell in the pod
kubectl exec -it $POD_NAME -- /bin/bash

