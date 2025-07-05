#!/bin/bash

# Check if a node name is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <node-name>"
  exit 1
fi

NODE_NAME=$1

# Confirm the node exists
if ! kubectl get node "$NODE_NAME" &>/dev/null; then
  echo "Error: Node '$NODE_NAME' does not exist."
  exit 1
fi

# Delete the node directly
echo "Deleting node '$NODE_NAME' from the cluster..."
kubectl delete node "$NODE_NAME"

if [ $? -eq 0 ]; then
  echo "Node '$NODE_NAME' has been successfully deleted from the cluster."
else
  echo "Error: Failed to delete node '$NODE_NAME'."
  exit 1
fi
