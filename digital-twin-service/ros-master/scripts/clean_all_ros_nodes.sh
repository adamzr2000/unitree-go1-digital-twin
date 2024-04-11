#!/bin/bash

# Get a list of all running ROS nodes
nodes=$(rosnode list)

# Loop through the nodes and kill them if they're not rosout
for node in $nodes; do
    if [ "$node" != "/rosout" ]; then
        echo "Killing node: $node"
        rosnode kill "$node"
    fi
done
