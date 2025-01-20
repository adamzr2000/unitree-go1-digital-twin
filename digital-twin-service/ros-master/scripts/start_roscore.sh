#!/bin/bash

# Start roscore in the background using screen
screen -dmS roscore roscore

# Wait for roscore to start
sleep 3

# Set enable_statistics based on the environment variable ENABLE_STATISTIC
ENABLE_STATISTICS=${ENABLE_STATISTICS:-false}
rosparam set enable_statistics "$ENABLE_STATISTICS"

# Print the value of enable_statistics for debugging
echo "enable_statistics parameter: $ENABLE_STATISTICS"

# Attach to the roscore screen session
screen -x roscore
