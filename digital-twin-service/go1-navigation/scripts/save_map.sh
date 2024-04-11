#!/bin/bash

# Prompt user for map name
read -p "Enter the name of the map: " map_name

# Check if the input is empty
if [ -z "$map_name" ]; then
    echo "Map name cannot be empty. Exiting."
    exit 1
fi

# Run map_saver with the provided map name
rosrun map_server map_saver -f /home/go1/maps/$map_name

