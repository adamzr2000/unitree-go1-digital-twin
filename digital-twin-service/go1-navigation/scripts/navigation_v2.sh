#!/bin/bash

# Define the directory path where maps are stored
maps_directory="/home/go1/maps/"

# List only .yaml files in the maps directory and number them
map_files=$(ls "$maps_directory" | grep '\.yaml$' | cat -n)

# Display available .yaml map files with numbers for selection
echo "Available .yaml map files:"
echo "$map_files"

# Prompt user to select a map file by number
read -p "Enter the number of the map file: " selected_number

# Check if input is a valid number
if ! [[ "$selected_number" =~ ^[0-9]+$ ]]; then
    echo "Invalid input. Please enter a valid number. Exiting."
    exit 1
fi

# Extract the selected map filename based on the entered number
selected_map=$(echo "$map_files" | awk -v num="$selected_number" '$1 == num {print $NF}')

# Check if the selected map file exists
if [ -z "$selected_map" ]; then
    echo "Invalid selection. Exiting."
    exit 1
fi

# Run roslaunch with the selected map file
roslaunch unitree_navigation navigation.launch rviz:=true map_file:=${maps_directory}${selected_map}