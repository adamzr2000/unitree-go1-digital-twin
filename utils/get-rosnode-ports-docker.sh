#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <node_name1> [node_name2] ... [-o <output_file>]"
    exit 1
fi

OUTPUT_FILE="output.csv"

# Parse optional parameters
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_FILE="$2"
            shift
            ;;
        *)
            NODE_NAMES+=("$1")
            ;;
    esac
    shift
done

if [ "${#NODE_NAMES[@]}" -eq 0 ]; then
    echo "Usage: $0 <node_name1> [node_name2] ... [-o <output_file>]"
    exit 1
fi

CONTAINER_NAME="roscore-edge"

# Write the CSV header to the output file
echo "Node,Port" > $OUTPUT_FILE

# Loop through each node name
for NODE_NAME in "${NODE_NAMES[@]}"; do
    # Run the rosnode info command inside the Docker container and store the output
    OUTPUT=$(docker exec -it $CONTAINER_NAME sh -c ". /opt/ros/noetic/setup.sh && rosnode info $NODE_NAME")

    # Extract the port number using grep and awk
    PORT=$(echo "$OUTPUT" | grep "contacting node" | awk -F':' '{print $3}' | awk -F'/' '{print $1}')

    if [ -n "$PORT" ]; then
        echo "$NODE_NAME,$PORT" >> $OUTPUT_FILE
    else
        echo "$NODE_NAME,Not found" >> $OUTPUT_FILE
    fi
done

echo "Node and port information has been saved to $OUTPUT_FILE"

