#!/bin/bash

# Defaults
CUSTOM_NAME=""
POSITIONAL_ARGS=()

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --script)
            SCRIPT_NAME="$2"
            shift 2
            ;;
        --file)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --name)
            CUSTOM_NAME="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 --script <script_name.py> --file <config_file.json> [--name <container_name>]"
            exit 1
            ;;
    esac
done

# Validate required arguments
if [[ -z "$SCRIPT_NAME" || -z "$CONFIG_FILE" ]]; then
    echo "Missing required arguments."
    echo "Usage: $0 --script <script_name.py> --file <config_file.json> [--name <container_name>]"
    exit 1
fi

# Set container name
CONTAINER_NAME="${CUSTOM_NAME:-${SCRIPT_NAME%.py}}"

echo "Running monitoring agent: $SCRIPT_NAME with config: $CONFIG_FILE in container: $CONTAINER_NAME"

docker run -d --rm \
    --name "$CONTAINER_NAME" \
    --net host \
    -v "./config:/config" \
    -v "./app:/app/" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    monitoring-agent:latest \
    "$SCRIPT_NAME" --file "/config/$CONFIG_FILE"

