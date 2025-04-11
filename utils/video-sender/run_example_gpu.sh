#!/bin/bash

# Check if server URL is provided
if [ -z "$1" ] || [ "$1" != "--server_url" ] || [ -z "$2" ]; then
    echo "Usage: $0 --server_url <server_url>"
echo "Example: $0 --server_url http://10.5.1.21:8888/upload_frame"
    exit 1
fi

# Set server URL from argument
SERVER_URL="$2"

host_app_dir="$(pwd)/app"

# Run the Docker container and start the client script
docker run \
    -it \
    --name video-sender \
    --hostname video-sender \
    --rm \
    --net host \
    --privileged \
    --gpus all \
    --group-add video \
    -e SERVER_URL="$SERVER_URL" \
    -v "$host_app_dir":/home/go1/app \
    video-sender:latest \
    python3 client.py --server_url="$SERVER_URL"

echo "Done."
