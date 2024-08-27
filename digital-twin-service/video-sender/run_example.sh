#!/bin/bash

# Define server URL for the client to connect to
SERVER_URL="http://10.5.1.21:8888/upload_frame"
# SERVER_URL="http://127.0.0.1:8888/upload_frame"

# Run the Docker container and start the client script
docker run \
    -it \
    --name video-sender \
    --hostname video-sender \
    --rm \
    --net host \
    --privileged \
    --group-add video \
    -e SERVER_URL="$SERVER_URL" \
    video-sender:latest \
    python3 client.py --server_url="$SERVER_URL"

echo "Done."
