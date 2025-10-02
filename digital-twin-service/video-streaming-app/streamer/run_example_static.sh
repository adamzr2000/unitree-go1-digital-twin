#!/bin/bash

# Default values
RECEIVER_IP="127.0.0.1"
RECEIVER_PORT="5554"

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --receiver-ip)
      RECEIVER_IP="$2"
      shift 2;;
    --receiver-port)
      RECEIVER_PORT="$2"
      shift 2;;
    --)
      shift
      break;;
    *)
      echo "Unknown option: $1"
      exit 1;;
  esac
done

# Run the Docker container
docker run --rm -it \
  --privileged \
  --name video-streamer \
  -v ./app/backup/test_static_stream.py:/app/video_streamer.py \
  -v ./app/backup/images:/images \
  -e RECEIVER_IP="$RECEIVER_IP" \
  -e RECEIVER_PORT="$RECEIVER_PORT" \
  -e IMAGE_FOLDER="/images" \
  video-streamer