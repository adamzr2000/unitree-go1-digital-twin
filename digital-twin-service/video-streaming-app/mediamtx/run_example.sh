#!/bin/bash

# Default value
WEBRTC_ADDITIONAL_HOSTS="127.0.0.1"

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --webrtc-additional-hosts)
      WEBRTC_ADDITIONAL_HOSTS="$2"
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
  -p 8890:8890/udp \
  -p 8889:8889 \
  -p 8189:8189 \
  --name mediamtx \
  -e MTX_WEBRTCADDITIONALHOSTS="$WEBRTC_ADDITIONAL_HOSTS" \
  bluenviron/mediamtx:latest
