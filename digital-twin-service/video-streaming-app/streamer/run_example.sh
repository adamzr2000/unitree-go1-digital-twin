#!/bin/bash

# Default values
WIDTH="1280"
HEIGHT="720"
FRAMERATE="30"
RECEIVER_IP="127.0.0.1"
RECEIVER_PORT="5554"
DEVICE="/dev/video0"
USE_D435I="false"
USE_H264="false"
BITRATE="2000"

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --width)
      WIDTH="$2"
      shift 2;;
    --height)
      HEIGHT="$2"
      shift 2;;
    --framerate)
      FRAMERATE="$2"
      shift 2;;
    --receiver-ip)
      RECEIVER_IP="$2"
      shift 2;;
    --receiver-port)
      RECEIVER_PORT="$2"
      shift 2;;
    --device)
      DEVICE="$2"
      shift 2;;
    --use-d435i)
      USE_D435I="true"
      shift ;;
    --use-h264)
      USE_H264="true"
      shift ;;      
    --bitrate)
      BITRATE="$2"
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
docker run --rm -d \
  --privileged \
  --name video-streamer \
  --network host \
  -v ./app:/app/ \
  -e WIDTH="$WIDTH" \
  -e HEIGHT="$HEIGHT" \
  -e FRAMERATE="$FRAMERATE" \
  -e RECEIVER_IP="$RECEIVER_IP" \
  -e RECEIVER_PORT="$RECEIVER_PORT" \
  -e DEVICE="$DEVICE" \
  -e USE_D435I="$USE_D435I" \
  -e USE_H264="$USE_H264" \
  -e BITRATE="$BITRATE" \
  video-streamer
