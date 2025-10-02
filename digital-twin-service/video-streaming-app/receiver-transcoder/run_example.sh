#!/bin/bash

# Default values
RECEIVER_PORT="5554"
WIDTH="1280"
HEIGHT="720"
BITRATE="2000"
SPEED_PRESET="ultrafast"
SRT_IP="127.0.0.1"
SRT_PORT="8890"
STREAM_NAME="test_stream"
ENABLE_MONITORING="true"
USE_H264="false"

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --receiver-port)
      RECEIVER_PORT="$2"
      shift 2;;
    --width)
      WIDTH="$2"
      shift 2;;
    --height)
      HEIGHT="$2"
      shift 2;;
    --bitrate)
      BITRATE="$2"
      shift 2;;
    --speed-preset)
      SPEED_PRESET="$2"
      shift 2;;
    --srt-ip)
      SRT_IP="$2"
      shift 2;;
    --srt-port)
      SRT_PORT="$2"
      shift 2;;
    --stream-name)
      STREAM_NAME="$2"
      shift 2;;
    --enable-monitoring)
      ENABLE_MONITORING="true"
      shift ;;
    --use-h264)
      USE_H264="true"
      shift ;;         
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
  --name video-receiver-transcoder \
  -p "${RECEIVER_PORT}:${RECEIVER_PORT}/udp" \
  -e RECEIVER_PORT="$RECEIVER_PORT" \
  -e WIDTH="$WIDTH" \
  -e HEIGHT="$HEIGHT" \
  -e BITRATE="$BITRATE" \
  -e SPEED_PRESET="$SPEED_PRESET" \
  -e SRT_IP="$SRT_IP" \
  -e SRT_PORT="$SRT_PORT" \
  -e STREAM_NAME="$STREAM_NAME" \
  -e ENABLE_MONITORING="$ENABLE_MONITORING" \
  -e USE_H264="$USE_H264" \
  -v ./app:/app/ \
  video-receiver-transcoder
