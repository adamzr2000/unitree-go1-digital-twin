#!/bin/bash
echo "Starting ROS container with role: $ROS_ROLE"

case "$ROS_ROLE" in
  roscore)
    echo "Launching roscore..."
    exec roscore
    ;;
  publisher)
    echo "Launching ROS publisher with:"
    echo "  Topic: $ROS_TOPIC"
    echo "  Rate: $ROS_RATE"
    echo "  Message: $ROS_MESSAGE"
    
    exec python3 -u dummy_publisher.py \
      --topic "${ROS_TOPIC:-chatter}" \
      --rate "${ROS_RATE:-2.0}" \
      --message "${ROS_MESSAGE:-Hello from consumer ROS node}"
    ;;
  subscriber)
    echo "Launching ROS subscriber..."
    exec python3 -u dummy_subscriber.py
    ;;
  *)
    echo "Invalid or missing ROS_ROLE. Available options: roscore, publisher, subscriber."
    exit 1
    ;;
esac
