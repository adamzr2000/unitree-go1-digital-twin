#!/bin/bash

container_image="dummy-ros-app"

echo "Building $container_image docker image."

docker build -t $container_image .

