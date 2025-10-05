#!/bin/bash

container_image="cpu-stress"

docker run \
    -d \
    --rm \
    -p 6667:5000 \
    --name $container_image \
    --privileged \
    -v "$(pwd)/app":/app \
    $container_image:latest