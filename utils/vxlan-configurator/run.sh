#!/bin/bash

container_image="vxlan-configurator"

docker run \
    -d \
    --rm \
    --name $container_image \
    --network=host \
    --cap-add=NET_ADMIN \
    --cap-add=NET_RAW \
    --privileged \
    -v "$(pwd)/app":/app \
    $container_image:latest