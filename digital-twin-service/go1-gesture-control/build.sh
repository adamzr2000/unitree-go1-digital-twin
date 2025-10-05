#!/bin/bash

container_image="go1-gesture-control"

echo "Building $container_image docker image."

sudo docker build . -t $container_image
