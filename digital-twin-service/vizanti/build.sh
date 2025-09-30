#!/bin/bash

container_image="vizanti"

echo "Building $container_image docker image."
sudo docker build . -t $container_image