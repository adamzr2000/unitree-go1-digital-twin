#!/bin/bash

# Assemble docker image. 
echo 'Building go1 docker image.'

docker build --pull -t isaac-sim:2023.1.0-ubuntu22.04 --build-arg ISAACSIM_VERSION=2023.1.0 --file Dockerfile.2023.1.0-ubuntu22.04 .
