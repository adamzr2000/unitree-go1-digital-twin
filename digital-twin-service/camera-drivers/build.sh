#!/bin/bash

# Assemble docker image. 
echo 'Building astra-camera docker image.'
sudo docker build . -t astra-camera
