#!/bin/bash

# Assemble docker image. 
echo 'Building d435i-camera docker image.'
sudo docker build . -t d435i-camera
