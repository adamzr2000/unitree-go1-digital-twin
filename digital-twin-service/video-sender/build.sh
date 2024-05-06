#!/bin/bash

#Assemble docker image. 
echo 'Building video-sender docker image.'
docker build . -t video-sender
