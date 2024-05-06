#!/bin/bash

#Assemble docker image. 
echo 'Building go1-gesture-control docker image.'
sudo docker build . -t go1-gesture-control
