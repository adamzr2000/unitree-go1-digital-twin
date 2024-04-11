#!/bin/bash

#Assemble docker image. 
echo 'Building go1-gesture-control docker image.'
docker build . -t go1-gesture-control
