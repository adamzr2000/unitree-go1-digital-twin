#!/bin/bash

# Assemble docker image. 
echo 'Building go1 docker image.'
sudo docker build . -t go1-base
