# ROS Master

## What is it?

This repository contains the files to lauch a ros master in docker container. 

The ROS Master provides naming and registration services to the rest of the nodes in the ROS system. It tracks publishers and subscribers to topics as well as services. The role of the Master is to enable individual ROS nodes to locate one another. Once these nodes have located each other they communicate with each other peer-to-peer.

## What does this container do?

This container runs `roscore`.

## Run it?

### Dependencies:

The ROS master has no dependencies.

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `go1-roscore` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Docker run example
In this folder we also provide a docker run example. 

To run the ROS Master:
```bash
./run_example.sh
```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `roscore-edge` container up and running.