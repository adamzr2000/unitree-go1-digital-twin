# Unitree Go1 Digital Twin. 

## What is this?

This repository contains the necessary files to run the Unitree Go1 Digital Twin replica. 

The digital twin monitors the state of a Unitree Go1 using ROS topics, specifically the `/joint_states` topic using the ROS interface for Isaac Sim. 

The information recovered from this topic is then used to actuate a 3D model of an Unitree Go1 included with Isaac Sim.

## What does this container do?

This container runs the robotics simulation platform NVIDIA Isaac Sim.

## Run it?

### Dependencies:

The Unitree Go1 Digital Twin depend on:

- ros-master (tutorial [here](../ros-master/)).
- go1-base (tutorial [here](../go1-base)).
 
> Note: be sure that all the dependencies are running before you run the Unitree Go1 Digital Twin container

### First you will need to build the container. 

Before getting started, ensure that the system has the latest [NVIDIA Driver](https://www.nvidia.com/en-us/drivers/unix/) and the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker) installed.

To build the Isaac Sim Docker image, please follow the detailed instructions provided in the official documentation at [NVIDIA-Omniverse/IsaacSim-dockerfiles](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html).

Use your [NGC Account](https://docs.nvidia.com/ngc/ngc-overview/index.html#registering-activating-ngc-account) to get access to the [Isaac Sim Container](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim) and generate your [NGC API Key](https://docs.nvidia.com/ngc/ngc-overview/index.html#generating-api-key).

Build the image running the following commands:
```bash
docker login nvcr.io

./build.sh
```

### This will build the `isaac-sim:2023.1.0-ubuntu22.04` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Docker run example

In this folder we provide two docker run examples:

1. To run the Unitree Go1 Digital Twin and visualize the simulation using the NVIDIA Omniverse Platform (recommended):
```bash
./run_example_omniverse_client.sh
```


2. To run the Unitree Go1 Digital Twin and visualize the simulation using the web browser (WebRTC):
```bash
./run_example_web_client.sh
```

Verify that the container is up and running:
```bash
docker image ls
```
In the output you should be able to see the `go1-digital-twin` container up and running


### How to use it?

If you opt for the first approach:
- Launch the `Omniverse Platform`
- Access the `Library` and launch the `Streaming Client` application.
- Enter the IP address of the edge machine where the container is running
- Once the GUI is available, navigate to `Window -> Extensions` and activate the `ros_bridge` extension
- Open the `go1-warehouse-env.usd` scene and click the play button
- You should see the Unitree Go1 digital replica

If you choose the second option (WebRTC):
- Open the link below in your browser, replacing with the IP address of the host machine where the container is running:
```
http://<host-ip-address>:8211/streaming/webrtc-client?server=<host-ip-address>
```

