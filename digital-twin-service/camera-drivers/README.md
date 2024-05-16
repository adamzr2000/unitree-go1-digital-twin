# astra ROS Drivers. 

## What is it?

This repository contains files for basic device management of the Depth Camera Astra S with ROS.

For further information, refer to the documentation available at: [astra ros package](https://github.com/orbbec/ros_astra_camera)


## What does this container do?

This container runs the ROS drivers for the Depth Camera Astra S | Orbbec 3D

## Run it?

### Dependencies:

The astra ROS Drivers depends on:
  - ros-master (tutorial [here](../ros-master/))

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `astra-camera` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Then, you need to apply Astra rules to assign a fixed name to the device port.

After building the image, it's necessary to execute the following script:
```bash
./apply_astra_rules.sh
```

This script copies necessary [56-orbbec-usb.rules](rules/56-orbbec-usb.rules) to the `/etc/udev/rules.d/` directory, sets appropriate permissions, and records the changes. It ensures that the device port remains consistent for the `astra` name.

### Docker run example
In this folder we also provide a docker run example. 

> Note: ensure that the astra camera is connected via USB to your PC before testing this container.

To run the astra ROS Drivers:
```bash
./run_example.sh
```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `astra-camera` container up and running.

