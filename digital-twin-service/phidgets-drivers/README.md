# Phidgets ROS Drivers. 

## What is it?

This repository contains files for basic device management of the [PhidgetSpatial Precision 3/3/3](https://www.phidgets.com/?prodid=1205&srsltid=AfmBOoq5wnBJ4TBEj7GeISFI8TD6qQ9OCh0vuECc95HaaD-B1GDWS0ut) with ROS.

The data captured by the PhidgetSpatial Precision 3/3/3 is published on the `/imu/data_raw` ROS topic.

For further information, refer to the documentation available at: 
- [phidgets ros package](https://wiki.ros.org/phidgets_imu)
- [phidgets source code](https://github.com/ros-drivers/phidgets_drivers/tree/noetic/phidgets_spatial)

## What does this container do?

This container runs the ROS drivers for the PhidgetSpatial Precision 3/3/3.

## Run it?

### Dependencies:

The LiDAR ROS Drivers depends on:
  - ros-master (tutorial [here](https://github.com/adamzr2000/unitree-go1-digital-twin/tree/main/digital-twin-service/ros-master))

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `phidgets-imu` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Then, you need to apply Phidgets rules to assign a fixed name to the device port.

After building the image, it's necessary to execute the following script:
```bash
./apply_phidgets_rules.sh
```

This script copies necessary [99-phidgets.rules](rules/99-phidgets.rules) to the `/etc/udev/rules.d/` directory, sets appropriate permissions, and records the changes. It ensures that the device port remains consistent for the `phidgets` name.

### Docker run example
In this folder we also provide a docker run example. 

> Note: ensure that the Precision 3/3/3 is connected via USB to your PC before testing this container.

To run the Precision 3/3/3 ROS Drivers:
```bash
./run_example.sh
```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `phidgets-imu` container up and running.