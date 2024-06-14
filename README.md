# Digital Twin Application @ DESIRE6G Project

<div align="center">

[![Static Badge](https://img.shields.io/badge/Latest_Release-dev-orange)](https://github.com/adamzr2000/unitree-go1-digital-twin/)
[![Static Badge](https://img.shields.io/badge/K3s-v1.28.8%2Bk3s1-blue)](https://github.com/k3s-io/k3s/releases/tag/v1.28.8%2Bk3s1)
[![Static Badge](https://img.shields.io/badge/Docker-v25.0.3-blue)](https://github.com/docker)

</div>


This repository comprises the code developed and integrated in [DESIRE6G](https://desire6g.eu/) project
in order to implement the Digital Twin use case.

Currently, this use case uses the [Unitree Go1](https://unitree-docs.readthedocs.io/en/latest/get_started/Go1_Edu.html)
quadruped robot and the [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) robot
simulator.

## Version
 - 0.1

## Requirements
 - Ubuntu 20.04/22.04 Operating System 
 - Docker
 - Docker Compose
 - NVIDIA GPU (RTX 2070 or higher)
 - NVIDIA GPU Driver (recommended version 525.85 or higher)
 - NVIDIA Container Toolkit
 - NVIDIA Omniverse 
 - Kubernetes Distribution: K3s v1.28.8+k3s1

## Getting Started
 ### Digital Twin service description
 The Digital Twin [service](./digital-twin-service/) is composed of 8 different modules:
 
 - `ROS Master`: The ROS Master provides naming and registration services to the rest of the modules in the Digital Twin service (detailed info [here](./digital-twin-service/ros-master/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available
 
 - `Go1 Base`: Enables interaction with the robot stack (integrated as a native application with a proprietary SDK) using [Robot Operating System (ROS)](https://www.ros.org/) (detailed info [here](./digital-twin-service/go1-base/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available
 
 - `Gesture Contol App`: Provides gesture control capabilities, utilizing a camera input to translate gestures into velocity commands for the robot. (detailed info [here](./digital-twin-service/gesture-control-app/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available
 
 - `Digital Replica`: Provides the virtual object that replicates the behavior of the Unitree Go1 quadruped robot (detailed info [here](./digital-twin-service/digital-replica/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available
 
 - `Lidar Drivers`: Provides the [RPLIDAR A3](https://www.slamtec.ai/product/slamtec-rplidar-a3/) drivers for mapping and navigation purposes (detailed info [here](./digital-twin-service/lidar-drivers/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

 - `Camera Drivers`: Provides the [Depth Camera Astra S | Orbbec 3D](https://shop.orbbec3d.com/Astra-S) drivers (detailed info [here](./digital-twin-service/camera-drivers/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

 - `Go1 Navigation`: Provides algorithms for simultaneous localization and mapping (SLAM) that enable the creation of maps for indoor environments and facilitate autonomous navigation (detailed info [here](./digital-twin-service/go1-navigation/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

 - `Rviz VNC`: Provides a VNC container with a ROS image, enabling visualization in [RViz](https://wiki.ros.org/rviz) of the SLAM capabilities. This showcases how the map of the environment is being created in real-time (detailed info [here](./digital-twin-service/rviz-vnc/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

 ### Install Requirements
 - Install Docker Engine (tutorial [here](https://docs.docker.com/engine/install/ubuntu/))
 - Install Docker Compose (tutorial [here](https://docs.docker.com/compose/install/))
 - Install NVIDIA GPU Driver (tutorial [here](https://www.nvidia.com/en-us/drivers/unix/))
 - Install NVIDIA Container Toolkit (tutorial [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))
 - Install NVIDIA Omniverse (tutorial [here](https://docs.omniverse.nvidia.com/digital-twins/latest/installation-guide.html))
 - Install K3s Kubernetes distribution (tutorial [here](https://docs.k3s.io/installation))
 - Clone this git repo: `git clone https://github.com/adamzr2000/unitree-go1-digital-twin.git`
 
 ### Run Digital Twin service
 - The scenarios [folder](./scenarios/) is composed of different deployment options for the Digital Twin service.
    - end-to-end-docker scenarios (tutorial [here](./scenarios/end-to-end-docker-swarm/)) ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available   
    - end-to-end-kubernetes scenarios (tutorial [here](./scenarios/end-to-end-kubernetes/)) ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available   
 
## DISCLAIMER
The modules provided in this repository are distributed in the hope that they
will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
See the License section for more details.

## License
The modules provided in this repository are distributed under a license.
The full license agreement can be found in the file LICENSE
in this distribution.
This software may not be copied, modified, sold or distributed other than
expressed in the named license agreement.
