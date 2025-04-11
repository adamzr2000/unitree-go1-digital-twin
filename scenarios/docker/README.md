# Unitree Go1 Digital Twin service deployment on end-to-end scenario

## What is this?

This repository contains the necessary files to run the Digital Twin stack deploying the containers in different hosts, using `Docker Compose` and the `Docker Swarm` mode. 

## Pre-requisites

Each machine running the containers must have the following components installed: 
* Docker
* Docker-compose 

Also, in absence of a centralized registry, the Docker images composing the service need to be already present in the machine (therefore you need to build each service image manually for every host machine before running the stack). 

## Hosts configuration

This is the multi-host setup: 

- **Master Node (EDGE)**: 
  - IP: `192.168.40.4`
  - Containers: `roscore-edge`, `digital-twin-app`, `gesture-control-app`, `go1-navigation`, `rviz-vnc`

- **Worker Node (ROBOT)**: 
  - IP: `192.168.40.70`
  - Containers: `lidar`, `go1-base`

Here is a diagram that represents visually the architecture of the scenario:

![E2E Scenario Docker](../../images/e2e-scenario-docker-swarm.png)

First configure the controller/master node (192.168.40.4 in our case) by initializing the Docker swarm like this: 
```bash
docker swarm init --advertise-addr 192.168.40.4
```

This command will be displayed upon success: 
```bash
docker swarm join --token <token> 192.168.40.4:2377
``` 
with the correct token.

Copy paste this command to the robot's Rpi to add as worker/slave node. 

From the master machine, you can see the nodes belonging to the Swarm using this: `docker node ls`

## Deployment

> Note: we have two separate docker compose files because the native docker swarm command doesn't support deploying containers with `device`, `privileged`, and `group_add: - video` options. These options are necessary for the `lidar-drivers` and for enabling camera input for the `gesture-control-app`. For automated deployment across multiple docker hosts, please refer to the [utils/ros-orchestrator](./../../utils/ros-orchestrator) directory. This includes a simple Flask server with a REST API to orchestrate the different modules of the Digital Twin service across multiple hosts using the [Docker SDK for Python](https://docker-py.readthedocs.io/en/stable/).

Run this command in the `edge` node: 
```bash
docker compose -f docker-compose-edge.yml up -d
```

Run this command in the `robot` node: 
```bash
docker compose -f docker-compose-robot.yml up -d
```

To remove the stack, use: 
```bash
docker compose -f docker-compose-edge.yml down
```

```bash
docker compose -f docker-compose-robot.yml down
```





