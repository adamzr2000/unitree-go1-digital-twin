# Unitree Go1 Digital Twin service deployment on end-to-end scenario

## What is this?
...

## Pre-requisites

Each machine running the containers must have the following components installed: 
* k3s

Also, in absence of a centralized registry, the Docker images composing the service need to be already present in the machine (therefore you need to build each service image manually for every host machine before running the stack, and then import them into the k3s registry). 

Execute the following command to import the docker images into your k3s registry:
```bash
docker save <docker-image-name> | sudo k3s ctr images import -
```

## Hosts configuration

This is the multi-host setup: 

* EDGE (10.5.98.101): `roscore-edge`, `digital-twin-app`, `gesture-control-app`, `go1-navigation`, `rviz-vnc`
* ROBOT (10.5.98.70): `lidar`, `go1-base`

Here is a diagram that represents visually the architecture of the scenario:

![E2E Scenario 5TONIC](../../images/e2e-scenario-kubernetes.svg)

## Cluster Installation

To effortlessly set up a fully-functional, single-node Kubernetes cluster, execute the following command:
```bash
curl -sfL https://get.k3s.io | sh -
```

## Cluster Configuration

