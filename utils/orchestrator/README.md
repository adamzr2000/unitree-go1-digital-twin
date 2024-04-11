# Unitree Go1 ROS Orchestrator.

## What is this?

This repository contains a basic Python Flask server designed to orchestrate the different modules of the Digital Twin service across multiple hosts using the [Docker SDK for Python](https://docker-py.readthedocs.io/en/stable/).

 ## Pre-requisites

Each machine running the containers must have the following components installed: 
* Docker

* The following python3 dependencies:
```bash
pip3 install -r requirements.txt
```

* Ensure your Docker daemon endpoint is configured to be accessible by following these steps:
1. Create a systemd drop-in directory for the docker service:

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d
```

2. Create a new override configuration file within the drop-in directory:

```bash
sudo nano /etc/systemd/system/docker.service.d/override.conf
```

3. Add the following content to `override.conf`:

```bash
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd -H fd:// -H tcp://0.0.0.0:2375
```

This configuration will override the default `ExecStart` directive for the Docker service and specify the desired options for the dockerd command.

4. Reload systemd to apply the changes:
```bash
sudo systemctl daemon-reload
```

5. Restart the Docker service to pick up the new configuration:
```bash
sudo systemctl restart docker
```
Also, in absence of a centralized registry, the Docker images composing the service need to be already present in the machine (therefore you need to build each service image manually for every host machine before running the stack). 

## Hosts configuration

This is the multi-host setup: 

* EDGE (10.5.98.101): `ros-master`, `digital-replica`, `gesture-control-app`, `go1-navigation`, `rviz-vnc`
* ROBOT (10.5.98.70): `lidar-drivers`, `go1-base`

> Note: the configuration can be adjusted by modifying the `DOCKER_CLIENTS` python dictionary, where the key represents the desired name for your host and the value is the docker client endpoint.

### How to use it?
1. Initiate docker swarm in the edge:
```bash
docker swarm init --advertise-addr 10.5.98.101
```

2. Create an overlay network:
```bash
docker network create --driver=overlay --subnet=10.0.0.0/16 --ip-range 10.0.1.0/24 --gateway 10.0.0.1 --attachable digital-twin-service
```

3. Join the swarm with the robot:
```bash
docker swarm join --advertise-addr 10.5.98.70 --token <swarm-token> 10.5.98.101:2377
```

3. Run the orchestrator:
```bash
./start_app.sh
```

## API Endpoints

### Deploy Services

Deploys predefined Docker containers on edge and robot clients.

**Request:**

```bash
curl -X POST http://localhost:9999/deploy/ -H "Content-Type: application/json"
```

### Terminate Services

Terminates all running services by stopping and removing all containers managed by the defined Docker clients.

**Request:**

```bash
curl -X POST http://localhost:9999/terminate/ -H "Content-Type: application/json"
```

### Start Specific Containers

Starts specific containers based on the names provided in the request.

**Request:**

```bash
curl -X POST http://localhost:9999/start/ -d '{"containerName": ["container1", "container2"]}' -H "Content-Type: application/json"
```

Replace "container1" and "container2" with the actual names of the containers you wish to start.

### Stop Specific Containers

Stop specific containers based on the names provided in the request.

**Request:**

```bash
curl -X POST http://localhost:9999/stop/ -d '{"containerName": ["container1", "container2"]}' -H "Content-Type: application/json"
```

Replace "container1" and "container2" with the actual names of the containers you wish to stop.




