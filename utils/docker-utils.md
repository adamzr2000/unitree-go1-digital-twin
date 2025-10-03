# Docker Swarm Multi-Node Connectivity Test

## Edge server node setup
```bash
docker swarm init --advertise-addr 10.5.1.21
docker network create --driver=overlay --attachable digital-twin-service
docker run -it --rm --name alpine1 --network digital-twin-service alpine sh
```

## Robot node join & test
```bash
docker swarm join --advertise-addr 10.11.15.132 --token <token> 10.5.1.21:2377
docker run -it --rm --name alpine2 --network digital-twin-service alpine sh
```