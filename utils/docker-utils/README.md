## Edge
```bash
docker swarm init --advertise-addr 10.5.1.21

docker network create --driver=overlay --attachable digital-twin-service
```

## Robot
```bash
docker swarm join --advertise-addr 10.11.15.132 --token <token> 10.5.1.21:2377
```

## Edge
```bash
docker run -it --rm --name alpine1 --network digital-twin-service alpine sh
```

## Robot
```bash
docker run -it --rm --name alpine2 --network digital-twin-service alpine sh
```