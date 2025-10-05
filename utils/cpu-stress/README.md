# VXLAN Configurator

1. Build the Docker image
```shell
./build.sh
```

2. Deploy
```shell
./run.sh
```

## Endpoints

### Health check
```shell
curl "http://localhost:6667/health" | jq
```
---
### Start stress (defaults to all cores):
```shell
curl -X POST "http://localhost:6667/stress" | jq
```
---
### Start with 2 workers for 10s:
```shell
curl -X POST "http://localhost:6667/stress?n=2&duration=10" | jq
```
---
### Stop immediately:
```shell
curl -X POST "http://localhost:6667/stop" | jq
```