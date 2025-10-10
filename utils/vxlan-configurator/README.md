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
curl http://localhost:6666/health | jq
```
---
### Create iface + peers:
- Edge
```shell
curl -X POST http://localhost:6666/vxlan \
  -H 'Content-Type: application/json' \
  -d '{
    "vni": 200,
    "iface": "br10",
    "port": 4747,
    "vxlan_ip": "172.20.50.1/24",
    "remote_ips": ["10.3.202.68"]
  }' | jq
```
- Robot
```shell
curl -X POST http://localhost:6666/vxlan \
  -H 'Content-Type: application/json' \
  -d '{
    "vni": 200,
    "iface": "ue0",
    "port": 4747,
    "vxlan_ip": "172.20.50.2/24",
    "remote_ips": ["10.5.1.21"]
  }' | jq
```
- Remote cluster
```shell
curl -X POST http://localhost:6666/vxlan \
  -H 'Content-Type: application/json' \
  -d '{
    "vni": 200,
    "iface": "usb-eth0",
    "port": 4747,
    "vxlan_ip": "172.20.50.3/24",
    "remote_ips": ["10.5.1.21"]
  }' | jq
```
---
### Add peers later:
```shell
curl -X POST http://localhost:6666/vxlan/vxlan200/peers \
  -H 'Content-Type: application/json' \
  -d '{"peers": ["10.5.99.30"]}' | jq
```
---
### Show FDB:
```shell
curl http://localhost:6666/vxlan/vxlan200/fdb | jq
```
---
### Delete interface:
```shell
curl -X DELETE http://localhost:6666/vxlan/vxlan200 | jq
```

