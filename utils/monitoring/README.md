# Kafka-Docker-MM2

This repository provides an example of synchronizing a distributed Kafka environment using MirrorMaker 2 (MM2). The deployment leverages MM2 to replicate data between Kafka clusters. MM2 runs as a standard Kafka broker with a modified entrypoint configured via `mm2.properties`.

This setup uses Kafka with RAFT and includes Kafka-UI for monitoring. It is based on Bitnami Kafka instead of Confluent Kafka and follows the approach outlined in [this guide](https://medium.com/larus-team/how-to-setup-mirrormaker-2-0-on-apache-kafka-multi-cluster-environment-87712d7997a4) with modifications.

## Project Structure
- `edge/` - Scripts for managing the Kafka instance at the edge (D6G site).
- `producer-consumer/` - Example Python-based Kafka producers and consumers.

## Quick Start

### Start Monitoring with Edge Kafka instance
```bash
./start_monitoring.sh --local-ip 10.5.1.21 --influx-export
```

```bash
./start_monitoring.sh --local-ip 10.5.1.21 --influx-export --topology-update
```

- Kafka UI: [http://10.5.1.21:8080](http://10.5.1.21:8080/)
- Grafana Dashboard: [http://10.5.1.21:3001](http://10.5.1.21:3000/)
- InfluxDB UI: [http://10.5.1.21:8087](http://10.5.1.21:8086/)


```bash
kubectl -n kube-system patch deploy metrics-server --type='json' -p='[
  {"op":"replace","path":"/spec/template/spec/containers/0/args/2","value":"--kubelet-preferred-address-types=ExternalIP,InternalIP,Hostname"},
  {"op":"add","path":"/spec/template/spec/containers/0/args/-","value":"--kubelet-insecure-tls"}
]'


# Verify
kubectl -n kube-system get deploy metrics-server -o json | jq -r '.spec.template.spec.containers[0].args[]'
```
> Note: This may be needed in the k3s master

### Stop Monitoring
```bash
./stop_monitoring.sh
```

---

## Utility Commands

### Edge Site Control
- Start Kafka Edge:
  ```bash
  ./start_edge.sh --local-ip <your-ip-address>
  ```
- Stop Kafka Edge:
  ```bash
  ./stop_edge.sh
  ```

### Producer Commands
- Start a Kafka Producer:
  ```bash
  python3 kafka_producer.py --file config/configP1.json
  ```
> Note: In the config file, replace `kafkaIP` with your IP address

### Consumer Commands
- Start a Kafka Consumer:
  ```bash
  python3 kafka_consumer.py --file config/configC1.json
  ```
> Note: In the config file, replace `kafkaIP` with your IP address

### InfluxDB Integration
- Send Kafka Data to InfluxDB:
  ```bash
  python3 kafka_to_influx.py --file config/configCInfluxDB.json
  ```
> Note: In the config file, replace `kafkaIP` with your IP address
