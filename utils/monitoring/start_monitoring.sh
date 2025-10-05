#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<EOF
Usage: $(basename "$0") --local-ip <IP_ADDRESS> [--topology-update] [--influx-export]

  --local-ip         IP address for Kafka / InfluxDB routing (required)
  --topology-update  If set, start the topology_updater agent
  --influx-export    If set, start the kafka_to_influx agent
EOF
  exit 1
}

# Defaults
TOPO_UPDATE=false
INFLUX_EXPORT=false

# Parse args
if [[ $# -lt 2 ]]; then
  usage
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local-ip)
      shift
      LOCAL_IP="$1"
      ;;
    --topology-update)
      TOPO_UPDATE=true
      ;;
    --influx-export)
      INFLUX_EXPORT=true
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
  shift
done

if [[ -z "${LOCAL_IP:-}" ]]; then
  echo "--local-ip is required" >&2
  usage
fi

# Compute the absolute directory this script lives in
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EDGE_SCRIPT_DIR="${SCRIPT_DIR}/edge"
PC_DIR="${SCRIPT_DIR}/producer-consumer"
CONFIG_DIR="${PC_DIR}/config"

echo "Starting monitoring with LOCAL_IP=${LOCAL_IP}"

# 1. Start the edge stack
echo "→ Launching edge stack..."
cd "$EDGE_SCRIPT_DIR"
./start_edge.sh --local-ip "$LOCAL_IP"

echo "Waiting 10s for edge services..."
sleep 10

# 2. Patch all config JSONs
echo "→ Patching configs in ${CONFIG_DIR} to use IP ${LOCAL_IP}..."
for cfg in "$CONFIG_DIR"/*.json; do
  sed -i \
    -e "s/\"kafkaIP\": \".*\"/\"kafkaIP\": \"${LOCAL_IP}\"/" \
    -e "s|\"influxdb_url\": \".*\"|\"influxdb_url\": \"http://${LOCAL_IP}:8087\"|" \
    "$cfg"
done

# 3. Move into producer-consumer
cd "$PC_DIR"

# 4. Always start Kafka Producer
echo "→ Starting Kafka Producer..."
if ./run_monitoring_agent.sh \
     --script kafka_producer.py \
     --file configP1.json \
     --name mon-kafka-producer; then
  echo "   ✓ Kafka Producer started"
else
  echo "   ✗ Kafka Producer failed"
fi

# 5. Conditionally start Topology Updater
if $TOPO_UPDATE; then
  sleep 2
  echo "→ Starting Topology Updater..."
  if ./run_monitoring_agent.sh \
       --script topology_updater.py \
       --file configTopologyUpdater.json \
       --name mon-topology-updater; then
    echo "   ✓ Topology Updater started"
  else
    echo "   ✗ Topology Updater failed"
  fi
fi

# 6. Conditionally start Kafka→Influx exporter
if $INFLUX_EXPORT; then
  sleep 2
  echo "→ Starting Kafka→InfluxDB exporter..."
  if ./run_monitoring_agent.sh \
       --script kafka_to_influx.py \
       --file configCInfluxDB.json \
       --name mon-kafka-to-influx; then
    echo "   ✓ Kafka→InfluxDB exporter started"
  else
    echo "   ✗ Kafka→InfluxDB exporter failed"
  fi
fi

echo "✅ Monitoring setup complete."

