import json
import logging
import argparse
import threading
from confluent_kafka import Consumer
from influxdb_client import InfluxDBClient, WriteOptions, Point
from influxdb_client.client.write_api import ASYNCHRONOUS
from datetime import datetime

# Configure Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
log = logging.getLogger(__name__)

# Parse Command-Line Argument for Config File
parser = argparse.ArgumentParser(description="Kafka to InfluxDB Consumer")
parser.add_argument('--file', type=str, required=True, help='Configuration file (JSON)')
args = parser.parse_args()

# Load JSON Configuration File
try:
    with open(args.file, 'r') as f:
        cfg = json.load(f)
    kafka_cfg    = cfg['kafka']
    influx_cfg   = cfg['influxdb']
    kafka_ip     = kafka_cfg['kafkaIP']
    kafka_port   = kafka_cfg['kafkaPort']
    kafka_group  = str(kafka_cfg['kafkaID'])
    topics       = ["infrastructure", "network", "applications"]
    influx_url   = influx_cfg['influxdb_url']
    influx_token = influx_cfg['influxdb_token']
    influx_org   = influx_cfg['influxdb_org']
    influx_bucket= influx_cfg['influxdb_bucket']
except Exception as e:
    log.error(f"Failed to load JSON config: {e}")
    exit(1)

# Initialize InfluxDB Client
try:
    influx_client = InfluxDBClient(
        url=influx_url,
        token=influx_token,
        org=influx_org
    )
    write_api = influx_client.write_api(
        write_options=WriteOptions(batch_size=100,
                                   flush_interval=2000,
                                   write_type=ASYNCHRONOUS)
    )
    log.info("Connected to InfluxDB")
except Exception as e:
    log.error(f"Error connecting to InfluxDB: {e}")
    exit(1)

def handle_message(msg, topic):
    try:
        payload   = json.loads(msg.value().decode("utf-8"))
        timestamp = datetime.fromisoformat(payload["time"])
        data      = payload["data"]
        batch     = []

        if topic == "infrastructure":
            for node, m in data.items():
                batch.append(
                    Point("infrastructure")
                    .tag("node", node)
                    .field("cpu_cores",       m["cpu_cores"])
                    .field("alloc_cpu_cores", m["alloc_cpu_cores"])
                    .field("cpu_pct",         m["cpu_pct"])
                    .field("mem_gb",          m["mem_gb"])
                    .field("alloc_mem_gb",    m["alloc_mem_gb"])
                    .field("mem_pct",         m["mem_pct"])
                    .time(timestamp)
                )

        elif topic == "network":
            for iface, s in data.get("network_io", {}).items():
                batch.append(
                    Point("network_io")
                    .tag("interface", iface)
                    .field("tx_bytes", s.get("tx_bytes", 0))
                    .field("rx_bytes", s.get("rx_bytes", 0))
                    .time(timestamp)
                )
            for tgt, s in data.get("ping", {}).items():
                batch.append(
                    Point("ping")
                    .tag("target", tgt)
                    .tag("ip",     s.get("ip", "unknown"))
                    .field("latency_ms",          s.get("latency_ms") or 0)
                    .field("packet_loss_percent", s.get("packet_loss_percent", 0))
                    .field("total_pings",         s.get("total_pings", 0))
                    .field("successful_pings",    s.get("successful_pings", 0))
                    .field("failed_pings",        s.get("failed_pings", 0))
                    .time(timestamp)
                )

        elif topic == "applications":
            for pod, s in data.items():
                p = Point("applications") \
                    .tag("pod", pod)

                node_val = s.get("node")
                if node_val:
                    p = p.tag("node", node_val)

                batch.append(
                   p.field("cpu_mcores",       s["cpu_mcores"])
                    .field("limit_cpu_mcores", s["limit_cpu_mcores"])
                    .field("cpu_limit_pct",    s.get("cpu_limit_pct", 0))
                    .field("mem_mb",           s["mem_mb"])
                    .field("limit_mem_mb",     s["limit_mem_mb"])
                    .field("mem_limit_pct",    s.get("mem_limit_pct", 0))
                    .field("net_tx_bytes",     s.get("net_tx_bytes", 0))
                    .field("net_rx_bytes",     s.get("net_rx_bytes", 0))
                    .time(timestamp)
                )

        if batch:
            write_api.write(bucket=influx_bucket, org=influx_org, record=batch)
            log.info(f"[{topic.upper()}] Wrote {len(batch)} points")
    except Exception as e:
        log.error(f"Error processing message on '{topic}': {e}")

def kafka_worker(topic):
    consumer_conf = {
        'bootstrap.servers': f"{kafka_ip}:{kafka_port}",
        'group.id':          f"{kafka_group}-{topic}",
        'auto.offset.reset': 'earliest'
    }
    consumer = Consumer(consumer_conf)
    consumer.subscribe([topic])
    log.info(f"[{topic}] Subscribed")
    try:
        while True:
            msg = consumer.poll(1.0)
            if msg and not msg.error():
                handle_message(msg, topic)
    except KeyboardInterrupt:
        log.info(f"[{topic}] Stopping")
    finally:
        consumer.close()

if __name__ == "__main__":
    threads = [
        threading.Thread(target=kafka_worker, args=(t,), daemon=True)
        for t in topics
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

