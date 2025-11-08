import argparse
import json
import logging
import sys
import time
import psutil

from kubernetes import client, config
from kubernetes.client.rest import ApiException
from kubernetes.stream import stream

from ping3 import ping
from kafkaConnections import kafkaConnections
from confluent_kafka import Producer
import threading
from datetime import datetime, timezone

# Configure Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
log = logging.getLogger(__name__)

# Argument Parser
parser = argparse.ArgumentParser(description="Kafka System Metrics Producer")
parser.add_argument('--file', dest='file', default="configP1.json", help="Configuration file")
args = parser.parse_args()

# ── Read config + log outcome ──────────────────────────────────────────────
try:
    with open(args.file, "r") as f:
        _cfg = json.load(f)
    log.info(f"Loaded config file: {args.file}")
except FileNotFoundError:
    log.warning(f"Config file not found: {args.file} — using defaults.")
    _cfg = {}
except json.JSONDecodeError as e:
    log.error(f"Invalid JSON in {args.file}: {e}. Using defaults.")
    _cfg = {}
except Exception as e:
    log.error(f"Error reading {args.file}: {e}. Using defaults.")
    _cfg = {}

def _truthy(v):
    if isinstance(v, bool): return v
    if isinstance(v, str):  return v.strip().lower() in {"1","true","yes","on"}
    return bool(v)

kafka_cfg = _cfg.get("kafka", {}) if isinstance(_cfg, dict) else {}
USE_MULTUS_INTERFACE = _truthy(
    _cfg.get("use_multus_interface", kafka_cfg.get("use_multus_interface", False))
)
MULTUS_IFACE_NAME = (
    _cfg.get("multus_interface_name", kafka_cfg.get("multus_interface_name", "net1"))
)

if USE_MULTUS_INTERFACE:
    log.info(f"Multus mode enabled: will read interface '{MULTUS_IFACE_NAME}' from /proc/net/dev.")
else:
    log.info("Multus mode disabled: using kubelet Summary API network bytes (typically eth0).")


# Kafka Initialization
try:
    ec = kafkaConnections(args.file)
    infrastructure_topic = "infrastructure"
    network_topic = "network"
    applications_topic = "applications"
    producer = ec.createKafkaProducer()
    for topic in [infrastructure_topic, network_topic, applications_topic]:
        ec.createKafkaTopic(topic)
except Exception as e:
    log.error(f"Failed to initialize Kafka: {e}")
    sys.exit(1)

# Kubernetes API Initialization
try:
    config.load_kube_config(config_file="/config/k8s-cluster-config.yaml")
    log.info("Successfully loaded kubeconfig from /config/k8s-cluster-config.yaml")
except Exception as e:
    log.error(f"Failed to load kubeconfig: {e}")
    sys.exit(1)
metrics_api = client.CustomObjectsApi()
core_api = client.CoreV1Api()

# ── Quick sanity log: confirm API works + print counts ──────────────────────
def k8s_sanity_log():
    try:
        # Nodes
        nodes = core_api.list_node(timeout_seconds=5).items
        node_count = len(nodes)

        # Pods (paged to avoid huge responses)
        pod_count = 0
        _continue = None
        while True:
            resp = core_api.list_pod_for_all_namespaces(
                limit=500, _continue=_continue, timeout_seconds=10
            )
            pod_count += len(resp.items)
            # 'continue' is a reserved word; python client exposes it as '_continue'
            _continue = getattr(resp.metadata, "_continue", None) or getattr(resp.metadata, "continue", None)
            if not _continue:
                break

        log.info(f"Kubernetes API OK — nodes={node_count}, pods={pod_count}")
    except ApiException as e:
        log.error(f"Kubernetes API sanity check failed (ApiException): {e}")
    except Exception as e:
        log.error(f"Kubernetes API sanity check failed: {e}")

k8s_sanity_log()


# Ping Targets
ping_targets = {
    "robot": "10.3.202.66",
}

ping_results = {name: {"total": 0, "success": 0, "failure": 0, "latest_latency": None} for name in ping_targets}

EXCLUDED_IFACE_PREFIXES = ["lo", "docker", "vibr", "br-", "br", "cni", "eno1", "eno2", "enp7s0", "enp9s0f0", "veth", "wlp"]
POLL_INTERVAL = 10  # seconds, match metrics-server scrape interval

def get_k8s_nodes_metrics():
    """Fetch usage, capacity, and compute utilization percentage per node in the Kubernetes cluster."""
    # Usage metrics
    try:
        resp = metrics_api.list_cluster_custom_object(group="metrics.k8s.io", version="v1beta1", plural="nodes")
        usage_map = {item["metadata"]["name"]: item["usage"] for item in resp.get("items", [])}
    except ApiException as e:
        log.error(f"Failed to fetch node usage: {e}")
        return {}
    # Capacity metrics
    try:
        nodes = core_api.list_node().items
    except ApiException as e:
        log.error(f"Failed to list nodes: {e}")
        return {}
    alloc_map = {n.metadata.name: n.status.allocatable for n in nodes}

    nodes_data = {}
    for node, usage in usage_map.items():
        # Parse usage
        cpu_str = usage.get("cpu")
        mem_str = usage.get("memory")
        # Convert CPU usage to millicores
        if cpu_str.endswith("n"): cpu_m = int(cpu_str[:-1]) // 1_000_000
        elif cpu_str.endswith("u"): cpu_m = int(cpu_str[:-1]) // 1000
        elif cpu_str.endswith("m"): cpu_m = int(cpu_str[:-1])
        else: cpu_m = int(float(cpu_str) * 1000)
        # Convert memory usage to bytes
        if mem_str.endswith("Ki"): mem_b = int(mem_str[:-2]) * 1024
        elif mem_str.endswith("Mi"): mem_b = int(mem_str[:-2]) * 1024**2
        elif mem_str.endswith("Gi"): mem_b = int(mem_str[:-2]) * 1024**3
        else: mem_b = int(mem_str)
        # Parse allocatable
        alloc = alloc_map.get(node, {})
        alloc_cpu_str = alloc.get("cpu", "0")
        alloc_mem_str = alloc.get("memory", "0")
        # Convert allocatable CPU to millicores
        if alloc_cpu_str.endswith("m"): alloc_cpu_m = int(alloc_cpu_str[:-1])
        else: alloc_cpu_m = int(float(alloc_cpu_str) * 1000)
        # Convert allocatable memory to bytes
        if alloc_mem_str.endswith("Ki"): alloc_mem_b = int(alloc_mem_str[:-2]) * 1024
        elif alloc_mem_str.endswith("Mi"): alloc_mem_b = int(alloc_mem_str[:-2]) * 1024**2
        elif alloc_mem_str.endswith("Gi"): alloc_mem_b = int(alloc_mem_str[:-2]) * 1024**3
        else: alloc_mem_b = int(alloc_mem_str)
        # Compute percentages
        cpu_pct = round(cpu_m / alloc_cpu_m * 100, 2) if alloc_cpu_m else 0.0
        mem_pct = round(mem_b / alloc_mem_b * 100, 2) if alloc_mem_b else 0.0
        # Convert to larger units
        cpu_cores = round(cpu_m / 1000, 3)
        alloc_cpu_cores = round(alloc_cpu_m / 1000, 3)
        mem_gb = round(mem_b / 1024**3, 3)
        alloc_mem_gb = round(alloc_mem_b / 1024**3, 3)
        nodes_data[node] = {
            "cpu_cores": cpu_cores,
            "alloc_cpu_cores": alloc_cpu_cores,
            "cpu_pct": cpu_pct,
            "mem_gb": mem_gb,
            "alloc_mem_gb": alloc_mem_gb,
            "mem_pct": mem_pct
        }
    return nodes_data

api_client = client.ApiClient()

# ── Helpers for Multus iface (no sidecars; API exec) ───────────────────────
def _parse_proc_net_dev(text: str):
    per = {}
    for line in text.strip().splitlines()[2:]:
        parts = [p for p in line.replace(":", " ").split() if p]
        if len(parts) < 11:
            continue
        iface, rx, tx = parts[0], int(parts[1]), int(parts[9])
        per[iface] = {"rx_bytes": rx, "tx_bytes": tx}
    return per

def _get_iface_bytes_via_api(namespace: str, pod: str, iface: str, container: str | None = None, timeout_s: int = 4):
    try:
        out = stream(
            core_api.connect_get_namespaced_pod_exec,
            name=pod,
            namespace=namespace,
            command=["cat", "/proc/net/dev"],
            container=container,
            stderr=True, stdin=False, stdout=True, tty=False,
            _request_timeout=timeout_s,
        )
        per = _parse_proc_net_dev(out)
        return per.get(iface)
    except ApiException as e:
        log.debug(f"Exec API error on {namespace}/{pod}:{iface}: {e}")
        return None
    except Exception as e:
        log.debug(f"Exec error on {namespace}/{pod}:{iface}: {e}")
        return None

# Helper to fetch per-pod network from Kubelet Summary API
def get_pod_network_metrics_on_node(node_name):
    """
    Use the Kubelet Summary API via the Python client’s ApiClient.call_api
    to fetch network stats for every pod on the given node.
    Returns a dict: { (namespace,name): {'rx_bytes':…, 'tx_bytes':…}, … }
    """
    try:
        summary_json, _, _ = api_client.call_api(
            '/api/v1/nodes/{node}/proxy/stats/summary', 'GET',
            path_params={'node': node_name},
            header_params={'Accept': 'application/json'},
            response_type='object'
        )
    except ApiException as e:
        log.error(f"ERROR fetching summary from {node_name}: {e}")
        return {}

    pod_net = {}
    for pod in summary_json.get('pods', []):
        ns   = pod['podRef']['namespace']
        name = pod['podRef']['name']
        net  = pod.get('network', {})

        # Default from Summary API (typically eth0-only)
        rx = net.get('rxBytes', 0)
        tx = net.get('txBytes', 0)

        # If enabled, try to read Multus iface (e.g., net1) via API exec and override
        if USE_MULTUS_INTERFACE:
            multus = _get_iface_bytes_via_api(ns, name, MULTUS_IFACE_NAME)
            if multus:
                rx, tx = multus['rx_bytes'], multus['tx_bytes']

        pod_net[(ns, name)] = {'rx_bytes': rx, 'tx_bytes': tx}
    return pod_net


def get_all_pod_network_metrics(node_map):
    """node_map: {(ns,name) -> node_name}"""
    merged = {}
    for node in sorted({n for n in node_map.values() if n}):
        m = get_pod_network_metrics_on_node(node)
        merged.update(m)
    return merged

def get_k8s_pods_metrics():
    try:
        pod_resp = metrics_api.list_cluster_custom_object(group="metrics.k8s.io", version="v1beta1", plural="pods")
        items = pod_resp.get("items", [])
    except ApiException as e:
        log.error(f"Failed to fetch pod usage: {e}")
        return {}
    try:
        pods = core_api.list_namespaced_pod(namespace="default").items
    except ApiException as e:
        log.error(f"Failed to list default-namespace pods: {e}")
        return {}
    spec_map = {(p.metadata.namespace, p.metadata.name): p.spec.containers for p in pods}
    node_map = {(p.metadata.namespace, p.metadata.name): p.spec.node_name for p in pods}

    #pod_net = get_pod_network_metrics_on_node(node_map.get(("default", items[0]["metadata"]["name"]), ""))
    pod_net = get_all_pod_network_metrics(node_map)

    def pct(used, limit): return round(used / limit * 100, 2) if limit else None
    pod_metrics = {}
    for item in items:
        ns, name = item["metadata"]["namespace"], item["metadata"]["name"]
        if ns != "default": continue
        key = (ns, name)
        node = node_map.get(key)
        used_cpu_m = used_mem_b = 0
        for c in item.get("containers", []):
            cpu, mem = c["usage"]["cpu"], c["usage"]["memory"]
            if cpu.endswith("m"): used_cpu_m += int(cpu[:-1])
            elif cpu.endswith("n"): used_cpu_m += int(cpu[:-1]) // 1_000_000
            elif cpu.endswith("u"): used_cpu_m += int(cpu[:-1]) // 1000
            else: used_cpu_m += int(float(cpu) * 1000)
            if mem.endswith("Mi"): used_mem_b += int(mem[:-2]) * 1024**2
            elif mem.endswith("Ki"): used_mem_b += int(mem[:-2]) * 1024
            elif mem.endswith("Gi"): used_mem_b += int(mem[:-2]) * 1024**3
        lim_cpu_m = lim_mem_b = 0
        for ctr in spec_map.get(key, []):
            limits = ctr.resources.limits or {}
            cpu_l = limits.get("cpu")
            if cpu_l: lim_cpu_m += int(cpu_l[:-1]) if cpu_l.endswith("m") else int(float(cpu_l) * 1000)
            mem_l = limits.get("memory")
            if mem_l and mem_l.endswith("Mi"): lim_mem_b += int(mem_l[:-2]) * 1024**2

        net_vals = pod_net.get(key, {"rx_bytes": None, "tx_bytes": None})
        pod_metrics[name] = {
            "node": node,
            "cpu_mcores": round(used_cpu_m, 3),
            "limit_cpu_mcores": lim_cpu_m,
            "cpu_limit_pct": pct(used_cpu_m, lim_cpu_m),
            "mem_mb": round(used_mem_b / 1024**2, 2),
            "limit_mem_mb": round(lim_mem_b / 1024**2, 2),
            "mem_limit_pct": pct(used_mem_b, lim_mem_b),
            "net_rx_bytes": net_vals.get("rx_bytes"),
            "net_tx_bytes": net_vals.get("tx_bytes")
        }
    return pod_metrics

def update_ping(name, ip):
    """Continuously pings the target IP and records latency & loss."""
    while True:
        try:
            latency = ping(ip, timeout=1)
            ping_results[name]["total"] += 1
            if latency is not None:
                ping_results[name]["success"] += 1
                ping_results[name]["latest_latency"] = round(latency * 1000, 2)
            else:
                ping_results[name]["failure"] += 1
        except Exception as e:
            log.warning(f"Ping error for {name} ({ip}): {e}")
            ping_results[name]["failure"] += 1
        time.sleep(1)


def get_network_io_metrics():
    """Collects per-interface TX/RX bytes in the edge server hosting the DT application."""
    counters = psutil.net_io_counters(pernic=True)
    net_io_data = {}
    for iface, stats in counters.items():
        if any(iface.startswith(pref) for pref in EXCLUDED_IFACE_PREFIXES):
            continue
        net_io_data[iface] = {
            "tx_bytes": stats.bytes_sent,
            "rx_bytes": stats.bytes_recv,
        }
    return net_io_data


def get_ping_metrics():
    ping_data = {}
    for name in ping_targets:
        stats = ping_results[name]
        total = stats["total"] or 1
        loss = round((stats["failure"] / total) * 100, 2)
        ping_data[name] = {
            "ip": ping_targets[name],
            "latency_ms": stats["latest_latency"],
            "packet_loss_percent": loss,
            "total_pings": stats["total"],
            "successful_pings": stats["success"],
            "failed_pings": stats["failure"],
        }
    return ping_data

def delivery_callback(err, msg):
    if err:
        log.error(f"Message delivery failed: {err}")
    else:
        log.info(f"Message delivered to {msg.topic()} [Partition {msg.partition()}] @ Offset {msg.offset()}")

# Launch ping threads
for name, ip in ping_targets.items():
    threading.Thread(target=update_ping, args=(name, ip), daemon=True).start()

# Helper
def publish(topic: str, data: dict):
    """Wrap with timestamp and send to Kafka with error handling."""
    payload = {
        "time": datetime.now(timezone.utc).isoformat(),
        "data": data,
    }
    print("DEBUG payload:", json.dumps(payload, separators=(",", ":"), ensure_ascii=False))
    try:
        producer.produce(
            topic,
            value=json.dumps(payload, separators=(",", ":")),
            callback=delivery_callback
        )
    except BufferError as e:
        log.warning(f"Queue full for {topic}, dropping: {e}")

# ─── Main loop ───────────────────────────────────────────────────────────────────
try:
    while True:
        # 1) Gather metrics
        infrastructure_metrics = get_k8s_nodes_metrics()
        network_io_metrics     = get_network_io_metrics()
        ping_metrics           = get_ping_metrics()
        applications_metrics   = get_k8s_pods_metrics()

        # 2) Build composite network metrics
        network_metrics = {
            "network_io": network_io_metrics,
            "ping":       ping_metrics,
        }

        # 3) Publish each topic with timestamp
        metrics_map = {
            infrastructure_topic: infrastructure_metrics,
            network_topic:        network_metrics,
            applications_topic:   applications_metrics,
        }

        for topic, mdata in metrics_map.items():
            publish(topic, mdata)

        # 4) Serve callbacks & single log
        producer.poll(1)
        log.info(f"[METRICS] Published to {', '.join(metrics_map.keys())}")

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    log.info("Kafka Producer Stopped.")
finally:
    producer.flush()
