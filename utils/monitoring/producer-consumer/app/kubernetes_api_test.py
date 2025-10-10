#!/usr/bin/env python3
import argparse, json, logging, sys
from kubernetes import client, config
from kubernetes.client.rest import ApiException

log = logging.getLogger("pod-net")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

def init_k8s(kubeconfig: str):
    try:
        config.load_kube_config(config_file=kubeconfig)
        log.info(f"Loaded kubeconfig from {kubeconfig}")
    except Exception as e:
        log.error(f"Failed to load kubeconfig: {e}")
        sys.exit(1)
    return client.ApiClient()

def get_pod_network_metrics_on_node(api_client: client.ApiClient, node_name: str, only_ns: str | None = None) -> dict:
    try:
        summary_json, _, _ = api_client.call_api(
            "/api/v1/nodes/{node}/proxy/stats/summary", "GET",
            path_params={"node": node_name},
            header_params={"Accept": "application/json"},
            response_type="object",
            _request_timeout=5.0,
        )
    except ApiException as e:
        log.error(f"ERROR fetching summary from node '{node_name}': {e}")
        return {}
    except Exception as e:
        log.error(f"Unexpected error talking to node '{node_name}': {e}")
        return {}

    pod_net = {}
    for pod in summary_json.get("pods", []):
        try:
            ns = pod["podRef"]["namespace"]
            if only_ns and ns != only_ns:
                continue
            name = pod["podRef"]["name"]
            net = pod.get("network", {}) or {}
            pod_net[(ns, name)] = {
                "rx_bytes": int(net.get("rxBytes", 0) or 0),
                "tx_bytes": int(net.get("txBytes", 0) or 0),
            }
        except Exception as e:
            log.debug(f"Skipping a pod entry due to parse issue: {e}")
            continue
    return pod_net

def main():
    p = argparse.ArgumentParser(description="Fetch per-pod network metrics from kubelet Summary API")
    p.add_argument("--node", default="xtreme", help="Kubernetes node name (default: xtreme)")
    p.add_argument("--kubeconfig", default="/config/k8s-cluster-config.yaml",
                   help="Path to kubeconfig (default: /config/k8s-cluster-config.yaml)")
    p.add_argument("--namespace", default="",
                   help="Namespace to include (default: default). Use empty string to include all.")
    args = p.parse_args()

    only_ns = args.namespace if args.namespace.strip() != "" else None
    api_client = init_k8s(args.kubeconfig)
    data = get_pod_network_metrics_on_node(api_client, args.node, only_ns=only_ns)

    printable = {f"{ns}/{name}": v for (ns, name), v in data.items()}
    print(json.dumps(printable, indent=2))

if __name__ == "__main__":
    main()
