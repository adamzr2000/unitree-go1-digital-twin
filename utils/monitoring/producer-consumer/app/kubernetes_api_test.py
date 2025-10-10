#!/usr/bin/env python3
import argparse, json, logging, sys, time
from kubernetes import client, config
from kubernetes.client.rest import ApiException
from kubernetes.stream import stream

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
log = logging.getLogger("pod-net1-raw")

def init_k8s(kubeconfig: str):
    try:
        config.load_kube_config(config_file=kubeconfig)
        log.info(f"Loaded kubeconfig from {kubeconfig}")
    except Exception as e:
        log.error(f"Failed to load kubeconfig: {e}")
        sys.exit(1)
    return client.CoreV1Api()

def parse_proc_net_dev(text: str):
    per = {}
    for line in text.strip().splitlines()[2:]:
        parts = [p for p in line.replace(":", " ").split() if p]
        if len(parts) < 11:
            continue
        iface, rx, tx = parts[0], int(parts[1]), int(parts[9])
        per[iface] = {"rx_bytes": rx, "tx_bytes": tx}
    return per

def get_iface_bytes(core: client.CoreV1Api, namespace: str, pod: str, iface: str, container: str | None, timeout_s: int = 5):
    try:
        out = stream(
            core.connect_get_namespaced_pod_exec,
            name=pod,
            namespace=namespace,
            command=["cat", "/proc/net/dev"],
            container=container,
            stderr=True, stdin=False, stdout=True, tty=False,
            _request_timeout=timeout_s,
        )
        per = parse_proc_net_dev(out)
        return per.get(iface)
    except ApiException as e:
        log.error(f"API exec failed: {e}")
        return None
    except Exception as e:
        log.error(f"Exec error: {e}")
        return None

def main():
    p = argparse.ArgumentParser(description="Poll a pod's specific interface (default: net1) and print raw RX/TX bytes")
    p.add_argument("--kubeconfig", default="/config/k8s-cluster-config.yaml", help="Path to kubeconfig")
    p.add_argument("--namespace", default="default", help="Pod namespace (default: default)")
    p.add_argument("--pod", required=True, help="Pod name")
    p.add_argument("--iface", default="net1", help="Interface to read (default: net1)")
    p.add_argument("--container", help="Optional container name")
    p.add_argument("--interval", type=float, default=5.0, help="Polling interval seconds (default: 5)")
    args = p.parse_args()

    core = init_k8s(args.kubeconfig)

    try:
        while True:
            counters = get_iface_bytes(core, args.namespace, args.pod, args.iface, args.container)
            if counters is None:
                print(json.dumps({
                    "namespace": args.namespace,
                    "pod": args.pod,
                    "iface": args.iface,
                    "error": "iface not found or exec failed"
                }, indent=2), flush=True)
            else:
                print(json.dumps({
                    "namespace": args.namespace,
                    "pod": args.pod,
                    "iface": args.iface,
                    "rx_bytes": counters["rx_bytes"],
                    "tx_bytes": counters["tx_bytes"]
                }, indent=2), flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)

if __name__ == "__main__":
    main()
