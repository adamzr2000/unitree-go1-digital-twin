import os
import time
import json
import sys
import requests
import psutil
import argparse
import logging

# Configure Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
log = logging.getLogger(__name__)

def get_free_cpu_cores(threshold: float = 5.0, interval: float = 5.0) -> int:
    """
    Sample per-core usage over `interval` seconds and count cores below `threshold`%.
    """
    usages = psutil.cpu_percent(percpu=True, interval=interval)
    return sum(1 for u in usages if u < threshold)


def get_free_ram_gb() -> int:
    """
    Returns the available RAM in whole gigabytes.
    """
    mem = psutil.virtual_memory()
    return int(mem.available / (1024 ** 3))


def get_free_disk_gb(path: str = "/") -> int:
    """
    Returns the free disk space for the given path in whole gigabytes.
    """
    disk = psutil.disk_usage(path)
    return int(disk.free / (1024 ** 3))


def update_node(site_id: str,
                topology_endpoint: str,
                iml_endpoint: str,
                cpu: int,
                mem: int,
                storage: int) -> None:
    """
    Deletes existing node for site_id and posts updated metrics.
    """
    base_url = topology_endpoint.rstrip('/')
    delete_url = f"http://{base_url}/nodes/{site_id}"
    post_url = f"http://{base_url}/nodes/"

    try:
        r = requests.delete(delete_url, headers={'accept': 'application/json'})
        r.raise_for_status()
    except requests.HTTPError:
        if r.status_code != 404:
            log.warning(f"Delete failed for {site_id}: status {r.status_code}")

    payload = {
        "site_id": site_id,
        "cpu": cpu,
        "mem": mem,
        "storage": storage,
        "iml_endpoint": iml_endpoint
    }
    try:
        r = requests.post(
            post_url,
            headers={'accept': 'application/json', 'Content-Type': 'application/json'},
            json=payload
        )
        r.raise_for_status()
        log.info(f"Updated topology DB for {site_id}: CPU={cpu}, RAM={mem}GB, Disk={storage}GB")
    except requests.HTTPError as e:
        log.error(f"Failed to post node {site_id}: {e}")


def monitor_and_update(site_id: str,
                       topology_endpoint: str,
                       iml_endpoint: str,
                       threshold: float = 5.0,
                       interval: float = 5.0,
                       disk_path: str = "/") -> None:
    """
    Continuously gathers metrics and updates the topology DB endpoint.
    """
    psutil.cpu_percent(percpu=True, interval=None)

    try:
        while True:
            free_cores = get_free_cpu_cores(threshold=threshold, interval=interval)
            free_ram = get_free_ram_gb()
            free_disk = get_free_disk_gb(disk_path)

            update_node(
                site_id=site_id,
                topology_endpoint=topology_endpoint,
                iml_endpoint=iml_endpoint,
                cpu=free_cores,
                mem=free_ram,
                storage=free_disk
            )
    except KeyboardInterrupt:
        log.info("Monitoring stopped by user.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor system and update topology DB using config file.")
    parser.add_argument('--file', required=True,
                        help='Name of JSON config file in /config directory')
    args = parser.parse_args()

    config_path = os.path.join('/config', args.file)
    try:
        with open(config_path) as cfgf:
            cfg = json.load(cfgf)
    except Exception as e:
        log.error(f"Failed to load config '{config_path}': {e}")
        sys.exit(1)

    site_id = cfg.get('site_id')
    topology_endpoint = cfg.get('topology_endpoint')
    iml_endpoint = cfg.get('iml_endpoint')
    threshold = cfg.get('threshold', 5.0)
    interval = cfg.get('interval', 5.0)
    disk_path = cfg.get('disk_path', '/')

    if not (site_id and topology_endpoint and iml_endpoint):
        log.error("Config must include 'site_id', 'topology_endpoint', and 'iml_endpoint'.")
        sys.exit(1)

    monitor_and_update(
        site_id=site_id,
        topology_endpoint=topology_endpoint,
        iml_endpoint=iml_endpoint,
        threshold=threshold,
        interval=interval,
        disk_path=disk_path
    )

