#!/usr/bin/env python3
import os
import time
import signal
import socket
import threading
import subprocess
import re

import rosgraph
import rospy
import rostopic

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import WriteOptions

STOP_EVENT = threading.Event()

def log(level, msg):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [{level}] {msg}", flush=True)

# ---------------- Env config ----------------
INFLUX_URL     = os.getenv("INFLUXDB_URL", "http://localhost:8086")
INFLUX_TOKEN   = os.getenv("INFLUXDB_TOKEN", "desire6g2024;")
INFLUX_ORG     = os.getenv("INFLUXDB_ORG", "desire6g")
INFLUX_BUCKET  = os.getenv("INFLUXDB_BUCKET", "monitoring")
TOPICS         = [t.strip() for t in os.getenv("TOPICS", "/joint_states").split(",") if t.strip()]
WAIT_FOR_MASTER= os.getenv("WAIT_FOR_MASTER", "true").lower() in ("1","true","yes","y")
WINDOW_SIZE    = int(os.getenv("WINDOW_SIZE", "50"))  # number of messages in rostopic window
NODE_ROLE      = os.getenv("NODE_ROLE", "auto")  # "edge" | "robot" | "auto"

def print_env_config():
    log("CONFIG", "======== Runtime configuration ========")
    log("CONFIG", f"InfluxDB URL: {INFLUX_URL}")
    log("CONFIG", f"InfluxDB Org: {INFLUX_ORG}")
    log("CONFIG", f"InfluxDB Bucket: {INFLUX_BUCKET}")
    log("CONFIG", f"InfluxDB Token: {INFLUX_TOKEN}")
    log("CONFIG", f"Wait for ROS master: {WAIT_FOR_MASTER}")
    log("CONFIG", f"Window size (messages): {WINDOW_SIZE}")
    log("CONFIG", f"Node role (requested): {NODE_ROLE}")
    log("CONFIG", f"Topics ({len(TOPICS)}): {', '.join(TOPICS) if TOPICS else '<none>'}")
    log("CONFIG", "=======================================")

# call this after reading env vars, before starting threads
print_env_config()

# ---------------- Helpers ----------------
def wait_for_master():
    while not rosgraph.is_master_online() and not STOP_EVENT.is_set():
        log("INFO", "Waiting for ROS master...")
        time.sleep(1)

def resolve_msg_class(topic):
    """Return message class (or None) for a topic without blocking."""
    try:
        cls, _, _ = rostopic.get_topic_class(topic, blocking=False)
        return cls
    except Exception:
        return None

def topic_has_header_stamp(topic):
    cls = resolve_msg_class(topic)
    if cls is None:
        log("WARN", f"[delay-check] Cannot resolve message type for {topic}; skipping delay.")
        return False

    try:
        has_header = any(a == 'header' for a in getattr(cls, '__slots__', []))
        log("INFO", f"[delay-check] {topic} type: {cls.__module__}.{cls.__name__} | header.stamp: {'yes' if has_header else 'no'}")
        return bool(has_header)
    except Exception as e:
        log("WARN", f"[delay-check] Error inspecting {topic} type: {e}; skipping delay.")
        return False

def clamp_nonnegative(x):
    return x if x >= 0.0 else 0.0

def kbps_from_human(token):
    """
    Convert rostopic 'average:' token like '12.3KB/s' or '1.2MB/s' to kbps (kilobits/s).
    """
    m = re.match(r"(\d+(?:\.\d+)?)([KMG]?B)/s", token)
    if not m:
        # Fallback: plain number => B/s
        return float(token) * 8.0 / 1000.0
    num = float(m.group(1))
    unit = m.group(2)  # B, KB, MB, GB
    scale = {"B":1, "KB":1024, "MB":1024**2, "GB":1024**3}[unit]
    bps = num * scale * 8.0
    return bps / 1000.0

def bytes_from_human(token):
    m = re.match(r"(\d+(?:\.\d+)?)([KMG]?B)", token)
    if not m:
        return float(token)  # assume bytes
    num = float(m.group(1))
    unit = m.group(2)
    scale = {"B":1, "KB":1024, "MB":1024**2, "GB":1024**3}[unit]
    return num * scale

def monitor_rostopic(topic, kind, write_api):
    cmd = ["rostopic", "delay" if kind == "delay" else "bw", topic, "-w", str(WINDOW_SIZE)]
    log("INFO", f"[start] {kind} monitor for {topic} (window={WINDOW_SIZE}) -> {' '.join(cmd)}")

    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    subscribed = False
    last_progress_log = time.time()

    try:
        while not STOP_EVENT.is_set():
            # Non-blocking-ish stderr read (line-by-line if available)
            if process.stderr and not process.stderr.closed:
                try:
                    if process.stderr.peek():  # may not exist in all environments
                        err_line = process.stderr.readline()
                        if err_line:
                            log("WARN", f"{topic} ({kind}) stderr: {err_line.strip()}")
                except Exception:
                    # fallback: ignore if peek not supported
                    pass

            line = process.stdout.readline()
            if not line:
                # If the process ended, flush any remaining stderr
                err = process.stderr.read() if process.stderr else ""
                if err.strip():
                    log("ERROR", f"{topic} ({kind}) stderr (final): {err.strip()}")
                break

            s = line.strip()
            if s.startswith("subscribed to"):
                subscribed = True
                log("INFO", f"[subscribed] {kind} -> {topic}")
                last_progress_log = time.time()
                continue

            if not subscribed:
                # Still waiting for subscribe confirmation
                if time.time() - last_progress_log > 5:
                    log("INFO", f"[waiting-subscribe] {kind} -> {topic}")
                    last_progress_log = time.time()
                continue

            # Heartbeat while waiting for enough samples/output
            if time.time() - last_progress_log > 5:
                log("INFO", f"[waiting-data] {kind} -> {topic}")
                last_progress_log = time.time()

            if kind == "delay" and s.startswith("average delay:"):
                parts = s.split()
                try:
                    avg_ms = clamp_nonnegative(float(parts[2]) * 1000.0)
                except Exception:
                    continue

                stats = process.stdout.readline().strip()
                try:
                    min_s = re.search(r"min:\s+([\d.]+)s", stats).group(1)
                    max_s = re.search(r"max:\s+([\d.]+)s", stats).group(1)
                    std_s = re.search(r"std dev:\s+([\d.]+)s", stats).group(1)
                except Exception as e:
                    log("WARN", f"Delay stats parse failed for {topic}: {e}")
                    min_s = max_s = std_s = "0"

                p = (Point("ros_metrics")
                     .tag("topic", topic)
                     .tag("source", "ros")
                     .tag("metric_type", "delay")
                     .field("avg_ms", avg_ms)
                     .field("min_ms", float(min_s) * 1000.0)
                     .field("max_ms", float(max_s) * 1000.0)
                     .field("std_ms", float(std_s) * 1000.0)
                     .time(time.time_ns()))
                try:
                    write_api.write(bucket=INFLUX_BUCKET, record=p)
                    log("INFO", f"[write] delay {topic}: avg={avg_ms:.2f}ms min={float(min_s)*1000:.2f}ms max={float(max_s)*1000:.2f}ms")
                except Exception as e:
                    log("ERROR", f"Influx write (delay) failed for {topic}: {e}")

                last_progress_log = time.time()

            elif kind == "bandwidth" and s.startswith("average:"):
                parts = s.split()
                try:
                    avg_kbps = kbps_from_human(parts[1])
                except Exception as e:
                    log("WARN", f"BW parse fail for {topic}: {e}")
                    continue

                stats = process.stdout.readline().strip()
                try:
                    mean_b = bytes_from_human(re.search(r"mean:\s+(\S+)", stats).group(1))
                    min_b  = bytes_from_human(re.search(r"min:\s+(\S+)", stats).group(1))
                    max_b  = bytes_from_human(re.search(r"max:\s+(\S+)", stats).group(1))
                except Exception as e:
                    log("WARN", f"BW stats parse fail for {topic}: {e}")
                    mean_b = min_b = max_b = 0

                p = (Point("ros_metrics")
                     .tag("topic", topic)
                     .tag("source", "ros")
                     .tag("metric_type", "bandwidth")
                     .field("avg_kbps", avg_kbps)
                     .field("mean_bytes", float(mean_b))
                     .field("min_bytes", float(min_b))
                     .field("max_bytes", float(max_b))
                     .time(time.time_ns()))
                try:
                    write_api.write(bucket=INFLUX_BUCKET, record=p)
                    log("INFO", f"[write] bandwidth {topic}: avg={avg_kbps:.1f} kbps mean={float(mean_b):.0f}B")
                except Exception as e:
                    log("ERROR", f"Influx write (bw) failed for {topic}: {e}")

                last_progress_log = time.time()

    except Exception as e:
        log("ERROR", f"{topic} ({kind}) exception: {e}")
    finally:
        try:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except Exception:
                process.kill()
        except Exception:
            pass
        log("INFO", f"[stop] {kind} monitor for {topic}")

def signal_handler(sig, frame):
    log("INFO", f"Signal {sig} received, shutting down…")
    STOP_EVENT.set()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if WAIT_FOR_MASTER:
        wait_for_master()
        if STOP_EVENT.is_set():
            return

    # Influx client with batching
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    write_api = client.write_api(write_options=WriteOptions(batch_size=500, flush_interval=2000))

    role = NODE_ROLE
    if role == "auto":
        role = "edge" if "/joint_states" in TOPICS else "robot"

    node_name = f"monitoring_node_{role}"
    rospy.init_node(node_name, anonymous=False)

    threads = []
    for t in TOPICS:
        log("INFO", f"Starting bandwidth monitor thread for {t}")
        th_b = threading.Thread(target=monitor_rostopic, args=(t, "bandwidth", write_api), daemon=True)
        th_b.start()
        threads.append(th_b)

        if topic_has_header_stamp(t):
            log("INFO", f"Starting delay monitor thread for {t}")
            th_d = threading.Thread(target=monitor_rostopic, args=(t, "delay", write_api), daemon=True)
            th_d.start()
            threads.append(th_d)
        else:
            log("INFO", f"Skipping delay for {t}: no header.stamp on message type.")


    # Keep main thread alive until stop
    try:
        while not STOP_EVENT.is_set():
            time.sleep(0.5)
    finally:
        STOP_EVENT.set()
        for th in threads:
            th.join(timeout=1.0)
        try:
            write_api.flush()
        except Exception:
            pass
        client.close()
        log("INFO", "Shutdown complete.")

if __name__ == "__main__":
    main()
