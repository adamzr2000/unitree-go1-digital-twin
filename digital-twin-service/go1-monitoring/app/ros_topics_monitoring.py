#!/usr/bin/env python3
import os
import io
import re
import time
import signal
import socket
import threading

import rosgraph
import rospy
import rostopic

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import WriteOptions

# ---------------- Globals ----------------
STOP_EVENT = threading.Event()

def log(level, msg):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [{level}] {msg}", flush=True)

# ---------------- Env config ----------------
INFLUX_URL     = os.getenv("INFLUXDB_URL", "http://localhost:8086")
INFLUX_TOKEN   = os.getenv("INFLUXDB_TOKEN", "")
INFLUX_ORG     = os.getenv("INFLUXDB_ORG", "desire6g")
INFLUX_BUCKET  = os.getenv("INFLUXDB_BUCKET", "monitoring")
TOPICS         = [t.strip() for t in os.getenv("TOPICS", "/joint_states").split(",") if t.strip()]
WAIT_FOR_MASTER= os.getenv("WAIT_FOR_MASTER", "true").lower() in ("1","true","yes","y")
NODE_ROLE      = os.getenv("NODE_ROLE", "auto")  # "edge" | "robot" | "auto"

# Reporting knobs
EMA_ALPHA        = float(os.getenv("EMA_ALPHA", "0.3"))       # smoothing factor [0..1]
REPORT_EVERY_SEC = float(os.getenv("REPORT_EVERY_SEC", "1.0"))# write period
RESOLVE_TIMEOUT  = float(os.getenv("RESOLVE_TIMEOUT_SEC", "15.0"))  # per-topic msg type resolve timeout

def _mask_secret(s: str, show_start: int = 3, show_end: int = 3) -> str:
    if not s:
        return "<empty>"
    if len(s) <= show_start + show_end:
        return "*" * len(s)
    return f"{s[:show_start]}***{s[-show_end:]}"

def print_env_config():
    log("CONFIG", "======== Runtime configuration ========")
    log("CONFIG", f"InfluxDB URL: {INFLUX_URL}")
    log("CONFIG", f"InfluxDB Org: {INFLUX_ORG}")
    log("CONFIG", f"InfluxDB Bucket: {INFLUX_BUCKET}")
    log("CONFIG", f"InfluxDB Token: {_mask_secret(INFLUX_TOKEN)}")
    log("CONFIG", f"Wait for ROS master: {WAIT_FOR_MASTER}")
    log("CONFIG", f"Topics ({len(TOPICS)}): {', '.join(TOPICS) if TOPICS else '<none>'}")
    log("CONFIG", f"EMA_ALPHA: {EMA_ALPHA} | REPORT_EVERY_SEC: {REPORT_EVERY_SEC} | RESOLVE_TIMEOUT_SEC: {RESOLVE_TIMEOUT}")
    log("CONFIG", f"Node role (requested): {NODE_ROLE}")
    log("CONFIG", "=======================================")

print_env_config()

# ---------------- Helpers ----------------
def wait_for_master():
    while not rosgraph.is_master_online() and not STOP_EVENT.is_set():
        log("INFO", "Waiting for ROS master…")
        time.sleep(0.5)

def _rosify(name: str) -> str:
    s = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    if not s or not s[0].isalpha():
        s = "n_" + s
    return s

def resolve_msg_class_with_retry(topic: str, timeout_s: float):
    """Try to resolve the message class for a topic quickly; retry until timeout."""
    t0 = time.time()
    interval = 0.25
    while not STOP_EVENT.is_set():
        try:
            cls, _, _ = rostopic.get_topic_class(topic, blocking=False)
            if cls is not None:
                return cls
        except Exception:
            pass
        if time.time() - t0 >= timeout_s:
            return None
        time.sleep(interval)
    return None

def has_header_stamp(msg_cls) -> bool:
    try:
        return 'header' in getattr(msg_cls, '__slots__', [])
    except Exception:
        return False

# ---------------- Monitors ----------------
class BandwidthMon:
    """
    Bytes accounting per callback:
      - serialize msg to BytesIO to get size (bytes)
      - accumulate since last report
      - every REPORT_EVERY_SEC -> compute kbps, EMA smooth, write to Influx
    """
    def __init__(self, topic: str, write_api, host_tag: str, role_tag: str):
        self.topic = topic
        self.write_api = write_api
        self.host_tag = host_tag
        self.role_tag = role_tag

        self.bytes_accum = 0
        self.last_report_wall = time.time()
        self.ema_kbps = None

        msg_cls = resolve_msg_class_with_retry(topic, RESOLVE_TIMEOUT)
        if msg_cls is None:
            raise RuntimeError(f"[bw] Cannot resolve message type for {topic} within {RESOLVE_TIMEOUT:.1f}s")
        self.sub = rospy.Subscriber(topic, msg_cls, self.cb, queue_size=50)
        log("INFO", f"[bw] subscribed: {topic} ({msg_cls.__module__}.{msg_cls.__name__})")

    def cb(self, msg):
        if STOP_EVENT.is_set():
            return
        # Estimate message size (bytes) via ROS serialization
        try:
            buf = io.BytesIO()
            msg.serialize(buf)
            size_bytes = buf.getbuffer().nbytes
        except Exception:
            # Fallback: approximate
            try:
                size_bytes = len(repr(msg).encode("utf-8"))
            except Exception:
                return

        self.bytes_accum += size_bytes

        now = time.time()
        dt = now - self.last_report_wall
        if dt >= REPORT_EVERY_SEC and self.bytes_accum > 0:
            inst_kbps = (self.bytes_accum * 8.0 / dt) / 1000.0
            self.ema_kbps = inst_kbps if self.ema_kbps is None else (
                EMA_ALPHA * inst_kbps + (1.0 - EMA_ALPHA) * self.ema_kbps
            )

            p = (Point("ros_metrics")
                 .tag("topic", self.topic)
                 .tag("source", "ros")
                 .tag("metric_type", "bandwidth")
                 .tag("host", self.host_tag)
                 .tag("role", self.role_tag)
                 .field("avg_kbps", float(f"{self.ema_kbps:.3f}"))
                 .time(time.time_ns()))
            try:
                self.write_api.write(bucket=INFLUX_BUCKET, record=p)
                log("INFO", f"[bw] {self.topic}: {self.ema_kbps:.1f} kbps (dt={dt:.2f}s, bytes={self.bytes_accum})")
            except Exception as e:
                log("ERROR", f"Influx write (bw) {self.topic} failed: {e}")

            # reset window
            self.bytes_accum = 0
            self.last_report_wall = now

class DelayMon:
    """
    One-way publish->receive delay (ms) using header.stamp.
    EMA-smoothed; reported every REPORT_EVERY_SEC.
    """
    def __init__(self, topic: str, write_api, host_tag: str, role_tag: str):
        self.topic = topic
        self.write_api = write_api
        self.host_tag = host_tag
        self.role_tag = role_tag

        self.ema_ms = None
        self.last_report_wall = time.time()

        msg_cls = resolve_msg_class_with_retry(topic, RESOLVE_TIMEOUT)
        if msg_cls is None:
            raise RuntimeError(f"[delay] Cannot resolve message type for {topic} within {RESOLVE_TIMEOUT:.1f}s")
        if not has_header_stamp(msg_cls):
            raise RuntimeError(f"[delay] Message type for {topic} has no header.stamp")

        self.sub = rospy.Subscriber(topic, msg_cls, self.cb, queue_size=50)
        log("INFO", f"[delay] subscribed: {topic} ({msg_cls.__module__}.{msg_cls.__name__})")

    def cb(self, msg):
        if STOP_EVENT.is_set():
            return
        if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
            return
        pub = msg.header.stamp
        if pub.to_nsec() == 0:
            return
        recv = rospy.Time.now()
        d_ms = (recv - pub).to_nsec() / 1e6
        if d_ms < 0:
            d_ms = 0.0  # clamp negatives (clock skew)

        self.ema_ms = d_ms if self.ema_ms is None else (
            EMA_ALPHA * d_ms + (1.0 - EMA_ALPHA) * self.ema_ms
        )

        now = time.time()
        if now - self.last_report_wall >= REPORT_EVERY_SEC:
            p = (Point("ros_metrics")
                 .tag("topic", self.topic)
                 .tag("source", "ros")
                 .tag("metric_type", "delay")
                 .tag("host", self.host_tag)
                 .tag("role", self.role_tag)
                 .field("avg_ms", float(f"{self.ema_ms:.3f}"))
                 .time(time.time_ns()))
            try:
                self.write_api.write(bucket=INFLUX_BUCKET, record=p)
                log("INFO", f"[delay] {self.topic}: {self.ema_ms:.2f} ms")
            except Exception as e:
                log("ERROR", f"Influx write (delay) {self.topic} failed: {e}")
            self.last_report_wall = now

# ---------------- Main ----------------
def signal_handler(sig, _):
    log("INFO", f"Signal {sig} → shutting down…")
    STOP_EVENT.set()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if WAIT_FOR_MASTER:
        wait_for_master()
        if STOP_EVENT.is_set():
            return

    # Influx (batching)
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    write_api = client.write_api(write_options=WriteOptions(batch_size=500, flush_interval=2000))

    # Role + node name
    role = NODE_ROLE
    if role == "auto":
        role = "edge" if "/joint_states" in TOPICS else "robot"
    host = socket.gethostname()
    node_name = _rosify(f"monitoring_node_{role}_{host}")
    rospy.init_node(node_name, anonymous=False)
    log("CONFIG", f"Node role (resolved): {role} | Node: {node_name}")

    # Start monitors
    alive = False
    for t in TOPICS:
        # BW always
        try:
            log("INFO", f"Start BW for {t}")
            _ = BandwidthMon(t, write_api, host_tag=host, role_tag=role)
            alive = True
        except Exception as e:
            log("WARN", f"BW init failed for {t}: {e}")

        # Delay only if stamped
        try:
            msg_cls = resolve_msg_class_with_retry(t, 0.0)  # quick check
            if msg_cls and has_header_stamp(msg_cls):
                log("INFO", f"Start Delay for {t}")
                _ = DelayMon(t, write_api, host_tag=host, role_tag=role)
                alive = True
            else:
                log("INFO", f"Skip Delay for {t} (no header.stamp)")
        except Exception as e:
            log("WARN", f"Delay init failed for {t}: {e}")

    if not alive:
        log("WARN", "No monitors started (could not resolve any topic types). Will idle.")

    # Spin
    try:
        while not STOP_EVENT.is_set():
            time.sleep(0.2)
    finally:
        STOP_EVENT.set()
        try:
            write_api.flush()
        except Exception:
            pass
        client.close()
        log("INFO", "Shutdown complete.")

if __name__ == "__main__":
    main()
