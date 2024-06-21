import time
import importlib
from collections import deque
import subprocess
import threading
import argparse
import re
import rosgraph

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS


import rospy
from std_msgs.msg import Header 

# Helper function for formatted prints
def log_message(level, message):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [{level}] {message}", flush=True)


# Dictionary mapping topics to their respective message types
TOPIC_MESSAGE_MAP = {
    '/scan': 'sensor_msgs.msg.LaserScan',
    '/go1_controller/cmd_vel': 'geometry_msgs.msg.TwistStamped',
    '/go1_controller/odom': 'nav_msgs.msg.Odometry',
    '/joint_states': 'sensor_msgs.msg.JointState',
    '/camera/color/image_raw': 'sensor_msgs.msg.Image',
    '/camera/depth/image_raw': 'sensor_msgs.msg.Image',
    '/camera/depth/points': 'sensor_msgs.msg.PointCloud2'
}

class JitterMonitor:
    def __init__(self, topic, write_api, influxdb_bucket, window_size=50):
        self.topic = topic
        self.window_size = window_size
        self.last_time = None
        self.last_send_time = time.time()
        self.delays = deque(maxlen=window_size)
        self.jitter_values = deque(maxlen=window_size - 1)
        self.write_api = write_api
        self.influxdb_bucket = influxdb_bucket

        msg_type = TOPIC_MESSAGE_MAP[topic]
        module_name, class_name = msg_type.rsplit('.', 1)
        msg_module = importlib.import_module(module_name)
        msg_class = getattr(msg_module, class_name)

        self.subscriber = rospy.Subscriber(topic, msg_class, self.callback)

    def callback(self, msg):
        current_time = msg.header.stamp

        if self.last_time is not None:
            delay_ns = (current_time - self.last_time).to_nsec()
            delay = delay_ns / 1e6  # Convert to milliseconds
        
            self.delays.append(delay)

            if len(self.delays) > 1:
                diff = abs(self.delays[-1] - self.delays[-2])
                self.jitter_values.append(diff)

                if time.time() - self.last_send_time >= 30:
                    if self.jitter_values:
                        average_jitter = sum(self.jitter_values) / len(self.jitter_values)
                        self.send_jitter(average_jitter)
                        self.last_send_time = time.time()

        self.last_time = current_time

    def send_jitter(self, jitter):
        formatted_jitter_ms = f"{jitter:.3f}"
        point = Point("jitter")\
            .tag("topic", self.topic)\
            .field("jitter_ms", float(formatted_jitter_ms))\
            .time(time.time_ns(), WritePrecision.NS)

        try:
            self.write_api.write(bucket=self.influxdb_bucket, record=point)
            log_message("INFO", f"Sent jitter data to InfluxDB for topic {self.topic}")
        except Exception as e:
            log_message("ERROR", f"Failed to send jitter data to InfluxDB for topic {self.topic}: {e}")


class DelayMonitor:
    def __init__(self, topic, write_api, influxdb_bucket):
        self.topic = topic
        self.write_api = write_api
        self.influxdb_bucket = influxdb_bucket
        self.last_send_time = time.time()


        msg_type = TOPIC_MESSAGE_MAP[topic]
        module_name, class_name = msg_type.rsplit('.', 1)
        msg_module = importlib.import_module(module_name)
        msg_class = getattr(msg_module, class_name)

        self.subscriber = rospy.Subscriber(topic, msg_class, self.callback)

    def callback(self, msg):
        receive_time = rospy.Time.now()

        publish_time = msg.header.stamp

        # if not isinstance(publish_time, rospy.Time):
        #     log_message("ERROR", f"Message on topic {self.topic} does not have a valid header.stamp")
        #     return

        # Calculate delay in nanoseconds
        delay_ns = (receive_time - publish_time).to_nsec()
        delay = abs(delay_ns / 1e6)  # Convert to milliseconds and take absolute value
        
        current_time = time.time()
        if current_time - self.last_send_time >= 2.0:  # Send every 2 seconds
            self.send_delay(delay)
            self.last_send_time = current_time


    def send_delay(self, delay):
        formatted_delay_ms = f"{delay:.3f}"
        point = Point("delay")\
            .tag("topic", self.topic)\
            .field("average_delay_ms", float(formatted_delay_ms))\
            .time(time.time_ns(), WritePrecision.NS)

        try:
            self.write_api.write(bucket=self.influxdb_bucket, record=point)
            log_message("INFO", f"Sent delay data to InfluxDB for topic {self.topic}: {formatted_delay_ms} ms")

        except Exception as e:
            log_message("ERROR", f"Failed to send delay data to InfluxDB for topic {self.topic}: {e}")

def monitor_delay_and_send(topic, write_api, influxdb_bucket):
    delay_monitor = DelayMonitor(topic, write_api, influxdb_bucket)
    rospy.spin()

def monitor_jitter_and_send(topic, write_api, influxdb_bucket, window_size=50):
    jitter_monitor = JitterMonitor(topic, write_api, influxdb_bucket, window_size)
    rospy.spin()

def extract_numeric_value_from_unit(data_str, is_bandwidth=False):
    """ Extracts the numeric value from a string and adjusts based on the unit. 
        Assumes the unit could be B/s, KB/s, MB/s, GB/s for bandwidth and similar without /s for size. """
    # Regular expression to separate the number from the unit
    match = re.match(r"(\d+(\.\d+)?)([KMG]?B/s?)?", data_str)
    if match:
        number = float(match.group(1))
        unit = match.group(3) if match.group(3) is not None else ''

        if is_bandwidth:
            # Convert based on the unit to bits per second
            if 'K' in unit:
                return number * 8  
            elif 'M' in unit:
                return number * 8 * 1024
            elif 'G' in unit:
                return number * 8 * 1024**2  
            else:  # Default is B/s, convert to Kb/s
                return number * 8 / 1024
        else:
            # Convert size based on the unit to bytes
            if 'K' in unit:
                return number * 1024
            elif 'M' in unit:
                return number * 1024**2
            elif 'G' in unit:
                return number * 1024**3
            else:  # Default is B
                return number
    else:
        raise ValueError(f"Could not parse the value and unit from {data_str}")


def monitor_bw_delay_and_send(topic, metric_type, write_api, influxdb_bucket, window_size=None):
    if metric_type == "delay":
        command = ["unbuffer", "rostopic", "delay", topic]
    else:
        command = ["unbuffer", "rostopic", "bw", topic]

    if window_size is not None:
        command.extend(["-w", str(window_size)])
    
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    try:
        is_subscribed = False
        while True:
            line = process.stdout.readline()
            if not line:
                break
            if line.strip().startswith("subscribed to"):
                is_subscribed = True
            elif is_subscribed:
                if metric_type == "delay" and line.strip().startswith("average delay:"):
                    parts = line.strip().split()
                    average_delay = abs(float(parts[2]) * 1000)  # Extract the average delay and convert to milliseconds

                    stats_line = process.stdout.readline()
                    # print("Debug stats_line:", stats_line) 
                    stats_parts = stats_line.strip().split()

                    # Extract min, max, and std_dev from the stats line
                    try:
                        min_delay = stats_parts[stats_parts.index('min:') + 1][:-1]
                        max_delay = stats_parts[stats_parts.index('max:') + 1][:-1]
                        std_dev = stats_parts[stats_parts.index('std') + 2][:-1]
                    except (ValueError, IndexError) as e:
                        log_message("ERROR", f"Error extracting delay values for {topic}: {e}")
                        min_delay, max_delay, std_dev = '0', '0', '0'  # Default values in case of an error

                    # Convert string times to float and format to milliseconds
                    min_delay_ms = float(min_delay) * 1000
                    max_delay_ms = float(max_delay) * 1000
                    std_dev_ms = float(std_dev) * 1000

                    point = Point("delay")\
                        .tag("topic", topic)\
                        .field("average_delay_ms", average_delay)\
                        .field("min_delay_ms", min_delay_ms)\
                        .field("max_delay_ms", max_delay_ms)\
                        .field("std_dev_delay_ms", std_dev_ms)\
                        .time(time.time_ns(), WritePrecision.NS)

                    try:
                        write_api.write(bucket=influxdb_bucket, record=point)
                        log_message("INFO", f"Sent delay data to InfluxDB for topic {topic}")
                    except Exception as e:
                        log_message("ERROR", f"Failed to send delay data to InfluxDB for topic {topic}: {e}")


                elif metric_type == "bandwidth" and line.strip().startswith("average:"):
                    parts = line.strip().split()                    
                    average_bandwidth_kbps = extract_numeric_value_from_unit(parts[1], is_bandwidth=True)  # Parse bandwidth with unit handling
                    stats_line = process.stdout.readline()

                    # log_message("DEBUG", f"Stats line for bandwidth: {stats_line}")
                    try:
                        min_bw = extract_numeric_value_from_unit(re.search(r"min: (\S+)", stats_line).group(1), is_bandwidth=False)
                        max_bw = extract_numeric_value_from_unit(re.search(r"max: (\S+)", stats_line).group(1), is_bandwidth=False)
                        mean_size = extract_numeric_value_from_unit(re.search(r"mean: (\S+)", stats_line).group(1), is_bandwidth=False)
                    except AttributeError as e:
                        log_message("ERROR", f"Failed to parse bandwidth stats for {topic}: {e}")
                        min_bw, max_bw, mean_size = 0, 0, 0


                    point = Point(metric_type)\
                        .tag("topic", topic)\
                        .field("average_bandwidth_kbps", average_bandwidth_kbps)\
                        .field("average_message_size_bytes", mean_size)\
                        .field("min_message_size_bytes", min_bw)\
                        .field("max_message_size_bytes", max_bw)\
                        .time(time.time_ns(), WritePrecision.NS)

                    try:
                        write_api.write(bucket=influxdb_bucket, record=point)
                        log_message("INFO", f"Sent bandwidth data to InfluxDB for topic {topic}")
                    except Exception as e:
                        log_message("ERROR", f"Failed to send bandwidth data to InfluxDB for topic {topic}: {e}")
    except Exception as e:
        log_message("ERROR", f"Exception occurred while monitoring {topic} for {metric_type}: {e}")
    finally:
        process.kill()
        process.wait()
        

def start_monitoring(topics, write_api, influxdb_bucket, window_size=None, manual_delay=False):
    threads = []
    for topic in topics:
        if manual_delay:
            thread_delay = threading.Thread(target=monitor_delay_and_send, args=(topic, write_api, influxdb_bucket))
        else:
            thread_delay = threading.Thread(target=monitor_bw_delay_and_send, args=(topic, "delay", write_api, influxdb_bucket, window_size))
        thread_bandwidth = threading.Thread(target=monitor_bw_delay_and_send, args=(topic, "bandwidth", write_api, influxdb_bucket, window_size))
        thread_jitter = threading.Thread(target=monitor_jitter_and_send, args=(topic, write_api, influxdb_bucket, window_size))
        threads.extend([thread_delay, thread_bandwidth, thread_jitter])
        thread_delay.start()
        thread_bandwidth.start()
        thread_jitter.start()

    for thread in threads:
        thread.join()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Monitor ROS topics and send metrics to InfluxDB.')
    parser.add_argument('topics', type=str, nargs='+', help='List of ROS topics to monitor for both delay and bandwidth metrics.')
    parser.add_argument('--window_size', type=int, default=50, help='Window size for the metric calculation')
    parser.add_argument('--influxdb_url', type=str, required=True, help='InfluxDB server URL')
    parser.add_argument('--influxdb_token', type=str, required=True, help='InfluxDB access token')
    parser.add_argument('--influxdb_org', type=str, required=True, help='InfluxDB organization')
    parser.add_argument('--influxdb_bucket', type=str, required=True, help='InfluxDB bucket for data')
    parser.add_argument('--manual_delay', action='store_true', help='Use manual delay calculation instead of <rostopic delay> command')
    parser.add_argument('--wait', action='store_true', help='Wait for ROS master to be ready before starting')

    args = parser.parse_args()

    if args.wait:
        while not rosgraph.is_master_online():
            log_message("INFO", "Waiting for ROS master...")
            time.sleep(1)


    # Initialize InfluxDB client
    client = InfluxDBClient(url=args.influxdb_url, token=args.influxdb_token, org=args.influxdb_org)
    write_api = client.write_api(write_options=SYNCHRONOUS)
    influxdb_bucket = args.influxdb_bucket

    # Set the node name based on whether "/joint_states" is in the list of topics
    node_name = "monitoring_node_edge" if "/joint_states" in args.topics else "monitoring_node_robot"

    # Initialize the ROS node with a base name and set anonymous=True
    rospy.init_node(node_name, anonymous=False)

    start_monitoring(args.topics, write_api, influxdb_bucket, args.window_size, args.manual_delay)