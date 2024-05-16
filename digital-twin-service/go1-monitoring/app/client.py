import time
import importlib
from collections import deque
import subprocess
import threading
import argparse
import re

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS


import rospy
from std_msgs.msg import Header 


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
    def __init__(self, topic,  window_size=50):
        self.topic = topic
        self.window_size = window_size
        self.last_time = None
        self.last_send_time = time.time()  # Initialize the last send time
        self.delays = deque(maxlen=window_size)  # Use deque to automatically limit size
        self.jitter_values = deque(maxlen=window_size - 1)  # Store jitter values between delays

        # Dynamically import the message type
        msg_type = TOPIC_MESSAGE_MAP[topic]
        module_name, class_name = msg_type.rsplit('.', 1)
        msg_module = importlib.import_module(module_name)
        msg_class = getattr(msg_module, class_name)

        self.subscriber = rospy.Subscriber(topic, msg_class, self.callback)

    def callback(self, msg):
        current_time = msg.header.stamp.to_sec()  # Using the timestamp from the message header

        if self.last_time is not None:
            delay = (current_time - self.last_time) * 1000.0  # Convert delay to milliseconds
            self.delays.append(delay)

            if len(self.delays) > 1:
                diff = abs(self.delays[-1] - self.delays[-2])
                self.jitter_values.append(diff)

                # Check if 5 seconds have passed since the last send
                if time.time() - self.last_send_time >= 5:
                    if len(self.jitter_values) > 0:
                        average_jitter = sum(self.jitter_values) / len(self.jitter_values)
                        self.send_jitter(average_jitter)
                        self.last_send_time = time.time()  # Update last send time

        self.last_time = current_time

    def send_jitter(self, jitter):
        formatted_jitter_ms = f"{jitter:.3f}"
        point = Point("jitter")\
            .tag("topic", self.topic)\
            .field("jitter_ms", float(formatted_jitter_ms))\
            .time(time.time_ns(), WritePrecision.NS)

        write_api.write(bucket=influxdb_bucket, record=point)
        print(f"Sent jitter data to InfluxDB for topic {self.topic}")

def monitor_jitter_and_send(topic, window_size=50):
    # rospy.init_node('jitter_monitor_node')
    jitter_monitor = JitterMonitor(topic, window_size)
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
            # Convert based on the unit
            if 'K' in unit:
                return number
            elif 'M' in unit:
                return number * 1024
            elif 'G' in unit:
                return number * 1024**2
            else:  # Default is B/s, convert to KB/s
                return number / 1024
        else:
            # Convert size based on the unit
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


def monitor_bw_delay_and_send(topic, metric_type, window_size=None):

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
                    average_delay = float(parts[2]) * 1000  # Extract the average delay and convert to milliseconds

                    stats_line = process.stdout.readline()
                    # print("Debug stats_line:", stats_line) 
                    stats_parts = stats_line.strip().split()

                    # Extract min, max, and std_dev from the stats line
                    try:
                        min_index = stats_parts.index('min:') + 1
                        min_delay = stats_parts[min_index][:-1]  # Removing the trailing 's'

                        max_index = stats_parts.index('max:') + 1
                        max_delay = stats_parts[max_index][:-1]  # Removing the trailing 's'

                        std_dev_index = stats_parts.index('std') + 2  # 'std dev:' is split into 'std' and 'dev:'
                        
                        std_dev = stats_parts[std_dev_index][:-1]  # Removing the trailing 's'
                    except (ValueError, IndexError) as e:
                        print("Error extracting values:", e)
                        min_delay, max_delay, std_dev = '0', '0', '0'  # Default values in case of an error

                    # Convert string times to float and format to milliseconds
                    min_delay_ms = float(min_delay) * 1000
                    max_delay_ms = float(max_delay) * 1000
                    std_dev_ms = float(std_dev) * 1000

                    formatted_average_delay_ms = f"{average_delay:.3f}"
                    formatted_min_delay_ms = f"{min_delay_ms:.3f}"
                    formatted_max_delay_ms = f"{max_delay_ms:.3f}"
                    formatted_std_dev_delay_ms = f"{std_dev_ms:.3f}"

                    point = Point(metric_type)\
                        .tag("topic", topic)\
                        .field("average_delay_ms", float(formatted_average_delay_ms))\
                        .field("min_delay_ms", float(formatted_min_delay_ms))\
                        .field("max_delay_ms", float(formatted_max_delay_ms))\
                        .field("std:_dev_delay_ms", float(formatted_std_dev_delay_ms))\
                        .time(time.time_ns(), WritePrecision.NS)

                    write_api.write(bucket=influxdb_bucket, record=point)
                    print(f"Sending {point} data for topic {topic}")


                elif metric_type == "bandwidth" and line.strip().startswith("average:"):
                    parts = line.strip().split()                    
                    average_bandwidth_kilobytes_second = extract_numeric_value_from_unit(parts[1], is_bandwidth=True)  # Parse bandwidth with unit handling

                    stats_line = process.stdout.readline()
                    # print("Debug stats_line:", stats_line) 
                    # stats_parts = stats_line.strip().split()
                    # average_message_size_bytes = extract_numeric_value_from_unit(stats_parts[1])  # Parse message size with unit handling

                    min_bw = extract_numeric_value_from_unit(re.search(r"min: (\S+)", stats_line).group(1), is_bandwidth=False)
                    max_bw = extract_numeric_value_from_unit(re.search(r"max: (\S+)", stats_line).group(1), is_bandwidth=False)
                    mean_size_bytes = extract_numeric_value_from_unit(re.search(r"mean: (\S+)", stats_line).group(1), is_bandwidth=False)


                    point = Point(metric_type)\
                        .tag("topic", topic)\
                        .field("average_bandwidth_kilobytes_s", average_bandwidth_kilobytes_second)\
                        .field("average_message_size_bytes", mean_size_bytes)\
                        .field("min_message_size_bytes", min_bw)\
                        .field("max_message_size_bytes", max_bw)\
                        .time(time.time_ns(), WritePrecision.NS)

                    write_api.write(bucket=influxdb_bucket, record=point)
                    print(f"Sending {point} data for topic {topic}")
    finally:
        process.kill()
        process.wait()

def start_monitoring(topics,  window_size=None):
    threads = []
    for topic in topics:
        # Start threads for both delay and bandwidth for each topic
        thread_delay = threading.Thread(target=monitor_bw_delay_and_send, args=(topic, "delay",  window_size))
        thread_bandwidth = threading.Thread(target=monitor_bw_delay_and_send, args=(topic, "bandwidth",  window_size))
        thread_jitter = threading.Thread(target=monitor_jitter_and_send, args=(topic,  window_size))
        threads.append(thread_delay)
        threads.append(thread_bandwidth)
        threads.append(thread_jitter)
        thread_delay.start()
        thread_bandwidth.start()
        thread_jitter.start()

    for thread in threads:
        thread.join()  # Wait for all threads to complete


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Monitor ROS topics and send metrics to InfluxDB.')
    parser.add_argument('topics', type=str, nargs='+', help='List of ROS topics to monitor for both delay and bandwidth metrics.')
    parser.add_argument('--window_size', type=int, default=50, help='Window size for the metric calculation')
    parser.add_argument('--influxdb_url', type=str, required=True, help='InfluxDB server URL')
    parser.add_argument('--influxdb_token', type=str, required=True, help='InfluxDB access token')
    parser.add_argument('--influxdb_org', type=str, required=True, help='InfluxDB organization')
    parser.add_argument('--influxdb_bucket', type=str, required=True, help='InfluxDB bucket for data')

    args = parser.parse_args()

    # Initialize InfluxDB client
    client = InfluxDBClient(url=args.influxdb_url, token=args.influxdb_token, org=args.influxdb_org)
    write_api = client.write_api(write_options=SYNCHRONOUS)
    influxdb_bucket = args.influxdb_bucket

    rospy.init_node('monitoring_node')
    start_monitoring(args.topics, args.window_size)
