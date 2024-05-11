import subprocess
import requests
import json
import threading 
import argparse  

import rospy
from std_msgs.msg import Header  # Example message type; adjust based on actual use
from collections import deque
import importlib
import time


# Dictionary mapping topics to their respective message types
TOPIC_MESSAGE_MAP = {
    '/scan': 'sensor_msgs.msg.LaserScan',
    '/go1_controller/cmd_vel': 'geometry_msgs.msg.TwistStamped',
    '/go1_controller/odom': 'nav_msgs.msg.Odometry',
    '/joint_states': 'sensor_msgs.msg.JointState'
}

class JitterMonitor:
    def __init__(self, topic, server_url, window_size=50):
        self.topic = topic
        self.server_url = server_url
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
        metric_data = {
            "topic": self.topic,
            "metric": "jitter",
            "value": f"{jitter:.3f} ms"
        }
        response = requests.post(f"{self.server_url}/api/metrics", json=metric_data)
        # print(f"Sent jitter data to {self.server_url}; Response: {response.status_code} - {response.text}")

def monitor_jitter_and_send(topic, server_url, window_size=50):
    # rospy.init_node('jitter_monitor_node')
    jitter_monitor = JitterMonitor(topic, server_url, window_size)
    rospy.spin()

def monitor_bw_delay_and_send(topic, metric_type, server_url, window_size=None):

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


                    # Prepare the combined value string
                    value = f" (average/min/max/std_dev): {average_delay:.3f} / {min_delay_ms:.3f} / {max_delay_ms:.3f} / {std_dev_ms:.3f} ms"

                elif metric_type == "bandwidth" and line.strip().startswith("average:"):
                    parts = line.strip().split()                    
                    average_value = parts[1]  # average bandwidth value
                    stats_line = process.stdout.readline()
                    # print("Debug stats_line:", stats_line) 
                    stats_parts = stats_line.strip().split()
                    average_message_size = stats_parts[1]  # mean size of the messages (in bytes)
                    value = f": {average_value} (message size: {average_message_size})"

                metric_data = {
                    "topic": topic,
                    "metric": metric_type,
                    "value": value
                }
                print(f"Sending data: metric={metric_type};topic={topic }")  # Debug output
                response = requests.post(f"{server_url}/api/metrics", json=metric_data)
                print(f"Response from server: {response.status_code}, {response.text}")  # Debug output
    finally:
        process.kill()
        process.wait()

def start_monitoring(topics, server_url, window_size=None):
    threads = []
    for topic in topics:
        # Start threads for both delay and bandwidth for each topic
        thread_delay = threading.Thread(target=monitor_bw_delay_and_send, args=(topic, "delay", server_url, window_size))
        thread_bandwidth = threading.Thread(target=monitor_bw_delay_and_send, args=(topic, "bandwidth", server_url, window_size))
        thread_jitter = threading.Thread(target=monitor_jitter_and_send, args=(topic, server_url, window_size))
        threads.append(thread_delay)
        threads.append(thread_bandwidth)
        threads.append(thread_jitter)
        thread_delay.start()
        thread_bandwidth.start()
        thread_jitter.start()

    for thread in threads:
        thread.join()  # Wait for all threads to complete


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Monitor ROS topics and send metrics to a server.')
    parser.add_argument('topics', type=str, nargs='+',
                        help='List of ROS topics to monitor for both delay and bandwidth metrics.')
    parser.add_argument('--server_url', type=str, default='http://127.0.0.1:5000',
                        help='URL of the server to send metrics to (default: http://127.0.0.1:5000)')
    parser.add_argument('--window_size', type=int, default=50,
                        help='Window size for the metric calculation (default: 50)')

    args = parser.parse_args()

    rospy.init_node('monitoring_node')  # Initialize the node in the main function

    # Start monitoring with the specified topics, server URL, and window size
    start_monitoring(args.topics, args.server_url, args.window_size)
