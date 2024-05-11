#!/usr/bin/env python
import rospy
import importlib
from collections import deque

class JitterCalculator:
    def __init__(self, topic, message_type, num_messages):
        self.last_time = None
        self.delays = deque(maxlen=num_messages)  # Use deque to automatically limit size
        self.jitter_values = deque(maxlen=num_messages - 1)  # Store jitter values between delays

        # Try to import the message type dynamically
        try:
            module_name, class_name = message_type.rsplit('.', 1)
            msg_module = importlib.import_module(module_name)
            msg_class = getattr(msg_module, class_name)
        except (ImportError, AttributeError) as e:
            rospy.logerr(f"Failed to import message type {message_type}: {e}")
            raise ImportError(f"Could not import message type {message_type}")

        # Subscribe to the specified topic
        self.sub = rospy.Subscriber(topic, msg_class, self.callback)

    def callback(self, msg):
        current_time = msg.header.stamp.to_sec()  # Using the timestamp from the message header

        if self.last_time is not None:
            delay = (current_time - self.last_time) * 1000.0  # Convert delay to milliseconds
            self.delays.append(delay)

            if len(self.delays) > 1:
                diff = abs(self.delays[-1] - self.delays[-2])
                self.jitter_values.append(diff)

                if len(self.jitter_values) > 0:
                    average_jitter = sum(self.jitter_values) / len(self.jitter_values)
                    rospy.loginfo(f"Current Average Jitter for {self.sub.resolved_name}: {average_jitter:.2f} ms")

        self.last_time = current_time

if __name__ == '__main__':
    rospy.init_node('jitter_calculator_node')
    
    topic_name = rospy.get_param('~topic_name', '/scan')
    message_type = rospy.get_param('~message_type', 'sensor_msgs.msg.LaserScan')
    num_messages = int(rospy.get_param('~num_messages', 2))
    
    jc = JitterCalculator(topic_name, message_type, num_messages)
    rospy.spin()
