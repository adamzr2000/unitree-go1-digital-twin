#!/usr/bin/env python3

import rospy
import argparse
from std_msgs.msg import String


class DummySubscriber:
    def __init__(self, topic_name):
        rospy.init_node('dummy_subscriber', anonymous=True)

        # Store topic name
        self.topic_name = topic_name

        # Create subscriber
        self.subscriber = rospy.Subscriber(self.topic_name, String, self.listener_callback)
        rospy.loginfo(f"Subscribed to topic: {self.topic_name}")

    def listener_callback(self, msg):
        rospy.loginfo(f"Received message: {msg.data}")

    def spin(self):
        rospy.spin()


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="ROS1 Dummy Subscriber")
    parser.add_argument("--topic", type=str, default="chatter", help="Topic name to subscribe to")

    args = parser.parse_args()

    # Start subscriber node
    node = DummySubscriber(args.topic)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
