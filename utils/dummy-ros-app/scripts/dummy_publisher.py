#!/usr/bin/env python3

import rospy
import argparse
from std_msgs.msg import String


class DummyPublisher:
    def __init__(self, topic_name, publish_rate, message):
        rospy.init_node('dummy_publisher', anonymous=True)

        # Store parameters
        self.topic_name = topic_name
        self.publish_rate = publish_rate
        self.message = message

        # Set up publisher
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)

        # Set up rate
        self.rate = rospy.Rate(self.publish_rate)

    def publish_message(self):
        while not rospy.is_shutdown():
            msg = String()
            msg.data = self.message
            rospy.loginfo(f"Publishing: {msg.data}")
            self.publisher.publish(msg)
            self.rate.sleep()


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="ROS1 Dummy Publisher")
    parser.add_argument("--topic", type=str, default="chatter", help="Topic name to publish to")
    parser.add_argument("--rate", type=float, default=1.0, help="Publishing rate (Hz)")
    parser.add_argument("--message", type=str, default="Hello from dummy_publisher", help="Message to publish")

    args = parser.parse_args()

    # Start publisher node
    node = DummyPublisher(args.topic, args.rate, args.message)
    try:
        node.publish_message()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
