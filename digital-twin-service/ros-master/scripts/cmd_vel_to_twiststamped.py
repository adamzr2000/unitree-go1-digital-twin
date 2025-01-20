import rospy
from geometry_msgs.msg import Twist, TwistStamped
import argparse

def cmd_vel_callback(msg):
    """
    Callback function to handle incoming Twist messages and convert them to TwistStamped.
    """
    # Create a TwistStamped message
    twist_stamped_msg = TwistStamped()
    
    # Populate the header with the current time
    twist_stamped_msg.header.stamp = rospy.Time.now()
    twist_stamped_msg.header.frame_id = "cmd_vel_frame"  # Updated frame_id to be related to cmd_vel

    # Copy the Twist data into the TwistStamped message
    twist_stamped_msg.twist = msg

    # Publish the TwistStamped message
    twist_stamped_pub.publish(twist_stamped_msg)

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Translate Twist messages from one topic to TwistStamped messages on another topic.")
    parser.add_argument("--input_topic", type=str, default="/cmd_vel", help="Name of the input topic (default: /cmd_vel)")
    parser.add_argument("--output_topic", type=str, default="/go1_controller/cmd_vel", help="Name of the output topic (default: /go1_controller/cmd_vel)")
    args = parser.parse_args()

    # Initialize the ROS node
    rospy.init_node("cmd_vel_to_twiststamped")

    # Create a publisher for the output topic
    twist_stamped_pub = rospy.Publisher(args.output_topic, TwistStamped, queue_size=1)

    # Create a subscriber for the input topic with a small queue size for immediate processing
    rospy.Subscriber(args.input_topic, Twist, cmd_vel_callback, queue_size=1)

    # Keep the node running
    rospy.loginfo(f"cmd_vel_to_twiststamped node started. Translating {args.input_topic} to {args.output_topic}")
    rospy.spin()
