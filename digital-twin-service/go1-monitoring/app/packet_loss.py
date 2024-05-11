import rospy
from rosgraph_msgs.msg import TopicStatistics

def statistics_callback(msg):
    if msg.topic == "/scan" or msg.topic == "/joint_states":
        print(f"Dropped messages for topic '{msg.topic}': {msg.dropped_msgs}")
        # print(f"Maximum stamp age for topic '{msg.topic}': {msg.stamp_age_max.secs}")

if __name__ == "__main__":
    rospy.init_node('statistics_reader')
    
    rospy.Subscriber("/statistics", TopicStatistics, statistics_callback)
    
    rospy.spin()  # Keeps the node running until it's explicitly shut down
