#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
import time 

class MyNode:
    def __init__(self):
        rospy.init_node('publish_clock')  # Initialize the ROS node
        rospy.loginfo("[publish_clock] Node initialized successfully.")

    def run(self):
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
        simulated_time = rospy.Time(0, 0)  # Start at time 0
        rospy.loginfo("[publish_clock] Clock publisher initialized.")

        while not rospy.is_shutdown():
            clock_msg = Clock()
            clock_msg.clock = simulated_time
            clock_pub.publish(clock_msg)

            # Increment simulated time by 1 millisecond (1000 Hz)
            simulated_time = simulated_time + rospy.Duration.from_sec(0.001)

            # Throttled log to avoid spamming
            rospy.loginfo_throttle(1, "[publish_clock] Publishing clock time: %s", simulated_time.to_sec())

            # Manual sleep for 1 millisecond
            time.sleep(0.001)

if __name__ == '__main__':
    try:
        node = MyNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted and shutting down.")