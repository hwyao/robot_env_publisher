#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
from dynamic_reconfigure.server import Server
from robot_env_publisher.cfg import TimeControlConfig  
import time  

class ClockPublisherWithSlideBar:
    def __init__(self):
        rospy.init_node('clock_publisher_slidebar')  # Initialize the ROS node
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
        self.simulated_time = rospy.Time(0, 0)  # Start at time 0
        self.time_offset = 0.0         # Time offset controlled by the slide bar
        self.enable_clock_run = False  # Checkbox state

        # Dynamic reconfigure server to adjust parameters
        self.server = Server(TimeControlConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.time_offset = config['time_offset']
        self.enable_clock_run = config['enable_clock_run']
        return config

    def run(self):
        while not rospy.is_shutdown():
            try:
                clock_msg = Clock()
                if self.enable_clock_run:
                    self.simulated_time = self.simulated_time + rospy.Duration.from_sec(0.001)
                else:
                    self.simulated_time = rospy.Time.from_sec(self.time_offset)

                clock_msg.clock = self.simulated_time
                self.clock_pub.publish(clock_msg)

                # Manual sleep for 1 millisecond
                time.sleep(0.001)

                rospy.loginfo_throttle(1, "[clock_publisher_slidebar] Publishing clock time: %s", self.simulated_time.to_sec())
            except Exception as e:
                rospy.logerr(f"[clock_publisher_slidebar] Error occurred: {e}")
                break

if __name__ == '__main__':
    try:
        node = ClockPublisherWithSlideBar()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted and shutting down.")
