#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time


class Publisher:
    def __init__(self, node_name,frequency=10.0):
        self.initialized = False
        self.node_name = node_name
        self.frequency = frequency
        self.last_msg = 0         # Stores last published message. Starts at 0
        rospy.loginfo("Initializing ROS publishing node with frequency... {freq} Hz".format(
            freq=self.frequency))
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/Skapis_endpoint",
            UInt8,         
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.recvSequence = 0
        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Publisher Node initialized!... {freq} Hz".format(
            freq=self.frequency))
        # frequency = 1.0 # Frequency in Herz. 1.0 is 1 Hz
        self.timer = rospy.Timer(rospy.Duration(1/self.frequency), self.publish_time)

    def publish_time(self, event):
        msg = UInt8()
        msg.data = self.last_msg   # Get last message

        self.publisher.publish(msg)
        self.sendSequence += 1

        self.last_msg = msg.data # Store last published message
        
        rospy.loginfo("Message sent (({seqNumber})): {payload}".format(
            seqNumber=self.sendSequence,
            payload=msg.data))

        self.last_msg = (self.last_msg + 1) % 256 # Increment by one and wrap around at 256


if __name__ == "__main__":
    try:
        publisher = Publisher(node_name = "Skapis_publisher", frequency = 10.0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
