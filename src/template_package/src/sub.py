#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time

class Subscriber:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        # self.frequency = frequency
        rospy.loginfo("Initializing ROS subscribing node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/Skapis_endpoint",
            UInt8,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=1,
        )

        self.recvSequence = 0
        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Subscriber Node initialized!")
        # frequency = 1.0 # Frequency in Herz. 1.0 is 1 Hz
        # self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.publish_time)

    def subscriber_cb(self, data):
        if not self.initialized:
            return

        if hasattr(self, 'last_received'):
            predicted_value = (self.last_received + 1) % 256
        else:
            predicted_value = None
        actual_value = data.data
        
        self.recvSequence += 1

        if predicted_value is None:
            rospy.logwarn("First message received ({seqNumber}): {payload}".format(
                seqNumber=self.recvSequence,
                payload=None))
        else:
            if actual_value == predicted_value:
                rospy.loginfo("Received message ({seqNumber}): {payload} Prediction correct!".format(
                    seqNumber=self.recvSequence,
                    payload=predicted_value))
            else:
                rospy.logwarn("Received message ({seqNumber}): {payload} Prediction failed! Expected: {expected}".format(
                    seqNumber=self.recvSequence,
                    payload=actual_value,
                    expected=predicted_value))
        self.last_received = actual_value


if __name__ == "__main__":
    try:
        subscriber = Subscriber(node_name = "Skapis_subscriber")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass