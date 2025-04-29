#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time

class Publisher:
    def __init__(self, node_name,frequency):
        self.initialized = False
        self.node_name = node_name
        self.frequency = frequency
        self.last_msg = 0         # Stores last published message. Starts at 0
        rospy.loginfo("Initializing ROS publishing node...")
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
        rospy.loginfo("Node initialized!")
        # frequency = 1.0 # Frequency in Herz. 1.0 is 1 Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.publish_time)

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

class Subscriber:
    def __init__(self, node_name,frequency):
        self.initialized = False
        self.node_name = node_name
        self.frequency = frequency
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
        rospy.loginfo("Node initialized!")
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
            rospy.loginfo("Received message ({seqNumber}): {payload}".format(
                seqNumber=self.recvSequence,
                payload=actual_value))
        else:
            if actual_value == predicted_value:
                rospy.loginfo("Received message ({seqNumber}): {payload} Prediction correct!".format(
                    seqNumber=self.recvSequence,
                    payload=actual_value))
            else:
                rospy.logwarn("Received message ({seqNumber}): {payload} Prediction failed! Expected: {expected}".format(
                    seqNumber=self.recvSequence,
                    payload=actual_value,
                    expected=predicted_value))

        self.last_received = actual_value
    
    # def publish_time(self, event):
    #     msg = UInt8()
    #     msg.data = self.last_msg   # Get last message

    #     # self.publisher.publish(msg)
    #     self.sendSequence += 1

    #     self.last_msg = msg.data # Store last published message
        
    #     rospy.loginfo("Message sent (({seqNumber})): {payload}".format(
    #         seqNumber=self.sendSequence,
    #         payload=msg.data))

    #     self.last_msg = (self.last_msg + 1) % 256 # Increment by one and wrap around at 256

class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.last_msg = 0         # Stores last published message. Starts at 0
        # self.predicted_value = None # Stores last predicted value. Starts with None
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
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
        rospy.loginfo("Node initialized!")
        frequency = 1.0 # Frequency in Herz. 1.0 is 1 Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/frequency), self.publish_time)

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
            rospy.loginfo("Received message ({seqNumber}): {payload}".format(
                seqNumber=self.recvSequence,
                payload=actual_value))
        else:
            if actual_value == predicted_value:
                rospy.loginfo("Received message ({seqNumber}): {payload} Prediction correct!".format(
                    seqNumber=self.recvSequence,
                    payload=actual_value))
            else:
                rospy.logwarn("Received message ({seqNumber}): {payload} Prediction failed! Expected: {expected}".format(
                    seqNumber=self.recvSequence,
                    payload=actual_value,
                    expected=predicted_value))

        self.last_received = actual_value
    
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
        # camera_node = Node(node_name = "Skapis_node")
        # rospy.init_node("Skapis_node", anonymous=True)

        publisher = Publisher(node_name = "Skapis_publisher", frequency = 1.0)
        subscriber = Subscriber(node_name = "Skapis_subscriber", frequency = 1.0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
