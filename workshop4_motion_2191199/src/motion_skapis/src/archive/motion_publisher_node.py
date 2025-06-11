#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class MotionPublisherNode:
    def __init__(self):
        self.initialized = False
        self.node_name = "motion_publisher_skapis_node"
        rospy.loginfo("Initializing Motion Publisher Node........")
        rospy.init_node(self.node_name)

        # Publisher
        self.publisher = rospy.Publisher(
            "/motion_skapis",
            String,
            queue_size=1,
        )

        # Motion command queue
        self.commands = [
            "forward 0.5",
            "turn 90",
            "forward 0.3",
            "turn -90",
            "forward 0.2"
        ]

        self.command_index = 0
        self.total_published = 0
        self.initialized = True

        rospy.loginfo("Motion Publisher Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(2.0), self.publish_command)  # send every 2 seconds

    def publish_command(self, event):
        if not self.initialized:
            return

        if self.command_index >= len(self.commands):
            rospy.loginfo("All commands sent. Idle publisher.")
            # rospy.signal_shutdown("Done publishing motion commands.")
            return

        cmd_str = self.commands[self.command_index]
        msg = String()
        msg.data = cmd_str
        self.publisher.publish(msg)

        self.total_published += 1
        rospy.loginfo("Published command: '{}', Total Published: {}".format(cmd_str, self.total_published))

        self.command_index += 1

if __name__ == "__main__":
    try:
        node = MotionPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
