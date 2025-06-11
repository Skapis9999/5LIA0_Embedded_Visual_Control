#!/usr/bin/env python2


import rospy
from std_msgs.msg import String
import time
import math
import sys
import os
import yaml


# Add local folder 'motorStuffcalibration' to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
motion_api_path = os.path.join(current_dir, "motorStuffcalibration")
sys.path.append(motion_api_path)

from motion_api import MotionController

# === Load calibration constants ===
with open(current_dir+"/calibration.yaml", "r") as f:
    calib = yaml.safe_load(f)

WHEEL_RADIUS = calib["wheel_radius"]
WHEEL_BASELINE = calib["wheel_baseline"]
TICKS_PER_REV = calib["ticks_per_rev"]
GAIN = calib["gain"]
TRIM = calib["trim"]


class MotionSubscriberNode:
    def __init__(self):
        self.initialized = False
        self.node_name = "motion_subscriber_skapis_node"
        rospy.loginfo("Initializing Motion Subscriber Node........")
        rospy.init_node(self.node_name)

        self.motion = MotionController()

        self.subscriber = rospy.Subscriber(
            "/motion_skapis",
            String,
            self.subscriber_cb,
            queue_size=None,
        )

        self.total_received = 0
        self.initialized = True
        rospy.loginfo("Motion Subscriber Node initialized!")

    def subscriber_cb(self, msg):
        if not self.initialized:
            return

        command_str = msg.data.strip()
        self.total_received += 1
        rospy.loginfo("Received command: '{}', Total received: {}".format(command_str, self.total_received))

        try:
            command, value_str = command_str.split()
            value = float(value_str)

            if command == "forward":
                # expected_ticks = TICKS_PER_REV * value / (2 * math.pi * WHEEL_RADIUS) 
                rospy.loginfo("Executing: move_forward({})".format(value))
                # rospy.loginfo("Expected ticks are({:.2f})".format(expected_ticks))
                # Get starting ticks
                start_left = self.motion.encoder_left._ticks
                start_right = self.motion.encoder_right._ticks

                self.motion.move_forward(value, speed=0.3)
                # Get final ticks
                end_left = self.motion.encoder_left._ticks
                end_right = self.motion.encoder_right._ticks

                delta_left = end_left - start_left
                delta_right = end_right - start_right
                avg = (delta_left + delta_right) / 2.0
                expected = self.motion.distance_to_ticks(value)

                print("\n=== Encoder Result ===")
                print("Left ticks:  ", delta_left)
                print("Right ticks: ", delta_right)
                print("Average:     ", avg)
                print("Expected ticks: ", expected)

            elif command == "turn":
                angle_rad = math.radians(value)
                arc = (WHEEL_BASELINE * abs(angle_rad)) / 2
                expected_ticks = TICKS_PER_REV * arc / (2 * math.pi * WHEEL_RADIUS)
                rospy.loginfo("Executing: turn({})".format(value))
                rospy.loginfo("Expected ticks are({:.2f})".format(expected_ticks))
                self.motion.turn(value, speed=0.3)
            else:
                rospy.logwarn("Unknown command: {}".format(command))
        except Exception as e:
            rospy.logerr("Error parsing command '{}': {}".format(command_str, str(e)))

        time.sleep(0.5)

if __name__ == "__main__":
    try:
        node = MotionSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
