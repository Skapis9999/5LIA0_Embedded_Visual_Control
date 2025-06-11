#!/usr/bin/env python2
import rospy, time, sys, os, math, yaml
from motorStuffcalibration.motorDriver import DaguWheelsDriver
from motorStuffcalibration.encoderDriver import WheelEncoderDriver
from std_msgs.msg import String

# === Load calibration constants ===
with open("/home/jetbot/EVC/workshops/workshop4_motion/src/motion/src/motorStuffcalibration/calibration.yaml", "r") as f:
    calib = yaml.safe_load(f)

WHEEL_RADIUS = calib["wheel_radius"]
WHEEL_BASELINE = calib["wheel_baseline"]
TICKS_PER_REV = calib["ticks_per_rev"]
GAIN = calib["gain"]
TRIM = calib["trim"]
GPIO_LEFT = 12
GPIO_RIGHT = 35

class FatMotionSubscriberNode:
    def __init__(self):
        self.initialized = False
        self.node_name = "fat_motion_subscriber_node"
        rospy.loginfo("Initializing Fat Motion Subscriber Node...")
        rospy.init_node(self.node_name)
        self.motor_driver = DaguWheelsDriver()
        self.encoder_left = WheelEncoderDriver(GPIO_LEFT)
        self.encoder_right = WheelEncoderDriver(GPIO_RIGHT)
        self.subscriber = rospy.Subscriber(
            "/motion_command",
            String,
            self.interpret_cmd,
            queue_size=None,
        )
        self.total_received = 0
        self.initialized = True
        rospy.loginfo("Motion Subscriber Node initialized!")

    def interpret_cmd(self, msg):
        if not self.initialized:
            return
    
        cmd = msg.data.strip()
        self.total_received += 1
        rospy.loginfo("Received command: '{}', Total received: {}".format(cmd, self.total_received))

        try:
            command, val = cmd.split()
            if command == "forward":
                self.move(float(val), speed=0.3)
            elif command == "turn":
                self.turn(float(val), speed=0.3)
            else:
                rospy.logwarn("Unknown command: {}".format(command))
        except Exception as e:
            rospy.logerr("Error parsing command '{}': {}".format(cmd, str(e)))
    
    def stop(self):
        self.motor_driver.set_wheels_speed(0, 0)

    def distance_to_ticks(self, distance):
        revs = distance / (2 * math.pi * WHEEL_RADIUS)
        return revs * TICKS_PER_REV

    def _adjusted_speeds(self, base_speed):
        left = base_speed * GAIN * (1 + TRIM)
        right = base_speed * GAIN * (1 - TRIM)
        return left, right

    def move(self, dist, speed=0.3):
        rospy.loginfo("Executing: move_forward({})".format(dist))
        target_ticks = self.distance_to_ticks(dist)
        start_left = self.encoder_left._ticks
        start_right = self.encoder_right._ticks
        left, right = self._adjusted_speeds(speed)

        self.motor_driver.set_wheels_speed(left, right)

        while True:
            left_ticks = self.encoder_left._ticks - start_left
            right_ticks = self.encoder_right._ticks - start_right
            avg_ticks = (left_ticks + right_ticks) / 2.0
            if avg_ticks >= target_ticks:
                break
            time.sleep(0.01)

        self.stop()

    def turn(self, angle_degrees, speed=0.3):
        """
        Turns the robot in place by the given angle (in degrees).
        Positive = right turn, Negative = left turn.
        """
        angle_rad = math.radians(angle_degrees)

        # Arc length each wheel must travel (in meters)
        arc = (WHEEL_BASELINE * abs(angle_rad)) / 2

        # Convert arc to encoder ticks
        target_ticks = self.distance_to_ticks(arc)
        rospy.loginfo("Expected ticks per wheel: {:.1f}".format(target_ticks))

        # Start encoder values
        start_left = self.encoder_left._ticks
        start_right = self.encoder_right._ticks

        # Use only positive speed internally
        abs_speed = abs(speed)
        left_cmd, right_cmd = self._adjusted_speeds(abs_speed)

        # Set direction based on turn direction
        if angle_degrees > 0:
            # Right turn: left wheel forward, right wheel backward
            self.motor_driver.set_wheels_speed(left_cmd, -right_cmd)
            rospy.loginfo("Turning RIGHT: L=+{:.2f}, R=-{:.2f}".format(left_cmd, right_cmd))
        else:
            # Left turn: left wheel backward, right wheel forward
            self.motor_driver.set_wheels_speed(-left_cmd, right_cmd)
            rospy.loginfo("Turning LEFT: L=-{:.2f}, R=+{:.2f}".format(left_cmd, right_cmd))

        # Wait for both wheels to hit target ticks
        while True:
            dl = abs(self.encoder_left._ticks - start_left)
            dr = abs(self.encoder_right._ticks - start_right)
            avg = (dl + dr) / 2.0

            if avg >= target_ticks:
                break
            time.sleep(0.01)

        self.stop()

        # Final result print
        rospy.loginfo("Actual ticks L/R: {}, {}".format(dl, dr))

if __name__ == "__main__":
    try:
        node = FatMotionSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
