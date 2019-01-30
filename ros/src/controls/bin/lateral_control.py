#!/usr/bin/python

import time
from controls.PI import PI
from opencaret_msgs.msg import LongitudinalPlan
from std_msgs.msg import Float32, Bool
import math
from util import util
import numpy as np
import rospy



MAX_STEERING = 0.3
STEER_FILTER = 0.95
RATE = 50.0

class CONTROL_MODE:
    TURNING = 1,
    STRAIGHTENING = 2


class LateralController():
    kP = 0.1
    kI = 0.01
    kF = 0.05
    DEADBAND = 0.05

    def __init__(self):
        self.ego_velocity = 0
        self.steering_output = 0.0
        self.target_steering_angle = 0.0
        self.current_steering_angle = 0.0
        self.mode = CONTROL_MODE.STRAIGHTENING
        self.pi = PI(self.kP, self.kI, self.kF, -MAX_STEERING, MAX_STEERING)
        self.p_pub = rospy.Publisher('spid_p', Float32, queue_size=1)
        self.ff_pub = rospy.Publisher('spid_ff', Float32, queue_size=1)
        self.i_pub = rospy.Publisher('spid_i', Float32, queue_size=1)
        rospy.Subscriber("wheel_speed", Float32, self.on_speed)
        rospy.Subscriber("/steering/wheel_angle/raw", Float32, self.on_wheel_angle)
        rospy.Subscriber("/target_steering_angle", Float32, self.on_target_steering_angle)
        self.steering_pub = rospy.Publisher('/steering_command', Float32, queue_size=1)
        self.last_request_angle_time = None
        self.controls_enabled = False
        self.controls_enabled_sub = rospy.Subscriber('controls_enable', Bool, self.on_controls_enable)

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.pid_spin()
            rate.sleep()

    def on_target_steering_angle(self, target):
        self.target_steering_angle = target.data

    def on_wheel_angle(self, angle):
        self.current_steering_angle = angle.data

    def on_controls_enable(self, msg):
        self.controls_enabled = msg.data

    def on_speed(self, msg):
        self.ego_velocity = msg.data

    def pid_spin(self):
        if not self.controls_enabled:
            return
        ff = 1.0 / (1. + self.ego_velocity)
        output = self.pi.update(self.target_steering_angle, self.current_steering_angle, ff)

        #  PI debug
        self.p_pub.publish(Float32(data=self.pi.P))
        self.ff_pub.publish(Float32(data=self.pi.FF))
        self.i_pub.publish(Float32(data=self.pi.I))

        self.steering_output = STEER_FILTER * self.steering_output + (1.0 - STEER_FILTER) * output
        self.steering_pub.publish(Float32(data=self.steering_output))


def main():
    rospy.init_node('lateral_control', anonymous=False, log_level=rospy.DEBUG)
    LateralController()


if __name__ == '__main__':
    main()