#!/usr/bin/python

import time
from controls.PI import PI
from opencaret_msgs.msg import LongitudinalPlan
from std_msgs.msg import Float32, Bool
import math
from util import util
import numpy as np
import rospy
import pickle

MAX_STEERING_ANGLE = 0.4

STEER_FILTER = 0.95
RATE = 20.0

class CONTROL_MODE:
    TURNING = 1,
    STRAIGHTENING = 2


class LateralController():
    kP = 0.001
    kI = 0.00001
    kF = 0.0001
    DEADBAND = 0.05

    def __init__(self):
        self.ego_velocity = 0
        self.steering_angle = 0.0
        self.cte = 0.0
        self.curvature = 0.0
        self.steering_output = 0.0
        self.mode = CONTROL_MODE.STRAIGHTENING
        self.pi = PI(self.kP, self.kI, self.kF, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
        self.p_pub = rospy.Publisher('spid_p', Float32, queue_size=1)
        self.ff_pub = rospy.Publisher('spid_ff', Float32, queue_size=1)
        self.i_pub = rospy.Publisher('spid_i', Float32, queue_size=1)
        rospy.Subscriber("wheel_speed", Float32, self.on_speed)
        rospy.Subscriber("/steering/wheel_angle/raw", Float32, self.on_wheel_angle)
        rospy.Subscriber("/cte", Float32, self.on_cte)
        rospy.Subscriber("/curvature", Float32, self.on_curvature)

        self.steering_pub = rospy.Publisher('/steering_angle_target', Float32, queue_size=1)
        self.controls_enabled = True

        self.controls_enabled_sub = rospy.Subscriber('controls_enable', Bool, self.on_controls_enable)

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.pid_spin()
            rate.sleep()

    def on_target_steering_angle(self, target):
        self.steering_angle = target.data

    def on_wheel_angle(self, angle):
        self.current_steering_angle = angle.data

    def on_cte(self, cte):
        self.cte = cte.data

    def on_curvature(self, curvature):
        self.curvature = curvature.data

    def on_controls_enable(self, msg):
        if not self.controls_enabled and msg.data:
            self.pi.clear()
        self.controls_enabled = msg.data

    def on_speed(self, msg):
        self.ego_velocity = msg.data

    def pid_spin(self):
        if not self.controls_enabled:
            return

        ff = self.curvature
        output = self.pi.update(0, self.cte, ff)

        output = min(MAX_STEERING_ANGLE, max(-MAX_STEERING_ANGLE, output))

        #  PI debug
        self.p_pub.publish(Float32(data=self.pi.P))
        self.ff_pub.publish(Float32(data=self.pi.FF))
        self.i_pub.publish(Float32(data=self.pi.I))

        self.steering_output = STEER_FILTER * self.steering_output + (1.0 - STEER_FILTER) * output
        self.steering_pub.publish(Float32(data=-self.steering_output))


def main():
    rospy.init_node('lateral_control', anonymous=False, log_level=rospy.DEBUG)
    LateralController()


if __name__ == '__main__':
    main()