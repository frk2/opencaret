#!/usr/bin/python
import time
from opencaret_msgs.msg import LongitudinalPlan, LongitudinalTarget
from std_msgs.msg import Float32, Bool
import math
from util import util
import numpy as np
import rospy

PLAN_LOOKAHEAD_INDEX = 3
TIME_STEP = 0.2
MAX_PLANNER_DELAY = 1.0  # after 1.0s of no plan, consider the planner dead.

RATE = 20.0

class LongitudinalController():
    def __init__(self):
        self.ego_accel = 0
        self.ego_velocity = 0

        rospy.Subscriber('longitudinal_plan', LongitudinalPlan, self.on_plan)
        self.plan_deviation_pub = rospy.Publisher('/plan_deviation', Float32, queue_size=1)
        self.long_target_pub = rospy.Publisher('/longitudinal_target', LongitudinalTarget, queue_size=1)
        self.target_speed_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)
        rospy.Subscriber("wheel_speed", Float32, self.on_speed)
        self.last_plan_time = None
        self.controls_enabled = False
        self.plan = None
        self.acceleration_plan = None
        self.velocity_plan = None
        self.controls_enabled_sub = rospy.Subscriber('controls_enable', Bool, self.on_controls_enable)

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.planner_spin()
            rate.sleep()

    def on_controls_enable(self, msg):
        self.controls_enabled = msg.data

    def on_speed(self, msg):
        self.ego_velocity = msg.data

    def on_plan(self, msg):
        self.plan = msg
        self.velocity_plan = np.array(self.plan.velocity).astype(np.float32)[PLAN_LOOKAHEAD_INDEX:]
        self.acceleration_plan = np.array(self.plan.accel).astype(np.float32)[PLAN_LOOKAHEAD_INDEX:]
        self.last_plan_time = time.time()

    def set_target_speed(self, target_vel):
        target_vel = max(0, target_vel)
        self.target_speed_pub.publish(Float32(data=target_vel))

    def on_imu(self, msg):
        self.ego_accel = msg.linear_acceleration.x

    def find_current_position_in_plan(self):
        dt = time.time() - self.last_plan_time
        closest_plan_index = int(min(len(self.velocity_plan) - 1, math.floor(dt / TIME_STEP)))
        time_since_closest_plan_index = dt - closest_plan_index * TIME_STEP
        current_plan_deviation = self.velocity_plan[closest_plan_index].item() - self.ego_velocity
        return closest_plan_index,  time_since_closest_plan_index, current_plan_deviation

    def is_plan_stale(self):
        dt = time.time() - self.last_plan_time
        return dt > MAX_PLANNER_DELAY

    def planner_spin(self):
        if not self.controls_enabled:
            return

        deviation = 0.0
        if self.plan:
            closest_plan_index, time_since_closest_plan_index, deviation = self.find_current_position_in_plan()
            acceleration = self.acceleration_plan[closest_plan_index].item()
            if not self.is_plan_stale():
                velocity = self.velocity_plan[closest_plan_index] + acceleration * time_since_closest_plan_index
            else:
                velocity = self.velocity_plan[-1].item()  # Try just going at the last velocity and hope for the best!
        else:
            acceleration = 0.0
            velocity = 0.0

        self.plan_deviation_pub.publish(Float32(data=deviation))
        control_msg = LongitudinalTarget()
        control_msg.speed = velocity
        control_msg.accel = acceleration
        self.long_target_pub.publish(control_msg)
        self.target_speed_pub.publish(Float32(data=velocity))

def main():
    rospy.init_node('longitudinal_control', anonymous=False, log_level=rospy.DEBUG)
    LongitudinalController()


if __name__ == '__main__':
    main()
