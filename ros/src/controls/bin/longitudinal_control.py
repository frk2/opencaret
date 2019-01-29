#!/usr/bin/python
import time
from controls.PI import PI
from opencaret_msgs.msg import LongitudinalPlan
from std_msgs.msg import Float32, Bool
import math
from util import util
import numpy as np
import rospy

PLAN_LOOKAHEAD_INDEX = 3
TIME_STEP = 0.2
MAX_THROTTLE = 0.4
MAX_BRAKE = 0.5
MAX_PLANNER_DELAY = 1.0  # after 1.0s of no plan, consider the planner dead.
THROTTLE_FILTER = 0.9
BRAKE_FILTER = 0.9

RATE = 50.0

class CONTROL_MODE:
    ACCELERATE = 1,
    BRAKE = 2


class LongitudinalController():
    kP = 0.1
    kI = 0.01
    kF = 0.13
    DEADBAND_ACCEL = 0.05
    DEADBAND_BRAKE = -0.05

    def __init__(self):
        self.ego_accel = 0
        self.ego_velocity = 0
        self.brake_output = 0.0
        self.throttle_output = 0.0
        self.target_throttle = 0.0
        self.target_brake = 0.0

        self.mode = CONTROL_MODE.BRAKE
        self.pi = PI(self.kP, self.kI, self.kF, -MAX_BRAKE, MAX_THROTTLE)
        rospy.Subscriber('longitudinal_plan', LongitudinalPlan, self.on_plan)
        rospy.Subscriber("debug_target_speed", Float32,self.on_debug_target_speed)
        self.plan_deviation_pub = rospy.Publisher('/plan_deviation', Float32, queue_size=1)
        self.target_speed_pub = rospy.Publisher('pid_target_speed', Float32, queue_size=1)
        self.target_acc = rospy.Publisher('pid_target_accel', Float32, queue_size=1)
        self.p_pub = rospy.Publisher('pid_p', Float32, queue_size=1)
        self.ff_pub = rospy.Publisher('pid_ff', Float32, queue_size=1)
        self.i_pub = rospy.Publisher('pid_i', Float32, queue_size=1)
        rospy.Subscriber("wheel_speed", Float32, self.on_speed)
        self.throttle_pub = rospy.Publisher('/throttle_command', Float32, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_command', Float32, queue_size=1)
        self.last_plan_time = None
        self.controls_enabled = False
        self.plan = None
        self.acceleration_plan = None
        self.velocity_plan = None
        self.controls_enabled_sub = rospy.Subscriber('controls_enable', Bool, self.on_controls_enable)

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.pid_spin()
            rate.sleep()

    def set_target_throttle(self, target, force=False):
        self.target_throttle = target
        if force:
            self.throttle_output = target

    def set_target_brake(self, target, force=False):
        self.target_brake = target
        if force:
            self.brake_output = target

    def on_controls_enable(self, msg):
        self.controls_enabled = msg.data

    def on_speed(self, msg):
        self.ego_velocity = msg.data

    def on_plan(self, msg):
        self.plan = msg
        self.velocity_plan = np.array(self.plan.velocity).astype(np.float32)[PLAN_LOOKAHEAD_INDEX:]
        self.acceleration_plan = np.array(self.plan.accel).astype(np.float32)[PLAN_LOOKAHEAD_INDEX:]
        self.last_plan_time = time.time()

    def on_debug_target_speed(self, msg):
        self.set_target_speed(msg.data)

    def set_target_speed(self, target_vel):
        target_vel = max(0, target_vel)
        self.target_speed_pub.publish(Float32(data=target_vel))

    def on_imu(self, msg):
        self.ego_accel = msg.linear_acceleration.x

    def find_current_position_in_plan(self):
        dt = time.time() - self.last_plan_time
        closest_plan_index = min(len(self.velocity_plan) - 1, math.floor(dt / TIME_STEP))
        time_since_closest_plan_index = dt - closest_plan_index * TIME_STEP
        current_plan_deviation = self.velocity_plan[closest_plan_index].item() - self.ego_velocity
        return closest_plan_index,  time_since_closest_plan_index, current_plan_deviation

    def is_plan_stale(self):
        dt = time.time() - self.last_plan_time
        return dt > MAX_PLANNER_DELAY

    def pid_spin(self):
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
        output = self.pi.update(velocity, self.ego_velocity, acceleration)

        #  PI debug
        self.target_speed_pub.publish(Float32(data=velocity))
        self.target_acc.publish(Float32(data=acceleration))
        self.p_pub.publish(Float32(data=self.pi.P))
        self.ff_pub.publish(Float32(data=self.pi.FF))
        self.i_pub.publish(Float32(data=self.pi.I))

        if self.mode == CONTROL_MODE.BRAKE and self.ego_velocity <= 0.5 and velocity <= 0.5 and acceleration <= 0.1:
            rospy.logwarn("Brake clamping for stop")
            output = -0.25

        if output > self.DEADBAND_ACCEL:
            # print("Accelerating: {}".format(self.pid.output))
            if self.mode == CONTROL_MODE.BRAKE:
                self.pi.clear()
                self.mode = CONTROL_MODE.ACCELERATE
            self.set_target_brake(0.0)
            self.set_target_throttle(output)

        elif output < self.DEADBAND_BRAKE:
            if self.mode == CONTROL_MODE.ACCELERATE:
                self.pi.clear()
                self.mode = CONTROL_MODE.BRAKE
            # print("Braking: {}".format(self.pid.output))
            self.set_target_brake(-output + 0.05)
            self.set_target_throttle(0.0)
        else:
            if self.mode == CONTROL_MODE.BRAKE:
                self.set_target_brake(-min(0.0, output - 0.05))
            else:
                self.set_target_throttle(max(0.0, output))

        self.throttle_output = THROTTLE_FILTER * self.throttle_output + (1.0 - THROTTLE_FILTER) * self.target_throttle
        self.brake_output = BRAKE_FILTER * self.brake_output + (1.0 - BRAKE_FILTER) * self.target_brake
        self.throttle_pub.publish(Float32(data=self.throttle_output))
        self.brake_pub.publish(Float32(data=self.brake_output))


def main():
    rospy.init_node('longitudinal_control', anonymous=False, log_level=rospy.DEBUG)
    LongitudinalController()


if __name__ == '__main__':
    main()
