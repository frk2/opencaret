import time

import rclpy
from controls.PID import PID
from opencaret_msgs.msg import LongitudinalPlan
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import math
from util import util
import numpy as np

PLAN_LOOKAHEAD_INDEX = 2
TIME_STEP = 0.2
MAX_THROTTLE = 0.4
MAX_BRAKE = 0.3
MAX_PLANNER_DELAY = 1.0  # after 1.0s of no plan, consider the planner dead.

class CONTROL_MODE:
    ACCELERATE = 1,
    BRAKE = 2


class LongitudinalController(Node):
    kP = 0.1
    kI = 0.03
    DEADBAND_ACCEL = 0.05
    DEADBAND_BRAKE = -0.05

    def __init__(self):
        super().__init__('lateral_controler')
        self.ego_accel = 0
        self.ego_velocity = 0
        self.mode = CONTROL_MODE.BRAKE
        self.pid = PID(self.kP, self.kI, 0, -MAX_BRAKE, MAX_THROTTLE)
        self.create_subscription(LongitudinalPlan, 'longitudinal_plan', self.on_plan)
        self.create_subscription(Float32, "debug_target_speed", self.on_debug_target_speed)
        self.target_speed_pub = self.create_publisher(Float32, '/target_speed')
        self.plan_deviation_pub = self.create_publisher(Float32, '/plan_deviation')
        self.create_subscription(Float32, "wheel_speed", self.on_speed)
        self.throttle_pub = self.create_publisher(Float32, '/throttle_command')
        self.brake_pub = self.create_publisher(Float32, '/brake_command')
        self.last_plan_time = None
        self.controls_enabled = False
        self.plan = None
        self.acceleration_plan = None
        self.velocity_plan = None
        self.controls_enabled_sub = self.create_subscription(Bool, 'controls_enable', self.on_controls_enable)

        self.pid_timer = self.create_timer(1.0 / 50.0, self.pid_spin)

    def on_controls_enable(self, msg):
        self.controls_enabled = msg.data

    def on_speed(self, msg):
        self.ego_velocity = msg.data

    def on_plan(self, msg):
        self.plan = msg
        self.velocity_plan = np.array(self.plan.velocity).astype(np.float32)
        self.acceleration_plan = np.array(self.plan.accel).astype(np.float32)
        self.last_plan_time = time.time()

    def on_debug_target_speed(self, msg):
        self.set_target_speed(msg.data)

    def set_target_speed(self, target_vel):
        target_vel = max(0, target_vel)
        self.target_speed_pub.publish(Float32(data=target_vel))
        self.pid.SetPoint = target_vel

    def on_imu(self, msg):
        self.ego_accel = msg.linear_acceleration.x

    def find_current_position_in_plan(self):
        dt = time.time() - self.last_plan_time
        closest_plan_index = min(len(self.plan.velocity) - 1, math.floor(dt / TIME_STEP))
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
            velocity = 0.0

        self.pid.SetPoint = velocity

        self.target_speed_pub.publish(Float32(data=self.pid.SetPoint))
        self.plan_deviation_pub.publish(Float32(data=deviation))
        self.pid.update(self.ego_velocity)
        if self.pid.output > self.DEADBAND_ACCEL:
            # print("Accelerating: {}".format(self.pid.output))
            self.mode = CONTROL_MODE.ACCELERATE
            self.throttle_pub.publish(Float32(data=self.pid.output))
            self.brake_pub.publish(Float32(data=0.0))

        elif self.pid.output < self.DEADBAND_BRAKE:
            self.mode = CONTROL_MODE.BRAKE
            # print("Braking: {}".format(self.pid.output))
            self.brake_pub.publish(Float32(data=-self.pid.output))
            self.throttle_pub.publish(Float32(data=0.0))
        else:
            if self.mode == CONTROL_MODE.BRAKE:
                self.brake_pub.publish(Float32(data=-min(0.0, self.pid.output)))
            else:
                self.throttle_pub.publish(Float32(data=max(0.0, self.pid.output)))


def main():
    rclpy.init()
    controls = LongitudinalController()
    rclpy.spin(controls)
    controls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
