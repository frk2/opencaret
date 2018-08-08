from opencaret_msgs.msg import LeadVehicle, LongitudinalPlan
from std_msgs.msg import Float32
from rclpy.node import Node
import sys
import rclpy
import time
import math
from util import util

PLAN_LOOKAHEAD = 5
TIME_STEP = 0.1

class FakeEgo(Node):
    def __init__(self):
        super().__init__('fake_ego')
        self.plan = None
        self.last_plan_time = None
        self.plan_sub = self.create_subscription(LongitudinalPlan, 'longitudinal_plan', self.on_plan)
        self.wheel_speed_pub = self.create_publisher(Float32, 'wheel_speed')
        self.accel_pub = self.create_publisher(Float32, 'computed_accel')
        self.tick_timer = self.create_timer(1. / 30, self.tick)

    def on_plan(self, msg):
        self.plan = msg
        self.last_plan_time = time.time()

    def tick(self):
        if self.plan:
            dt = time.time() - self.last_plan_time
            closest_plan_index = math.floor(dt / TIME_STEP)
            time_since_closest_plan_index = dt - closest_plan_index * TIME_STEP
            acceleration = self.plan.accel[closest_plan_index]
            velocity = self.plan.velocity[closest_plan_index] + acceleration * time_since_closest_plan_index
            self.wheel_speed_pub.publish(Float32(data=util.ms_to_mph(velocity)))
            self.accel_pub.publish(Float32(data=acceleration))


def main():
    rclpy.init()
    fake_lead = FakeEgo()
    rclpy.spin(fake_lead)
    fake_lead.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

