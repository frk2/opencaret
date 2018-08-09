from opencaret_msgs.msg import LeadVehicle
from std_msgs.msg import Float32
from rclpy.node import Node
import sys
import rclpy
import time
from util import util


class FakeLeadVehicle(Node):
    def __init__(self, distance, velocity, accel):
        super().__init__('fake_lead_vehicle')
        self.distance = float(distance)
        self.velocity = util.mph_to_ms(float(velocity))
        self.accel = float(accel)
        self.time_since_last_update = None
        self.last_ego_speed = None
        self.current_ego_speed = 0
        self.lead_vehicle_pub = self.create_publisher(LeadVehicle, '/lead_vehicle')
        self.lead_vehicle_velocity = self.create_publisher(Float32, '/lead_vehicle_velocity')
        self.lead_vehilce_distance = self.create_publisher(Float32, '/lead_vehicle_distance')
        self.velocity_sub = self.create_subscription(Float32, 'wheel_speed', self.on_wheel_speed)
        self.tick_timer = self.create_timer(1. /10, self.tick)

    def tick(self):
        if self.time_since_last_update is None:
            self.time_since_last_update = time.time()
            return

        if self.last_ego_speed is None:
            self.last_ego_speed = self.current_ego_speed
        dt = time.time() - self.time_since_last_update
        self.time_since_last_update = time.time()
        ego_distance_travelled =  (self.last_ego_speed + self.current_ego_speed) / 2.0 * dt
        self.last_ego_speed = self.current_ego_speed
        lead_distance_travelled = self.velocity * dt + 0.5 * self.accel ** dt

        self.distance += lead_distance_travelled - ego_distance_travelled
        self.velocity += self.accel * dt
        lead_vehicle_msg = LeadVehicle()
        lead_vehicle_msg.distance = self.distance
        lead_vehicle_msg.velocity = self.velocity
        lead_vehicle_msg.accel = self.accel
        print(lead_vehicle_msg)
        self.lead_vehicle_pub.publish(lead_vehicle_msg)
        self.lead_vehicle_velocity.publish(Float32(data=self.velocity))
        self.lead_vehilce_distance.publish(Float32(data=self.distance))

    def on_wheel_speed(self, msg):
        self.current_ego_speed = float(msg.data)



def main():
    rclpy.init()
    fake_lead = FakeLeadVehicle(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin(fake_lead)
    fake_lead.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()







