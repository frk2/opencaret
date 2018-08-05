import rclpy
from rclpy.node import Node
from controls.PID import PID
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32,Bool
from opencaret_msgs.msg import LongitudinalPlan
from util import util

PLAN_LOOKAHEAD_INDEX = 5  # 1.0s lookahead since dt==0.2 in planner
MAX_THROTTLE = 0.4
MAX_BRAKE = 0.3

class CONTROL_MODE:
  ACCELERATE = 1,
  BRAKE = 2


class LateralController(Node):
  kP = 0.1
  kI = 0.03
  DEADBAND_ACCEL = 0.05
  DEADBAND_BRAKE = -0.05

  def __init__(self):
    super().__init__('lateral_controler')
    self.ego_accel = 0
    self.ego_velocity = 0
    self.last_mode = CONTROL_MODE.BRAKE
    self.pid = PID(self.kP, self.kI, 0, -MAX_BRAKE, MAX_THROTTLE)
    self.create_subscription(LongitudinalPlan, 'longitudinal_plan', self.on_plan)
    self.create_subscription(Float32, "debug_target_speed", self.on_debug_target_speed)
    self.target_speed_pub = self.create_publisher(Float32, '/target_speed')
    self.create_subscription(Float32, "wheel_speed", self.on_speed)
    self.throttle_pub =  self.create_publisher(Float32, '/throttle_command')
    self.brake_pub = self.create_publisher(Float32, '/brake_command')

    self.controls_enabled = False
    self.controls_enabled_sub = self.create_subscription(Bool, 'controls_enable', self.on_controls_enable)


    self.pid_timer = self.create_timer(1.0 / 50.0, self.pid_spin)


  def on_controls_enable(self, msg):
    self.controls_enabled = msg.data

  def on_speed(self, msg):
    self.ego_velocity = msg.data

  def on_plan(self, msg):
    target_acceleration = msg.accel[PLAN_LOOKAHEAD_INDEX]
    target_vel = util.ms_to_mph(msg.velocity[PLAN_LOOKAHEAD_INDEX])

    self.set_target_speed(target_vel)

  def on_debug_target_speed(self, msg):
    self.set_target_speed(msg.data)

  def set_target_speed(self, target_vel):
    target_vel = max(0, target_vel)
    self.target_speed_pub.publish(Float32(data=target_vel))
    self.pid.SetPoint = target_vel
    # if self.pid.SetPoint >= self.ego_velocity and self.last_mode == CONTROL_MODE.BRAKE:
    #   self.last_mode = CONTROL_MODE.ACCELERATE
    #   self.pid.clear()
    # elif self.pid.SetPoint < self.ego_velocity and self.last_mode == CONTROL_MODE.ACCELERATE:
    #   self.last_mode = CONTROL_MODE.BRAKE
    #   self.pid.clear()

  def on_imu(self, msg):
    self.ego_accel = msg.linear_acceleration.x

  def pid_spin(self):
    if not self.controls_enabled:
      return

    self.pid.update(self.ego_velocity)
    if self.pid.output > self.DEADBAND_ACCEL:
      # print("Accelerating: {}".format(self.pid.output))
      self.throttle_pub.publish(Float32(data=self.pid.output))
      self.brake_pub.publish(Float32(data=0.0))

    elif self.pid.output < self.DEADBAND_BRAKE:
      # print("Braking: {}".format(self.pid.output))
      self.brake_pub.publish(Float32(data=-self.pid.output))
      self.throttle_pub.publish(Float32(data=0.0))
    else:
      self.brake_pub.publish(Float32(data=0.0))
      self.throttle_pub.publish(Float32(data=0.0))


def main():
  rclpy.init()
  controls = LateralController()
  rclpy.spin(controls)
  controls.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()


