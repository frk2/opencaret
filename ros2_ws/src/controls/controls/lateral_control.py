import rclpy
from rclpy.node import Node
from controls.PID import PID
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from opencaret_msgs.msg import LongitudinalPlan

PLAN_LOOKAHEAD_INDEX = 5  # 1.0s lookahead since dt==0.2 in planner

class CONTROL_MODE:
  ACCELERATE = 1,
  BRAKE = 2


class LateralController(Node):
  kP = 0.05
  kI = 0.05
  DEADBAND_ACCEL = 0.05
  DEADBAND_BRAKE = -0.05

  def __init__(self):
    super(LateralController, self).__init__('lateral_controler')
    self.ego_accel = 0
    self.last_mode = CONTROL_MODE.BRAKE
    self.pid = PID(self.kP, self.kI, 0, -0.5, 0.5)
    self.create_subscription(LongitudinalPlan, 'plan', self.on_plan)
    self.create_subscription(Imu, '/imu', self.on_imu)
    self.throttle_pub =  self.create_publisher(Float32, 'throttle_cmd')
    self.brake_pub = self.create_publisher(Float32, 'brake_cmd')

    self.pid_timer = self.create_timer(1.0 / 30, self.pid_spin)

  def on_plan(self, msg):
    target_acceleration = msg.accel[PLAN_LOOKAHEAD_INDEX]
    self.pid.SetPoint = target_acceleration
    if self.pid.SetPoint > 0 and self.last_mode == CONTROL_MODE.BRAKE:
      self.last_mode = CONTROL_MODE.ACCELERATE
      self.pid.clear()
    elif self.pid.SetPoint < 0 and self.last_mode == CONTROL_MODE.ACCELERATE:
      self.last_mode = CONTROL_MODE.BRAKE
      self.pid.clear()

  def on_imu(self, msg):
    self.ego_accel = msg.linear_acceleration.x

  def pid_spin(self):
    self.pid.update(self.ego_accel)
    if self.pid.output > self.DEADBAND_ACCEL:
      self.throttle_pub.publish(Float32(data=self.pid.output))
      self.brake_pub.publish(Float32(data=0.0))

    elif self.pid.output < self.DEADBAND_BRAKE:
      self.brake_pub.publish(Float32(data=-self.pid.output))
      self.throttle_pub.publish(Float32(data=0.0))

def main():
  rclpy.init()
  controls = LateralController()
  rclpy.spin(controls)
  controls.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()


