import rclpy
import cantools
import oscc
import os
from rclpy.node import Node
from opencaret_msgs.msg import CanMessage
from std_msgs.msg import Float32

class KiaSoulDriver(Node):
    
    def __init__(self):
        super(KiaSoulDriver, self).__init__('kiasouldriver')
        self.can_sub = self.create_subscription(CanMessage, 'can_recv', self.on_can_message)
        self.can_pub = self.create_publisher(CanMessage, 'can_send')

        self.speed_pub = self.create_publisher(Float32, 'wheel_speed')
        self.steering_angle_pub = self.create_publisher(Float32, 'steering_angle')
        self.accel_pedal_pub = self.create_publisher(Float32, 'accel_pedal')
        self.brake_pedal_pub = self.create_publisher(Float32, 'brake_pedal')
        self.steering_torque = self.create_publisher(Float32, 'steering_torque')

        self.throttle_cmd_sub = self.create_subscription(Float32, 'throttle_cmd', self.on_throttle_cmd)
        self.brake_cmd_sub = self.create_subscription(Float32, 'brake_cmd', self.on_brake_cmd)

        self.kia_db = cantools.db.load_file(os.path.join(oscc.OSCC_PATH, 'kia_soul_ev.dbc'))
        self.oscc_db = cantools.db.load_file(os.path.join(oscc.OSCC_PATH, 'oscc.dbc'))

    def on_can_message(self, msg):
        if msg.interface == CanMessage.CANTYPE_CONTROL:
            if msg.id in self.kia_db._frame_id_to_message:
                # Kia CAN message
                kia_can_msg = self.kia_db.decode_message(msg.id, bytearray(msg.data))
                if kia_can_msg.name == "STEERING_ANGLE":
                    self.steering_angle_pub.publish(kia_can_msg.angle)
                elif kia_can_msg.name == "SPEED":
                    self.speed_pub.publish(kia_can_msg.rear_left)
            elif msg.id in self.oscc_db._frame_id_to_message:
                # OSCC Message. Currently this only publishes 0 or 1 to indicate
                # enabled or not. In the future this should be changed to
                # throttle/brake/steering values but that requires a firmware change
                # to the OSCC

                oscc_can_msg = self.oscc_db.decode_message(msg.id, bytearray(msg.data))
                if oscc_can_msg.name == "BRAKE_REPORT":
                    self.brake_pedal_pub.publish(oscc_can_msg.brake_report_enabled)
                elif oscc_can_msg.name == "STEERING_REPORT":
                    self.steering_torque.publish(oscc_can_msg.steering_report_enabled)
                elif oscc_can_msg.name == "THROTTLE_REPORT":
                    self.accel_pedal_pub.publish(oscc_can_msg.throttle_report_enabled)

    def on_throttle_cmd(self, msg):
        throttle_oscc_cmd = self.oscc_db.get_message_by_name("THROTTLE_COMMAND").encode({
            'throttle_command_pedal_request': msg.data
        })
        self.can_pub.publish(CanMessage(id=throttle_oscc_cmd.frame_id,
                                        interface=CanMessage.CANTYPE_CONTROL,
                                        data=throttle_oscc_cmd))
    def on_brake_cmd(self, msg):
        brake_oscc_cmd = self.oscc_db.get_message_by_name("BRAKE_COMMAND").encode({
            'brake_command_pedal_request': msg.data
        })
        self.can_pub.publish(CanMessage(id=brake_oscc_cmd.frame_id,
                                        interface=CanMessage.CANTYPE_CONTROL,
                                        data=brake_oscc_cmd))

def main():
    rclpy.init()
    decoder = KiaSoulDriver()
    rclpy.spin(decoder)
    decoder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()