import rclpy
import cantools
import oscc
import os
from rclpy.node import Node
from opencaret_msgs.msg import CanMessage
from std_msgs.msg import Float32, Bool
import time
from util import util

OSCC_MAGIC_NUMBER = 0xcc05

ACC_FILTER_FACTOR = 0.95

class KiaSoulDriver(Node):

    def __init__(self):
        super(KiaSoulDriver, self).__init__('kiasouldriver')

        self.enabled  = False
        self.last_velocity = None
        self.last_velocity_ts = None
        self.filtered_accel = 0
        self.can_sub = self.create_subscription(CanMessage, 'can_recv', self.on_can_message)
        self.can_pub = self.create_publisher(CanMessage, 'can_send')

        self.speed_pub = self.create_publisher(Float32, 'wheel_speed')
        self.accel_filtered_pub = self.create_publisher(Float32, 'computed_accel_filtered')
        self.accel_raw_pub = self.create_publisher(Float32, 'computed_accel_raw')
        self.steering_angle_pub = self.create_publisher(Float32, 'steering_angle')
        self.accel_pedal_pub = self.create_publisher(Float32, 'accel_pedal')
        self.brake_pedal_pub = self.create_publisher(Float32, 'brake_pedal')
        self.steering_torque = self.create_publisher(Float32, 'steering_torque')

        self.throttle_cmd_sub = self.create_subscription(Float32, 'throttle_command', self.on_throttle_cmd)
        self.brake_cmd_sub = self.create_subscription(Float32, 'brake_command', self.on_brake_cmd)
        self.controls_enable = self.create_subscription(Bool, 'controls_enable', self.on_controls_enable)

        self.kia_db = cantools.db.load_file(os.path.join(oscc.OSCC_PATH, 'kia_soul_ev.dbc'))
        self.oscc_db = cantools.db.load_file(os.path.join(oscc.OSCC_PATH, 'oscc.dbc'))


    def on_can_message(self, msg):
        if msg.interface == CanMessage.CANTYPE_CONTROL:
            if msg.id in self.kia_db._frame_id_to_message:
                # Kia CAN messageSTEERING_ANGLE_angle
                kia_can_msg = self.kia_db.decode_message(msg.id, bytearray(msg.data))
                msg_type = self.kia_db.get_message_by_frame_id(msg.id)
                if msg_type.name == "STEERING_ANGLE":
                    self.steering_angle_pub.publish(Float32(data=float(kia_can_msg["STEERING_ANGLE_angle"])))
                elif msg_type.name == "SPEED":
                    # print(kia_can_msg)
                    speed = util.mph_to_ms(float(kia_can_msg["SPEED_rear_left"]))
                    self.on_speed(speed, msg.can_timestamp)
            elif msg.id in self.oscc_db._frame_id_to_message:
                # OSCC Message. Currently this only publishes 0 or 1 to indicate
                # enabled or not. In the future this should be changed to
                # throttle/brake/steering values but that requires a firmware change
                # to the OSCC
                return
                oscc_can_msg = self.oscc_db.decode_message(msg.id, bytearray(msg.data))
                if oscc_can_msg.name == "BRAKE_REPORT":
                    self.brake_pedal_pub.publish(oscc_can_msg.brake_report_enabled)
                elif oscc_can_msg.name == "STEERING_REPORT":
                    self.steering_torque.publish(oscc_can_msg.steering_report_enabled)
                elif oscc_can_msg.name == "THROTTLE_REPORT":
                    self.accel_pedal_pub.publish(oscc_can_msg.throttle_report_enabled)

    def on_speed(self, speed, ts):
        if self.last_velocity is None:
            self.last_velocity = speed
            self.last_velocity_ts = ts
            return

        if ts > self.last_velocity_ts:
            self.speed_pub.publish(Float32(data=speed))
            accel = (speed - self.last_velocity) / (ts - self.last_velocity_ts)
            self.filtered_accel = ACC_FILTER_FACTOR * self.filtered_accel + (1 - ACC_FILTER_FACTOR) * accel
            self.accel_filtered_pub.publish(Float32(data=self.filtered_accel))
            self.accel_raw_pub.publish(Float32(data=accel))

        self.last_velocity_ts = ts
        self.last_velocity = speed

    def on_throttle_cmd(self, msg):
        if not self.enabled:
            return


        throttle_oscc_msg = self.oscc_db.get_message_by_name("THROTTLE_COMMAND")
        encoded_msg = throttle_oscc_msg.encode({
            'throttle_command_magic': OSCC_MAGIC_NUMBER,
            'throttle_command_pedal_request': msg.data,
            'throttle_command_reserved' : 0
        })
        self.can_pub.publish(CanMessage(id=throttle_oscc_msg.frame_id,
                                        interface=CanMessage.CANTYPE_CONTROL,
                                        data=encoded_msg))

    def on_brake_cmd(self, msg):
        if not self.enabled:
            return

        brake_oscc_msg = self.oscc_db.get_message_by_name("BRAKE_COMMAND")
        self.can_pub.publish(CanMessage(id=brake_oscc_msg.frame_id,
                                        interface=CanMessage.CANTYPE_CONTROL,
                                        data=brake_oscc_msg.encode({
                                            'brake_command_magic': OSCC_MAGIC_NUMBER,
                                            'brake_command_pedal_request': msg.data,
                                            'brake_command_reserved': 0
                                        })))

    def on_controls_enable(self, msg):
        self.oscc_enabled(msg.data)
        self.enabled = msg.data

    def oscc_enabled(self, enable):
        msgs = None
        if enable:
            msgs = [
                ("BRAKE_ENABLE", {
                    'brake_enable_magic': 0xcc05,
                    'brake_enable_reserved': 0
                }),
                ("THROTTLE_ENABLE", {
                    'throttle_enable_magic': OSCC_MAGIC_NUMBER,
                    'throttle_enable_reserved': 0
                }),
                # ("STEERING_ENABLE", {
                #     'steering_enable_magic': OSCC_MAGIC_NUMBER,
                #     'steering_enable_reserved': 0
                # }),
            ]
        else:
            msgs = [
                ("BRAKE_DISABLE", {
                    'brake_disable_magic': OSCC_MAGIC_NUMBER,
                    'brake_disable_reserved': 0
                }),
                ("THROTTLE_DISABLE", {
                    'throttle_disable_magic': OSCC_MAGIC_NUMBER,
                    'throttle_disable_reserved': 0
                }),
                ("STEERING_DISABLE", {
                    'steering_disable_magic': OSCC_MAGIC_NUMBER,
                    'steering_disable_reserved': 0
                })
            ]

        for name, data in msgs:
            msg = self.oscc_db.get_message_by_name(name)
            self.can_pub.publish(CanMessage(id=msg.frame_id,
                                        interface=CanMessage.CANTYPE_CONTROL,
                                        data=msg.encode(data)))


def main():
    rclpy.init()
    decoder = KiaSoulDriver()
    rclpy.spin(decoder)
    decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
