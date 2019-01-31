#!/usr/bin/python
import cantools
import oscc
import math
from opencaret_msgs.msg import CanMessage
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
from util import util
import rospy
import os
import struct

OSCC_MAGIC_NUMBER = 0xcc05
KIA_SOUL_STEERING_RATIO = 15.7
ACC_FILTER_FACTOR = 0.95
STEER_ACC_FILTER_FACTOR = 0.95


OSCC_DBC_PATH = os.path.join(oscc.__path__[1],"api","include","can_protocols")

class KiaSoulDriver():

    def __init__(self):
        self.enabled  = False
        self.last_velocity = None
        self.last_velocity_ts = None
        self.filtered_accel = 0
        self.can_pub = rospy.Publisher('/can_send', CanMessage, queue_size=1)
        self.last_steering_angle = None
        self.last_steering_angle_ts = None
        self.steering_accel = 0.0
        self.last_accel2 =  0.0
        self.speed_pub = rospy.Publisher('/wheel_speed', Float32, queue_size=1)
        self.accel_filtered_pub = rospy.Publisher('computed_accel_filtered', Float32, queue_size=1)
        self.accel_raw_pub = rospy.Publisher('computed_accel_raw', Float32, queue_size=1)
        self.steer_accel_pub = rospy.Publisher('/steering_accel', Float32, queue_size=1)
        self.steer_accel2_pub = rospy.Publisher('/steering_accel_2', Float32, queue_size=1)

        self.steering_wheel_angle_raw_pub = rospy.Publisher('/steering/wheel_angle/raw', Float32, queue_size=1)
        self.steering_angle_raw_pub = rospy.Publisher('/steering/yaw_angle/raw', Float32, queue_size=1)
        self.steering_joint_states_pub = rospy.Publisher('/steering/joint_states', JointState, queue_size=1)
        self.accel_pedal_pub = rospy.Publisher('/accel_pedal', Float32, queue_size=1)
        self.brake_pedal_pub = rospy.Publisher('/brake_pedal', Float32, queue_size=1)
        self.steering_torque = rospy.Publisher('/steering_torque', Float32, queue_size=1)
        self.kia_db = cantools.db.load_file(os.path.join(OSCC_DBC_PATH, 'kia_soul_ev.dbc'))
        self.oscc_db = cantools.db.load_file(os.path.join(OSCC_DBC_PATH, 'oscc.dbc'))

        self.throttle_cmd_sub = rospy.Subscriber('/throttle_command', Float32, self.on_throttle_cmd)
        self.brake_cmd_sub = rospy.Subscriber('/brake_command', Float32, self.on_brake_cmd)
        self.steering_sub = rospy.Subscriber('/steering_command', Float32, self.on_steering_cmd)
        self.controls_enable = rospy.Subscriber('/controls_enable', Bool, self.on_controls_enable)
        self.can_sub = rospy.Subscriber('/can_recv', CanMessage, self.on_can_message)
        self.file = open('/tmp/steering-data.csv', 'w')


    def on_can_message(self, msg):
        if msg.interface == CanMessage.CANTYPE_CONTROL:
            if msg.id in self.kia_db._frame_id_to_message:
                # Kia CAN messageSTEERING_ANGLE_angle
                kia_can_msg = self.kia_db.decode_message(msg.id, bytearray(msg.data))
                msg_type = self.kia_db.get_message_by_frame_id(msg.id)
                if msg_type.name == "STEERING_ANGLE":
                    steering_wheel_angle = float(kia_can_msg["STEERING_ANGLE_angle"]) * math.pi / 180.0
                    steering_wheel_angle_msg = Float32(data=steering_wheel_angle)
                    yaw_angle_msg = Float32(data=steering_wheel_angle/KIA_SOUL_STEERING_RATIO)
                    self.steering_wheel_angle_raw_pub.publish(steering_wheel_angle_msg)
                    self.steering_angle_raw_pub.publish(yaw_angle_msg)
                    joint_msg = JointState()
                    joint_msg.header.stamp = rospy.Time.from_seconds(msg.can_timestamp)
                    joint_msg.name=["steering_joint", "yaw", "front_left_steer_joint", "front_right_steer_joint"]
                    joint_msg.position = [
                        steering_wheel_angle,
                        steering_wheel_angle/KIA_SOUL_STEERING_RATIO,
                        steering_wheel_angle/KIA_SOUL_STEERING_RATIO,
                        steering_wheel_angle/KIA_SOUL_STEERING_RATIO
                    ]
                    self.steering_joint_states_pub.publish(joint_msg)
                    self.calc_steering_accel(steering_wheel_angle, msg.can_timestamp)
                elif msg_type.name == "SPEED":
                    # print(kia_can_msg)
                    speed = util.mph_to_ms(float(kia_can_msg["SPEED_rear_left"]))
                    self.on_speed(speed, msg.can_timestamp)
            elif msg.id in self.oscc_db._frame_id_to_message:
                # OSCC Message. Currently this only publishes 0 or 1 to indicate
                # enabled or not. In the future this should be changed to
                # throttle/brake/steering values but that requires a firmware change
                # to the OSCC
                oscc_can_msg = self.oscc_db.decode_message(msg.id, bytearray(msg.data))
                msg_type = self.oscc_db.get_message_by_frame_id(msg.id)

                # if oscc_can_msg.name == "BRAKE_REPORT":
                #     # self.brake_pedal_pub.publish(oscc_can_msg.brake_report_enabled)
                # elif oscc_can_msg.name == "STEERING_REPORT":
                #     self.steering_torque.publish(oscc_can_msg.steering_report_enabled)
                # el
                if msg_type.name == "STEERING_REPORT":
                    _,_,_,torque = struct.unpack_from("hccf", msg.data)
                    self.steering_torque.publish(Float32(data=torque / 12.7))
                    self.file.write("{},{},{},{}\n".format(torque / 12.7, self.steering_accel, self.last_steering_angle,
                                                           self.last_accel2))
                    self.file.flush()
                    # self.accel_pedal_pub.publish(oscc_can_msg.throttle_report_enabled)

    def calc_steering_accel(self, steering, ts):
        if self.last_steering_angle is None:
            self.last_steering_angle = steering
            self.last_steering_angle_ts = ts
            return
        if ts > self.last_steering_angle_ts:
            steer_accel = (steering - self.last_steering_angle) / (ts - self.last_steering_angle_ts)
            last_accel = self.steering_accel
            self.steering_accel = STEER_ACC_FILTER_FACTOR * self.steering_accel + (1 - STEER_ACC_FILTER_FACTOR) * steer_accel
            accel_2 = (self.steering_accel - last_accel) / (ts - self.last_steering_angle_ts)
            self.steer_accel_pub.publish(Float32(data=self.steering_accel))
            self.steer_accel2_pub.publish(Float32(data=accel_2))
            self.last_accel2 = accel_2
        self.last_steering_angle_ts = ts
        self.last_steering_angle = steering

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

    def on_steering_cmd(self, msg):
        if not self.enabled:
            return

        brake_oscc_msg = self.oscc_db.get_message_by_name("STEERING_COMMAND")
        self.can_pub.publish(CanMessage(id=brake_oscc_msg.frame_id,
                                        interface=CanMessage.CANTYPE_CONTROL,
                                        data=brake_oscc_msg.encode({
                                            'steering_command_magic': OSCC_MAGIC_NUMBER,
                                            'steering_command_torque_request': msg.data,
                                            'steering_command_reserved': 0
                                        })))

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
                ("STEERING_ENABLE", {
                    'steering_enable_magic': OSCC_MAGIC_NUMBER,
                    'steering_enable_reserved': 0
                }),
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
    rospy.init_node('kia_sour_driver', anonymous=False, log_level=rospy.DEBUG)
    # Get the parameters for the LLC node.
    KiaSoulDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
