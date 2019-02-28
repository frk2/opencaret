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
from canoc.can_transceiver import CanTransceiver
from util.util import SimpleTimedDiff
OSCC_MAGIC_NUMBER = 0xcc05
OSCC_STEERING_PID_COMMAND_MODE = 2
KIA_SOUL_STEERING_RATIO = 15.7
ACC_FILTER_FACTOR = 0.95
STEER_ACC_FILTER_FACTOR = 0

print(oscc.__path__)

OSCC_DBC_PATH = os.path.join(oscc.__path__[1],"api","include","can_protocols")

class KiaSoulDriver():

    def __init__(self):
        self.enabled  = False
        self.last_velocity = None
        self.last_velocity_ts = None
        self.filtered_accel = 0
        self.kia_db = cantools.db.load_file(os.path.join(OSCC_DBC_PATH, 'kia_soul_ev.dbc'))
        self.oscc_db = cantools.db.load_file(os.path.join(OSCC_DBC_PATH, 'oscc.dbc'))
        self.can_bus = CanTransceiver(CanTransceiver.CONTROL_INTERFACE_PARAM, delegate=self)
        self.last_steering_angle = None
        self.last_steering_angle_ts = None
        self.steering_accel = 0.0
        self.last_accel2 =  0.0
        self.last_torque = 0.0
        self.accel_average = SimpleTimedDiff(0.1)
        self.accel2_average = SimpleTimedDiff(0.25)
        self.speed_pub = rospy.Publisher('/wheel_speed', Float32, queue_size=1)
        self.accel_filtered_pub = rospy.Publisher('computed_accel_filtered', Float32, queue_size=1)
        self.accel_raw_pub = rospy.Publisher('computed_accel_raw', Float32, queue_size=1)

        self.steering_wheel_angle_raw_pub = rospy.Publisher('/steering/wheel_angle/raw', Float32, queue_size=1)
        self.steering_angle_raw_pub = rospy.Publisher('/steering/yaw_angle/raw', Float32, queue_size=1)
        self.steering_joint_states_pub = rospy.Publisher('/steering/joint_states', JointState, queue_size=1)
        self.accel_pedal_pub = rospy.Publisher('/accel_pedal', Float32, queue_size=1)
        self.brake_pedal_pub = rospy.Publisher('/brake_pedal', Float32, queue_size=1)

        self.throttle_cmd_sub = rospy.Subscriber('/throttle_command', Float32, self.on_throttle_cmd)
        self.brake_cmd_sub = rospy.Subscriber('/brake_command', Float32, self.on_brake_cmd)
        self.steering_sub = rospy.Subscriber('/steering_command', Float32, self.on_steering_cmd)
        self.controls_enable = rospy.Subscriber('/controls_enable', Bool, self.on_controls_enable)

        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        # disable controls
        self.oscc_enabled(False)

    def on_can_message(self, msg, can_timestamp):
        if msg.arbitration_id in self.kia_db._frame_id_to_message:
            # Kia CAN messageSTEERING_ANGLE_angle
            kia_can_msg = self.kia_db.decode_message(msg.arbitration_id, bytearray(msg.data))
            msg_type = self.kia_db.get_message_by_frame_id(msg.arbitration_id)
            if msg_type.name == "STEERING_ANGLE":
                steering_wheel_angle = float(kia_can_msg["STEERING_ANGLE_angle"]) * math.pi / 180.0
                steering_wheel_angle_msg = Float32(data=steering_wheel_angle)
                yaw_angle_msg = Float32(data=steering_wheel_angle/KIA_SOUL_STEERING_RATIO)
                self.steering_wheel_angle_raw_pub.publish(steering_wheel_angle_msg)
                self.steering_angle_raw_pub.publish(yaw_angle_msg)
                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.from_seconds(can_timestamp)
                joint_msg.name=["steering_joint", "yaw", "front_left_steer_joint", "front_right_steer_joint"]
                joint_msg.position = [
                    steering_wheel_angle,
                    steering_wheel_angle/KIA_SOUL_STEERING_RATIO,
                    steering_wheel_angle/KIA_SOUL_STEERING_RATIO,
                    steering_wheel_angle/KIA_SOUL_STEERING_RATIO
                ]
                self.steering_joint_states_pub.publish(joint_msg)

            elif msg_type.name == "SPEED":
                # print(kia_can_msg)
                speed = util.mph_to_ms(float(kia_can_msg["SPEED_rear_left"]))
                self.on_speed(speed, can_timestamp)
        elif msg.arbitration_id in self.oscc_db._frame_id_to_message:
            # OSCC Message. Currently this only publishes 0 or 1 to indicate
            # enabled or not. In the future this should be changed to
            # throttle/brake/steering values but that requires a firmware change
            # to the OSCC
            oscc_can_msg = self.oscc_db.decode_message(msg.arbitration_id, bytearray(msg.data))
            msg_type = self.oscc_db.get_message_by_frame_id(msg.arbitration_id)


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
        self.can_bus.send_message(id=throttle_oscc_msg.frame_id,
                                data=encoded_msg)

    def on_steering_cmd(self, msg):
        if not self.enabled:
            return

        steering_oscc_msg = self.oscc_db.get_message_by_name("STEERING_COMMAND")
        self.can_bus.send_message(id=0x84,
                                data=steering_oscc_msg.encode({
                                    'steering_command_magic': OSCC_MAGIC_NUMBER,
                                    'steering_command_torque_request': msg.data,
                                    'steering_command_reserved': 0
                                }))

    def on_brake_cmd(self, msg):
        if not self.enabled:
            return

        brake_oscc_msg = self.oscc_db.get_message_by_name("BRAKE_COMMAND")
        self.can_bus.send_message(id=brake_oscc_msg.frame_id,
                                    interface=CanMessage.CANTYPE_CONTROL,
                                    data=brake_oscc_msg.encode({
                                        'brake_command_magic': OSCC_MAGIC_NUMBER,
                                        'brake_command_pedal_request': msg.data,
                                        'brake_command_reserved': 0
                                    }))

    def on_controls_enable(self, msg):
        print('on_controls_enable')
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
            self.can_bus.send_message(id=msg.frame_id,
                                    data=msg.encode(data))


def main():
    rospy.init_node('kia_sour_driver', anonymous=False, log_level=rospy.DEBUG)
    # Get the parameters for the LLC node.
    KiaSoulDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
