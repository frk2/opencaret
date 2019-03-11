#!/usr/bin/python

import carla
import random
import weakref
from carla import ColorConverter as cc
import numpy as np
from sensor_msgs.msg import Image
import rospy
from carla_ros_bridge.parent import Parent
from carla_ros_bridge.msg import EgoVehicleControlInfo
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from opencaret_msgs.msg import LongitudinalTarget
from sensor_msgs.msg import JointState

import time
import math


PLAN_LOOKAHEAD_INDEX = 3
TIME_STEP = 0.2
ACC_RATE_FILTER = 0.9


class CarlaWorld:
    def __init__(self, world):
        self.world = world
        self.map = self.world.get_map()
        self.player = None
        self.camera = None
        self.ego_vehicle = None
        self.steering = 0.0
        self.seg = None

        self.camera_publisher = rospy.Publisher('/front_camera', Image, queue_size=1)
        self.seg_pub = rospy.Publisher('/front_camera_seg', Image, queue_size=1)

        self.ego_cmd = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)
        self.speed_pub = rospy.Publisher('/wheel_speed', Float32, queue_size=1)
        self.accel_filtered_pub = rospy.Publisher('computed_accel_filtered', Float32, queue_size=1)
        self.accel_raw_pub = rospy.Publisher('computed_accel_raw', Float32, queue_size=1)
        self.steering_wheel_angle_raw_pub = rospy.Publisher('/steering/wheel_angle/raw', Float32, queue_size=1)
        self.steering_angle_raw_pub = rospy.Publisher('/steering/yaw_angle/raw', Float32, queue_size=1)
        self.steering_joint_states_pub = rospy.Publisher('/steering/joint_states', JointState, queue_size=1)

        self.steering_sub = rospy.Subscriber('/steering_angle_target', Float32, self.on_steering_cmd)
        self.last_plan_time = None
        self.controls_enabled = False
        self.plan = None
        self.acceleration_plan = None
        self.velocity_plan = None
        self.ego_velocity = 0
        self.long_target = LongitudinalTarget()
        self.last_accel = None
        rospy.on_shutdown(self.stop)
        rospy.Subscriber('longitudinal_target', LongitudinalTarget, self.on_long_target)
        rospy.Subscriber('/carla/ego_vehicle/ego_vehicle_control_info', EgoVehicleControlInfo, self.on_ego_vehicle_info)

    def start(self):

        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.bmw.*'))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        spawn_points = self.map.get_spawn_points()
        spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        if self.player is not None:
            self.player.destroy()
        self.player = self.world.try_spawn_actor(blueprint, spawn_point)


        # self.ego_vehicle.ackermann_command_updated(msg)
        # Set up the sensors.

        camera_sensor_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_sensor_bp.set_attribute('image_size_x', str(1280))
        camera_sensor_bp.set_attribute('image_size_y', str(720))

        if self.camera is not None:
            self.camera.destroy()
        self.camera = self.world.spawn_actor(
            camera_sensor_bp,
            carla.Transform(carla.Location(x=0.0, z=1.7)),
            attach_to=self.player)
        weak_self = weakref.ref(self)
        self.camera.listen(
            lambda image: CarlaWorld.parse_image(weak_self, image))

        camera_sensor_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        camera_sensor_bp.set_attribute('image_size_x', str(1280))
        camera_sensor_bp.set_attribute('image_size_y', str(720))

        if self.seg is not None:
            self.seg.destroy()
        self.seg = self.world.spawn_actor(
            camera_sensor_bp,
            carla.Transform(carla.Location(x=0.0, z=1.7)),
            attach_to=self.player)
        weak_self = weakref.ref(self)
        self.seg.listen(
            lambda image: self.parse_seg(image))

    def stop(self):
        self.camera.destroy()
        self.player.destroy()

    def parse_seg(self, seg):
        # seg.convert(cc.CityScapesPalette)
        array = np.frombuffer(seg.raw_data, dtype=np.dtype("uint8")).copy()
        not_road = array != 7
        array[not_road] = 0
        array[~not_road] = 255
        img_to_publish = Image()
        img_to_publish.data = array.tolist()
        img_to_publish.encoding = 'bgra8'
        img_to_publish.width = seg.width
        img_to_publish.height = seg.height
        self.seg_pub.publish(img_to_publish)


    @staticmethod
    def parse_image(weak_self, image):
        self = weak_self()
        img_to_publish = Image()
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        img_to_publish.data = array.tolist()
        img_to_publish.encoding = 'bgra8'
        img_to_publish.width = image.width
        img_to_publish.height = image.height
        self.camera_publisher.publish(img_to_publish)
        # image.convert(cc.Raw)
        # array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        # array = np.reshape(array, (image.height, image.width, 3))

    def on_steering_cmd(self, msg):
        self.steering = msg.data
        self.send_msg()

    def send_msg(self):
        msg = AckermannDrive()
        msg.acceleration = self.long_target.accel
        msg.speed = self.long_target.speed
        msg.steering_angle = self.steering
        print(msg)
        self.ego_cmd.publish(msg)

    def on_long_target(self, msg):
        print(msg)
        self.long_target = msg
        self.send_msg()

    def on_ego_vehicle_info(self, msg):

        # speed accel
        self.speed_pub.publish(Float32(data=msg.current.speed))
        if self.last_accel is None:
            self.last_accel = msg.current.accel

        accel = self.last_accel * ACC_RATE_FILTER + msg.current.accel * (1 - ACC_RATE_FILTER)
        self.accel_raw_pub.publish(Float32(data=msg.current.accel))
        self.accel_filtered_pub.publish(Float32(data=accel))
        self.last_accel = accel
        # Steering angle
        steering_wheel_angle = msg.target.steering_angle
        steering_wheel_angle_msg = Float32(data=steering_wheel_angle)
        self.steering_wheel_angle_raw_pub.publish(steering_wheel_angle_msg)

def main(host, port):
    rospy.init_node('carla_driver', anonymous=False, log_level=rospy.DEBUG)
    client = carla.Client(host, port)
    client.set_timeout(2.0)
    world = CarlaWorld(client.get_world())
    world.start()

    rospy.spin()

if __name__ == '__main__':
    main('127.0.0.1', 2000)
