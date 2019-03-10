#!/usr/bin/python

import carla
import random
import weakref
from carla import ColorConverter as cc
import numpy as np
from sensor_msgs.msg import Image
import rospy
from carla_ros_bridge.parent import Parent
from ackermann_msgs.msg import AckermannDrive

class CarlaWorld:
    def __init__(self, world):
        self.world = world
        self.map = self.world.get_map()
        self.player = None
        self.camera = None
        self.ego_vehicle = None
        self.camera_publisher = rospy.Publisher('/front_camera', Image, queue_size=1)


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
        msg = AckermannDrive()
        msg.acceleration=1.0
        msg.speed = 5.0

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
        print(self.camera)
        weak_self = weakref.ref(self)
        self.camera.listen(
            lambda image: CarlaWorld.parse_image(weak_self, image))

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


def main(host, port):
    rospy.init_node('carla_driver', anonymous=False, log_level=rospy.DEBUG)
    client = carla.Client(host, port)
    client.set_timeout(2.0)
    world = CarlaWorld(client.get_world())
    world.start()

    rospy.spin()

if __name__ == '__main__':
    main('127.0.0.1', 2000)
