"""
This utility provides a compatibility layer between ROS 1 and 2.
YMMV will vary as this is not intended for 100% compatibility.
"""

use_ros_1=False
try:
    import rclpy
    from util import util
    from rclpy.node import Node
except:
    pass
try:
    import rospy
    Node=object
    use_ros_1=True
except:
    pass

node = None

if use_ros_1:
    from rospy import Subscriber, Publisher
else:
    class Subscriber(object):

        def __init__(self, topic, type, func, queue_size=0):
            global node
            self.sub = node.create_subscription(type, topic, func)

    class Publisher(object):

        def __init__(self, topic, type, queue_size=0):
            global node
            self.pub = node.create_publisher(type, topic)

        def publish(self, msg):
            self.pub.publish(msg)

def init_node(node_obj, name, log_level=None):
    global node
    node = node_obj
    if use_ros_1:
        rospy.init_node(name, log_level=log_level)

def launch_node(type):
    global node
    if use_ros_1:
        node = type()
        rospy.spin()
    else:
        rclpy.init()
        node = type()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
