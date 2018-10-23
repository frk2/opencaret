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

def launch_node(type, *args, **kargs):
    global node

    sleep = 0.1 if 'sleep' not in kargs else kargs['sleep']

    if use_ros_1:
        node = type(*args)
        if hasattr(type, 'on_run') and callable(getattr(type, 'on_run')):
            rate = rospy.Rate(1.0/sleep)
            while not rospy.is_shutdown():
                node.on_run()
                rate.sleep()
        else:
            rospy.spin()
    else:
        rclpy.init()
        node = type(*args)
        if hasattr(type, 'on_run') and callable(getattr(type, 'on_run')):
            executor = rclpy.get_global_executor()
            try:
                executor.add_node(node)
                while rclpy.ok():
                    executor.spin_once(timeout_sec=sleep)
                    node.on_run()
            finally:
                executor.remove_node(node)
        else:
            rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
