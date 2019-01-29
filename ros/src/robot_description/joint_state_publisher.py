#!/usr/bin/python

# @Author: Jose Rojas <jrojas>
# @Date:   2018-08-19T17:38:02-07:00
# @Email:  jrojas@redlinesolutions.co
# @Last modified by:   jrojas
# @Last modified time: 2018-07-22T01:45:00-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import math
from util import rospy_compat
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64

class JointStatePublisher(rospy_compat.Node):

    PUBLISHING_RATE = 10  # hz

    def __init__(self):
        if not(rospy_compat.use_ros_1):
            super().__init__('joint_state_publisher')
        rospy_compat.init_node(self, 'joint_state_publisher')
        self.steering_joint_sub = rospy_compat.Subscriber('steering/joint_states', JointState, self.on_steering_joints)
        self.joint_state_pub = rospy_compat.Publisher('joint_states', JointState, queue_size=1)
        self.joint_angles = {
            "steering_joint": float(0.0),
            "front_left_steer_joint": float(0.0),
            "front_right_steer_joint": float(0.0),
            "front_left_wheel_joint": float(0.0),
            "front_right_wheel_joint": float(0.0),
            "rear_left_wheel_joint": float(0.0),
            "rear_right_wheel_joint": float(0.0)
        }

    def on_steering_joints(self, msg):
        # XXX Ackerman angle not handled here
        for name, position in zip(msg.name, msg.position):
            self.joint_angles[name] = float(position)
            
    def on_publish(self):
        m = JointState()
        if rospy_compat.use_ros_1:
            m.header.stamp = rospy_compat.rospy.Time.now()

        names = []
        positions = []
        velocities = []
        efforts = []
        for k, v in self.joint_angles.items():
            names.append(k)
            positions.append(v)
            velocities.append(float(0.0))
            efforts.append(float(0))

        m.name = names
        m.position = positions
        m.velocity = velocities
        m.effort = efforts

        self.joint_state_pub.publish(m)

    def on_run(self):
        self.on_publish()

def main():
    rospy_compat.launch_node(JointStatePublisher, sleep=1.0/JointStatePublisher.PUBLISHING_RATE)

if __name__ == '__main__':
    main()
