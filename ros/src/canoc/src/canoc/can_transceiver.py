#!/usr/bin/python

import rospy
import can
import sys

RATE = 50.0

class CanTransceiver(can.Listener):

    CONTROL_INTERFACE_PARAM = 'can-control-interface'
    RADAR_INTERFACE_PARAM = 'can-radar-interface'

    CONTROL_IDS_MATCH = [0x83]

    """
    The transceiver is can bus interface agnostic which means it will try to recognize which CAN interface is
    attached to which CAN bus by figuring out which ids to expect. For example the RADAR can bus can be expected to
    be receiving 0x4ff constantly.

    This helps when using USB CAN readers since the order in which they come up cannot be strictly controlled

    """

    class CanBusListener(can.Listener):
        def __init__(self, can_id, callback):
            self.can_id = can_id
            self.callback = callback
            self.callback.get_logger().info("notifier started for {}".format(can_id))

        def on_message_received(self, msg):
            # print("Incoming message on {}, id: {}".format(self.can_id, msg.arbitration_id))
            self.callback.on_message_received(msg, self.can_id)

    def __init__(self, param_name, can_interfaces=None, delegate=None):
        self.can_bus = None
        self.notifier = None
        self.param_name = param_name
        self.can_interfaces = can_interfaces
        self.found = False
        self.delegate = delegate
        self.notifiers = []

        #self.can_loop()
        self.wait_for_interface()

    def wait_for_interface(self):
        rate = rospy.Rate(RATE)
        interface = None
        while not interface and not rospy.is_shutdown():
            interface = rospy.get_param(self.param_name)
            self.found = interface != None
            rate.sleep()
        if interface and not self.can_interfaces:
            rospy.loginfo("Listening on interfaces: {}".format(interface))
            self.can_bus = can.interface.Bus(bustype='socketcan', channel=interface, extended=False)
            self.notifiers.append(can.Notifier(self.can_bus, [self], timeout=0.1))

    def can_loop(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            msg = self.can_bus.recv(0.0)
            while(msg):
                self.on_message_received(msg)
                msg = self.can_bus.recv(0.0)
            rate.sleep()

    def send_message(self, id, data, is_extended=False):
        message = can.Message(arbitration_id=id, data=bytearray(data), extended_id=is_extended)
        self.can_bus.send(message)

    def on_message_received(self, msg):
        timestamp = rospy.Time.now().to_sec()
        if self.delegate != None and self.found:
            self.delegate.on_can_message(msg, timestamp)
