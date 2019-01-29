#!/usr/bin/python

import rospy
from opencaret_msgs.msg import CanMessage
import can
import sys

RATE = 50.0

class Transceiver(can.Listener):
    """
    The transceiver is can bus interface agnostic which means it will try to recognize which CAN interface is
    attached to which CAN bus by figuring out which ids to expect. For example the RADAR can bus can be expected to
    be receiving 0x4ff constantly.

    This helps when using USB CAN readers since the order in which they come up cannot be strictly controlled

    """

    # These are the IDs to look for the relevant can buses
    CONTROL_IDS_MATCH = [0x73, 0x83, 0x93]

    class CanBusListener(can.Listener):
        def __init__(self, can_id, callback):
            self.can_id = can_id
            self.callback = callback
            self.callback.get_logger().info("notifier started for {}".format(can_id))

        def on_message_received(self, msg):
            # print("Incoming message on {}, id: {}".format(self.can_id, msg.arbitration_id))
            self.callback.on_message_received(msg, self.can_id)

    def __init__(self, can_interface):
        self.can_bus = None
        self.notifier = None
        self.can_type = None
        self.can_interface = can_interface
        self.reset_logical_matching()

        self.sub = rospy.Subscriber('/can_send', CanMessage , self.on_send_message)
        self.pub = rospy.Publisher('/can_recv', CanMessage, queue_size=1)

        rospy.loginfo("Listening on interfaces: {}".format(self.can_interface))
        self.can_bus = can.interface.Bus(bustype='socketcan', channel=self.can_interface, extended=False)
        # self.notifier = can.Notifier(self.can_bus, [self], timeout=0.1)
        self.can_loop()

    def can_loop(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            msg = self.can_bus.recv(0.0)
            while(msg):
                self.on_message_received(msg)
                msg = self.can_bus.recv(0.0)
            rate.sleep()

    def reset_logical_matching(self):

        self.can_logical_match = {CanMessage.CANTYPE_CONTROL: (Transceiver.CONTROL_IDS_MATCH, set())}

    def on_send_message(self, msg):
        if self.can_type == msg.interface:
            message = can.Message(arbitration_id=msg.id, data=bytearray(msg.data), extended_id=msg.is_extended)
            self.can_bus.send(message)

    def on_message_received(self, msg):
        outmsg = CanMessage()
        self.match_canbus_to_logical(msg)
        if self.can_type:
            outmsg.interface = self.can_type
        else:
            outmsg.interface = self.can_interface

        outmsg.id = msg.arbitration_id
        outmsg.can_timestamp = msg.timestamp
        outmsg.header.stamp = rospy.Time.from_seconds(msg.timestamp)
        outmsg.data = list(msg.data)
        outmsg.is_extended = msg.is_extended_id
        outmsg.is_error = msg.is_error_frame
        self.pub.publish(outmsg)


    def match_canbus_to_logical(self, msg):
        """
        Tries to match a can bus to a logical counterpart using the id list above. The algorithm is super simple
        as it assumes that ids are mostly unique across the can buses

        :param msg: Incoming can bus message
        :param can_id: The can interface on which the message came
        :return: None
        """
        if self.can_type:
            return

        for k, v in self.can_logical_match.items():

            if msg.arbitration_id in v[0]:
                rospy.logwarn("While logical matching, Found {} to {}".format(msg.arbitration_id, v[0]))

                v[0].remove(msg.arbitration_id)
                v[1].add(self.can_interface)

                self.can_logical_match[k] = v
                if len(v[0]) == 0 and len(v[1]) == 1:
                    self.can_type = k
                    self.reset_logical_matching()
                    # remove this can_id from others and reset them so they can be free!
                    rospy.loginfo('Matching can bus {} with {}'.format(self.can_interface, k))
                    return


def main():
    rospy.init_node('transceiver', anonymous=False, log_level=rospy.DEBUG)
    # Get the parameters for the LLC node.
    interface = rospy.get_param('interface')
    Transceiver(can_interface=interface)

if __name__ == '__main__':
    main()
