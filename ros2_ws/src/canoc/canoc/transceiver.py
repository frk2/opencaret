import rclpy
from rclpy.node import Node
from opencaret_msgs.msg import CanMessage
import can
from bidict import bidict
import netifaces
import sys

class Transceiver(Node, can.Listener):
    """
    The transceiver is can bus interface agnostic which means it will try to recognize which CAN interface is
    attached to which CAN bus by figuring out which ids to expect. For example the RADAR can bus can be expected to
    be receiving 0x4ff constantly.

    This helps when using USB CAN readers since the order in which they come up cannot be strictly controlled

    """

    # These are the IDs to look for the relevant can buses
    RADAR_IDS_MATCH = [0x220, 0x221, 0x222, 0x223, 0x224, 0x225]
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
        super().__init__('canoc_transceiver_'+can_interface)
        self.can_bus = None
        self.notifier = None
        self.can_type = None
        self.can_interface = can_interface
        self.reset_logical_matching()

        self.sub = self.create_subscription(CanMessage, 'can_send', self.on_send_message)
        self.pub = self.create_publisher(CanMessage, 'can_recv')

        self.get_logger().info("Listening on interfaces: {}".format(self.can_interface))
        self.can_bus = can.interface.Bus(bustype='socketcan_native', channel=self.can_interface, extended=False)
        self.notifier = can.Notifier(self.can_bus, [self], timeout=0.1)


    #     self.create_timer(1.0 / 50.0, self.can_loop(bus, canbus))
    #
    # def can_loop(self, bus_id, can_bus):
    #     def poll_bus():
    #         msg = can_bus.recv(0.0)
    #         while(msg):
    #             self.on_message_received(msg, bus_id)
    #             msg = can_bus.recv(0.0)
    #     return poll_bus

    def reset_logical_matching(self):

        self.can_logical_match = {CanMessage.CANTYPE_RADAR: (Transceiver.RADAR_IDS_MATCH, set()),
                                  CanMessage.CANTYPE_CONTROL: (Transceiver.CONTROL_IDS_MATCH, set())}

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
                self.get_logger().warn("While logical matching, Found {} to {}".format(msg.arbitration_id, v[0]))

                v[0].remove(msg.arbitration_id)
                v[1].add(self.can_interface)

                self.can_logical_match[k] = v
                if len(v[0]) == 0 and len(v[1]) == 1:
                    self.can_type = k
                    self.reset_logical_matching()
                    # remove this can_id from others and reset them so they can be free!
                    self.get_logger().info('Matching can bus {} with {}'.format(self.can_interface, k))
                    return


def main():
    rclpy.init()
    can = Transceiver(sys.argv[1])
    rclpy.spin(can)
    can.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()