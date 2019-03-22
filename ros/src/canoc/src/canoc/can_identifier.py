import rospy
import can

from .can_transceiver import CanTransceiver

class CanIdentifier(CanTransceiver):

    def __init__(self, **kwargs):
        self.matching_ids = kwargs['matching_ids']
        del kwargs['matching_ids']
        super(CanIdentifier, self).__init__(**kwargs)

    def wait_for_interface(self):

        if self.can_interfaces:
            rospy.loginfo("Listening on interfaces: {}".format(self.can_interfaces))
            for interface in self.can_interfaces:
                can_bus = can.interface.Bus(bustype='socketcan', channel=interface, extended=False)
                self.notifiers.append(can.Notifier(can_bus, [self], timeout=0.1))

        super(CanIdentifier, self).wait_for_interface()

    def match_canbus_to_logical(self, msg):
        """
        Tries to match a can bus to a logical counterpart using the matching id list.

        :param msg: Incoming can bus message
        :param can_id: The can interface on which the message came
        :return: None
        """

        if msg.arbitration_id in self.matching_ids:
            self.found = True
            rospy.loginfo("While logical matching, Found {} to {}".format(msg.arbitration_id, self.param_name))
            rospy.set_param(self.param_name, msg.arbitration_id)

    def on_message_received(self, msg):
        timestamp = rospy.Time.now().to_sec()
        if not self.found:
            self.match_canbus_to_logical(msg)
