import rclpy
from rclpy.node import Node
from opencaret_msgs.msg import CanMessage
import can

class Receiver(Node, can.Listener):
    def __init__(self):
        super().__init__('canoc_receiver')
        self.pub = self.create_publisher(CanMessage, 'can_recv')
        self.can_bus1 = can.interface.Bus(bustype='socketcan_native', channel='can0', extended=False)
        self.notifier = can.Notifier(self.can_bus1, [self], timeout=0.1)

    def on_message_received(self, msg):
        outmsg = CanMessage()
        print(msg)
        outmsg.id = msg.arbitration_id
        outmsg.data = list(msg.data)
        outmsg.is_extended = msg.is_extended_id
        outmsg.is_error = msg.is_error_frame
        self.pub.publish(outmsg)


def main():
    rclpy.init()
    can = Receiver()
    rclpy.spin(can)
    can.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()