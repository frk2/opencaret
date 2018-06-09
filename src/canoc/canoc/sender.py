import rclpy
from rclpy.node import Node
from opencaret_msgs.msg import CanMessage
import can

class Sender(Node, can.Listener):
    def __init__(self):
        super().__init__('canoc_senderr')
        self.sub = self.create_subscription(CanMessage, 'can_send', self.on_send_message)
        self.can_bus1 = can.interface.Bus(bustype='socketcan_native', channel='can0', extended=False)

    def on_send_message(self, msg):
        print(msg)
        message = can.Message(arbitration_id=msg.id, data=bytearray(msg.data), extended_id=msg.is_extended)
        self.can_bus1.send(message)


def main():
    rclpy.init()
    can = Sender()
    rclpy.spin(can)
    can.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()