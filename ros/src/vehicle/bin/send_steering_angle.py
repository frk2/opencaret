import can
import struct
import sys
can_bus = can.interface.Bus(bustype='socketcan', channel="can0", extended=False)
message = can.Message(arbitration_id=132, data=struct.pack("xf", float(sys.argv[1])), extended_id=False)
print(message)
can_bus.send(message)