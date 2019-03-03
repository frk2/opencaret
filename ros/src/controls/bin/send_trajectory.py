import can
import cantools
import os
import oscc
import sys
import struct
OSCC_DBC_PATH = os.path.join(oscc.__path__[1],"api","include","can_protocols")
print(OSCC_DBC_PATH)
oscc_db = cantools.db.load_file(os.path.join(OSCC_DBC_PATH, 'oscc.dbc'))
can_bus = can.interface.Bus(bustype='socketcan', channel='can0', extended=False)
traj_cmd = oscc_db.get_message_by_name("VELOCITY_TRAJECTORY")
print(traj_cmd.encode({'velocity_trajectory_vel_start': float(sys.argv[1]),
                                                             'velocity_trajectory_acc_start': float(sys.argv[2])}))
msg = can.Message(arbitration_id=0x94, data=struct.pack('ff', float(sys.argv[1]), float(sys.argv[2])))
print(msg)
can_bus.send(msg)
