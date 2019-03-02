import can
import cantools
import os
import oscc
import sys
OSCC_DBC_PATH = os.path.join(oscc.__path__[1],"api","include","can_protocols")
oscc_db = cantools.db.load_file(os.path.join(OSCC_DBC_PATH, 'oscc.dbc'))
can_bus = can.interface.Bus(bustype='socketcan', channel='can0', extended=False)
traj_cmd = oscc_db.get_message_by_name("VELOCITY_TRAJECTORY")
msg = can.Message(arbitration_id=0x94, data=traj_cmd.encode({'velocity_trajectory_vel_start': sys.argv[1],
                                                             'velocity_trajectory_acc_start': sys.argv[2],
                                                             'velocity_trajectory_vel_end': sys.argv[3],
                                                             'velocity_trajectory_acc_end': sys.argv[4]}), extended_id=False)
print(msg)
can_bus.send(msg)
