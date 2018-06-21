import cantools
import rclpy
from opencaret_msgs.msg import CanMessage, RadarTrack, RadarTrackAccel, RadarTracks
from rclpy.node import Node
from util import util
import os.path
import opendbc

class ECU:
    CAM = 0  # camera
    DSU = 1  # driving support unit
    APGS = 2  # advanced parking guidance system


class CAR:
    PRIUS = 0
    LEXUS_RXH = 1
    RAV4 = 2
    RAV4H = 3
    COROLLA = 4


STATIC_MSGS = [
    (0x141, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 2, '\x00\x00\x00\x46'),
    (0x128, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 3,
     '\xf4\x01\x90\x83\x00\x37'),
    (0x283, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 3,
     '\x00\x00\x00\x00\x00\x00\x8c'),
    (0x344, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 5,
     '\x00\x00\x01\x00\x00\x00\x00\x50'),
    (0x160, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 7,
     '\x00\x00\x08\x12\x01\x31\x9c\x51'),
    (0x161, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 7,
     '\x00\x1e\x00\x00\x00\x80\x07'),
    (0x365, ECU.DSU, (CAR.RAV4, CAR.COROLLA), 0, 20, '\x00\x00\x00\x80\xfc\x00\x08'),
    (0x366, ECU.DSU, (CAR.RAV4, CAR.COROLLA), 0, 20, '\x00\x72\x07\xff\x09\xfe\x00'),
    (0x4CB, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100,
     '\x0c\x00\x00\x00\x00\x00\x00\x00'),
]


class ToyotaRadarController(Node):
    """
    This radar controller is hardcoded to work only with the Toyota Corolla/Rav4/Camry 2017 Denso unit
    """

    def __init__(self):
        super().__init__('radar')
        # self.tracks_pub = self.create_publisher(String, 'radar_tracks')
        self.can_pub = self.create_publisher(CanMessage, 'can_send')
        self.adas_db = cantools.db.load_file(os.path.join(opendbc.DBC_PATH, 'toyota_prius_2017_adas.dbc'))
        self.db = cantools.db.load_file(os.path.join(opendbc.DBC_PATH, 'toyota_prius_2017_pt_generated.dbc'))

        self.can_sub = self.create_subscription(CanMessage, 'can_recv', self.on_can_message)
        self.radar_pub = self.create_publisher(RadarTracks, 'radar_tracks')
        
        # TODO: This triggers self.power_on_radar() @ 100hz, need to investigate this further!
        self.power_on_timer = self.create_timer(1.0/100, self.power_on_radar)
        self.radar_is_on = False
        self.frame = 0
        self.last_update_ms = util.ms_since_epoch()
        self.current_radar_counter = 0
        self.radar_tracks_msg = RadarTracks()

    def power_on_radar(self):
        acc_message = self.db.get_message_by_name('ACC_CONTROL')
        acc_msg = acc_message.encode(
            {"ACCEL_CMD": 0.0, "SET_ME_X63": 0x63, "SET_ME_1": 1, "RELEASE_STANDSTILL": 1, "CANCEL_REQ": 0,
             "CHECKSUM": 113})
        msg = CanMessage(id=acc_message.frame_id, interface=CanMessage.CANTYPE_RADAR, data=acc_msg)
        # print("sending control command {} on bus: {}, vl: {}".format(acc_message.frame_id, 0, acc_msg))
        self.can_pub.publish(msg)

        for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
            if self.frame % fr_step == 0:
                if addr in (0x489, 0x48a) and bus == 0:
                    # add counter for those 2 messages (last 4 bits)
                    cnt = ((self.frame / 100) % 0xf) + 1
                    if addr == 0x48a:
                        # 0x48a has a 8 preceding the counter
                        cnt += 1 << 7
                    vl += chr(cnt)
                tosend = bytearray()
                tosend.extend(map(ord, vl))
                message = CanMessage(id=addr, interface=CanMessage.CANTYPE_RADAR, data=tosend)
                self.can_pub.publish(message)
        self.frame += 1.

    def on_can_message(self, can_msg):
        if can_msg.interface == CanMessage.CANTYPE_RADAR:
            if 528 <= can_msg.id <= 559:      #FIXME: Make these ids a enum maybe
                msg = self.adas_db.decode_message(can_msg.id, bytearray(can_msg.data))
                if self.current_radar_counter != msg["COUNTER"]:
                    # new update, send this track list
                    self.radar_pub.publish(self.radar_tracks_msg)
                    self.radar_tracks_msg = RadarTracks()
                    self.current_radar_counter = msg["COUNTER"]
                    return
                if 528 <= can_msg.id <= 543:
                    track = RadarTrack(track_id=can_msg.id - 528,
                                       counter=msg["COUNTER"],
                                       lat_dist=msg["LAT_DIST"],
                                       lng_dist=msg["LONG_DIST"],
                                       rel_speed=msg["REL_SPEED"],
                                       new_track=bool(msg["NEW_TRACK"]),
                                       valid=bool(msg["VALID"]))
                    self.radar_tracks_msg.radar_tracks.append(track)
                elif 544 <= can_msg.id <= 559:
                    accel = RadarTrackAccel(track_id=can_msg.id - 544,
                                            counter=msg["COUNTER"],
                                            rel_accel=float(msg["REL_ACCEL"]))
                    self.radar_tracks_msg.radar_accels.append(accel)

                if msg["VALID"] == 1:
                    self.radar_is_on = True

def main():
    rclpy.init()
    radar = ToyotaRadarController()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
