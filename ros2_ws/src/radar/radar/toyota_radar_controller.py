import cantools
import rclpy
import time
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


# XXX: The Corolla is the targeted car for now
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

    RADAR_VALID_MAX = 100
    RADAR_TRACK_ID_START = 528
    RADAR_TRACK_ID_RANGE = 16
    RADAR_TRACK_ID_END = RADAR_TRACK_ID_START + RADAR_TRACK_ID_RANGE - 1 # 543
    RADAR_TRACK_ACCEL_ID_START = RADAR_TRACK_ID_END + 1 # 544
    RADAR_TRACK_ACCEL_ID_END = RADAR_TRACK_ACCEL_ID_START + RADAR_TRACK_ID_RANGE - 1 # 559

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

        # This triggers self.power_on_radar() @ 100hz.
        # Based on OpenPilot, this is the base update rate required for delivering the CAN messages defined in STATIC_MSGS
        self.power_on_timer = self.create_timer(1.0/100.0, self.power_on_radar)
        self.radar_is_on = False
        self.frame = 0
        self.last_update_ms = util.ms_since_epoch()
        self.current_radar_counter = 0
        self.radar_tracks_msg = RadarTracks()

    def power_on_radar(self):
        for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
            if self.frame % fr_step == 0:
                tosend = bytearray()
                tosend.extend(map(ord, vl))
                # XXX This might be incorrect... interface might need to use the 'bus' variable
                message = CanMessage(id=addr, interface=CanMessage.CANTYPE_RADAR, data=tosend)
                self.can_pub.publish(message)
        self.frame += 1.

    def on_can_message(self, can_msg):
        if can_msg.interface == CanMessage.CANTYPE_RADAR:
            if self.RADAR_TRACK_ID_START <= can_msg.id <= self.RADAR_TRACK_ACCEL_ID_END:
                msg = self.adas_db.decode_message(can_msg.id, bytearray(can_msg.data))
                if self.current_radar_counter != msg["COUNTER"]:
                    # filter only valid tracks - disabled for now since it might be useful to monitor invalid tracks
                    # tracks = list(filter(lambda x : x[0].valid > 0, zip(self.radar_tracks_msg.radar_tracks, self.radar_tracks_msg.radar_accels)))
                    # self.radar_tracks_msg.radar_tracks = list(map(lambda x : x[0], tracks))
                    # self.radar_tracks_msg.radar_accels = list(map(lambda x : x[1], tracks))

                    # new update, send this track list
                    if len(self.radar_tracks_msg.radar_tracks) > 0:
                        self.radar_pub.publish(self.radar_tracks_msg)
                    self.radar_tracks_msg = RadarTracks()
                    track = RadarTrack()
                    track.valid = False
                    track.valid_count = -1
                    self.radar_tracks_msg.radar_tracks = [track] * self.RADAR_TRACK_ID_RANGE
                    self.radar_tracks_msg.radar_accels = [RadarTrackAccel()] * self.RADAR_TRACK_ID_RANGE
                    self.current_radar_counter = msg["COUNTER"]
                    return

                if self.RADAR_TRACK_ID_START <= can_msg.id <= self.RADAR_TRACK_ID_END:
                    track = RadarTrack(track_id=can_msg.id - self.RADAR_TRACK_ID_START,
                                       counter=msg["COUNTER"],
                                       lat_dist=msg["LAT_DIST"],
                                       lng_dist=msg["LONG_DIST"],
                                       rel_speed=msg["REL_SPEED"],
                                       new_track=bool(msg["NEW_TRACK"]),
                                       valid_count=0,
                                       valid=msg["VALID"] != 0)

                    curr_track_valid = self.radar_tracks_msg.radar_tracks[track.track_id].valid_count
                    if msg['LONG_DIST'] >=255 or msg['NEW_TRACK']:
                        curr_track_valid = 0 # reset counter

                    track.valid_count = min(self.RADAR_VALID_MAX,
                                    max(0, curr_track_valid + (1 if msg["VALID"] and msg['LONG_DIST'] < 255 else -1))
                                    )
                    self.radar_tracks_msg.radar_tracks[track.track_id] = track

                    if msg["VALID"] == 1:
                        self.radar_is_on = True

                elif self.RADAR_TRACK_ACCEL_ID_START <= can_msg.id <= self.RADAR_TRACK_ACCEL_ID_END:
                    accel = RadarTrackAccel(track_id=can_msg.id - self.RADAR_TRACK_ACCEL_ID_START,
                                            counter=msg["COUNTER"],
                                            rel_accel=float(msg["REL_ACCEL"]))
                    self.radar_tracks_msg.radar_accels[accel.track_id] = accel

def main():
    rclpy.init()
    radar = ToyotaRadarController()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
