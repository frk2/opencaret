import cantools
import rclpy
import time
from opencaret_msgs.msg import CanMessage, RadarTrack, RadarTrackAccel, RadarTracks
from rclpy.node import Node
from util import util
from radar import RADAR_VALID_MAX
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
    RADAR_TRACK_ID_START = 528
    RADAR_TRACK_ID_RANGE = 16
    RADAR_TRACK_ID_END = RADAR_TRACK_ID_START + RADAR_TRACK_ID_RANGE - 1  # 543
    RADAR_TRACK_ACCEL_ID_START = RADAR_TRACK_ID_END + 1  # 544
    RADAR_TRACK_ACCEL_ID_END = RADAR_TRACK_ACCEL_ID_START + RADAR_TRACK_ID_RANGE - 1  # 559

    """
    This radar controller is hardcoded to work only with the Toyota Corolla/Rav4/Camry 2017 Denso unit
    """

    def __init__(self):
        super().__init__('radar')
        # self.tracks_pub = self.create_publisher(String, 'radar_tracks')
        self.can_pub = self.create_publisher(CanMessage, 'can_send')
        self.adas_db = cantools.db.load_file(os.path.join(opendbc.DBC_PATH, 'toyota_prius_2017_adas.dbc'))
        self.db = cantools.db.load_file(os.path.join(opendbc.DBC_PATH, 'toyota_prius_2017_pt_generated.dbc'),
                                        strict=False)

        self.can_sub = self.create_subscription(CanMessage, 'can_recv', self.on_can_message)
        self.radar_pub = self.create_publisher(RadarTracks, 'radar_tracks')

        # This triggers self.power_on_radar() @ 100hz.
        # Based on OpenPilot, this is the base update rate required for delivering the CAN messages defined in STATIC_MSGS
        self.power_on_timer = self.create_timer(1.0 / 100.0, self.power_on_radar)
        self.radar_is_on = False
        self.frame = 0
        self.last_update_ms = util.ms_since_epoch()
        self.current_radar_counter = 0
        self.current_track_ids = {}
        self.current_radar_accels = []
        self.cache_radar_tracks = {}

        # only need to create these once, they are not recreated in the message loop
        for i in range(0, self.RADAR_TRACK_ID_RANGE):
            track = RadarTrack()
            track.track_id = i
            track.valid_count = 0
            track.valid = False
            self.cache_radar_tracks[i] = track

        self.reset_tracks()

    def reset_tracks(self):
        self.current_track_ids = {}
        self.current_radar_accels = []

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

                    current_radar_tracks = []
                    current_radar_accels = self.current_radar_accels

                    # decrease the valid count for all of the tracks that were missing
                    for k, track in self.cache_radar_tracks.items():
                        if k not in self.current_track_ids:
                            valid_count = track.valid_count = max(0, track.valid_count - 1)
                            if valid_count > 0:
                                # add track to list from cache, with a invalid flag, but no accel for simplicity's sake
                                track.counter = self.current_radar_counter
                                track.valid = False

                        if k in self.current_track_ids or track.valid_count > 0:
                            current_radar_tracks.append(track)

                    # new update, send this track list
                    if len(current_radar_tracks) > 0 or len(current_radar_accels) > 0:
                        radar_tracks_msg = RadarTracks()
                        radar_tracks_msg.radar_tracks = current_radar_tracks
                        radar_tracks_msg.radar_accels = current_radar_accels
                        self.radar_pub.publish(radar_tracks_msg)

                    self.reset_tracks()
                    self.current_radar_counter = msg["COUNTER"]

                if self.RADAR_TRACK_ID_START <= can_msg.id <= self.RADAR_TRACK_ID_END:

                    track_id = can_msg.id - self.RADAR_TRACK_ID_START
                    track = self.cache_radar_tracks[track_id]

                    if msg['LONG_DIST'] >=255 or msg['NEW_TRACK']:
                        track.valid_count = 0 # reset counter

                    curr_valid_count = track.valid_count
                    curr_valid_count += (1 if msg["VALID"] and msg['LONG_DIST'] < 255 else -1)
                    curr_valid_count = min(RADAR_VALID_MAX, max(0, curr_valid_count))

                    assert not(msg["VALID"] and msg['LONG_DIST']) or curr_valid_count > 0, print(msg, curr_valid_count)

                    track.counter = msg["COUNTER"]
                    track.lat_dist = msg["LAT_DIST"]
                    track.lng_dist = msg["LONG_DIST"]
                    track.rel_speed = msg["REL_SPEED"]
                    track.new_track = bool(msg["NEW_TRACK"])
                    track.valid_count = curr_valid_count
                    track.valid = bool(msg["VALID"])

                    self.current_track_ids[track_id] = True

                    if msg["VALID"] == 1:
                        self.radar_is_on = True

                elif self.RADAR_TRACK_ACCEL_ID_START <= can_msg.id <= self.RADAR_TRACK_ACCEL_ID_END:
                    track_id = can_msg.id - self.RADAR_TRACK_ACCEL_ID_START
                    accel = RadarTrackAccel(track_id=track_id,
                                            counter=msg["COUNTER"],
                                            rel_accel=float(msg["REL_ACCEL"]))
                    self.current_radar_accels.append(accel)

def main():
    rclpy.init()
    radar = ToyotaRadarController()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
