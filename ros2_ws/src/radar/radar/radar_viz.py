import rclpy
from opencaret_msgs.msg import RadarTracks
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.node import Node
from util import util


class RadarViz(Node):
    def __init__(self):
        super().__init__('radar_viz')
        self.radar_sub = self.create_subscription(RadarTracks, '/radar_tracks', self.on_radar_tracks)
        self.radar_rviz_pub = self.create_publisher(Marker, '/radar_viz')

    def on_radar_tracks(self, msg):
        marker = Marker()
        # marker.header.stamp = util.usec_since_epoch()
        marker.header.frame_id = "middle_radar_link"
        marker.ns = "radar_tracks"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.MODIFY
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.points = []
        for track in msg.radar_tracks:
            p = Point()
            p.x = float(track.lng_dist)
            p.y = -float(track.lat_dist)
            p.z = 0.0
            marker.points.append(p)

        self.radar_rviz_pub.publish(marker)



def main():
    rclpy.init()
    radar = RadarViz()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()