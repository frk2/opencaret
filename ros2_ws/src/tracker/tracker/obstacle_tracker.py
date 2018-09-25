from sklearn.cluster import KMeans, DBSCAN
import numpy as np
from opencaret_msgs.msg import RadarTrack, RadarTrackAccel, RadarTracks, Obstacles, Obstacle
from geometry_msgs.msg import Point
from rclpy.node import Node
import rclpy


class ObstacleTracker(Node):
    VALIDITY_THRESH = 2

    def __init__(self):
        super().__init__('obstacle_tracker')
        self.radar_sub = self.create_subscription(RadarTracks, 'radar_tracks', self.on_radar_tracks)
        self.obstacle_pub = self.create_publisher(Obstacles, 'obstacles')

    def on_radar_tracks(self, tracks_msg):
        points_list = []
        usable_radar_tracks = []
        for track in tracks_msg.radar_tracks:
            if track.valid_count > ObstacleTracker.VALIDITY_THRESH:
                points_list.append([track.filt_lng_dist, track.lat_dist])
                usable_radar_tracks.append(track)
        if len(points_list) > 0:
            dbscan = DBSCAN(eps=5.0, min_samples=1).fit(points_list)
            obstacles = Obstacles()
            clusters = [[]]*len(dbscan.components_)
            for i in range(len(dbscan.labels_)):
                clusters[dbscan.labels_[i]].append(usable_radar_tracks[i])

            for cluster in clusters:
                # calculate closest radar track:
                sorted_tracks = sorted(cluster, key=lambda track:track.filt_lng_dist)
                closest_track = sorted_tracks[0]
                obstacles.obstacles.append(Obstacle(point=Point(x=closest_track.filt_lng_dist, y=closest_track.lat_dist),
                                                    relative_speed=closest_track.filt_rel_speed))


            self.get_logger().info("Raw Input: \n {}, obstacles: {} \n Clusters: {} labels: {} \n clustersize: {}\n".format(points_list,obstacles, clusters, dbscan.labels_, len(clusters)))


def main():
    rclpy.init()
    tracker = ObstacleTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()








