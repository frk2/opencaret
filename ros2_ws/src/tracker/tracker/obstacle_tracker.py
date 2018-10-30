from sklearn.cluster import KMeans, DBSCAN
import numpy as np
from opencaret_msgs.msg import RadarTrack, RadarTrackAccel, RadarTracks, Obstacles, Obstacle
from geometry_msgs.msg import Point
from rclpy.node import Node
import rclpy
from radar.radar_track_ukf import RadarTrackUKF
from time import time

CAR_WIDTH = 4.0
DEFAULT_OBSTACLE_DISTANCE = 200.0

class ObstacleTracker(Node):
    VALIDITY_THRESH = 1

    def __init__(self):
        super().__init__('obstacle_tracker')
        self.radar_sub = self.create_subscription(RadarTracks, 'radar_tracks', self.on_radar_tracks)
        self.obstacle_pub = self.create_publisher(Obstacles, 'obstacles')
        self.lead_obstacle = self.create_publisher(Obstacle, 'lead_obstacle')
        self.all_obstacles = []
        self.last_lead_vehicle = None
        self.lead_ukf = RadarTrackUKF()
        self.last_ukf_update = time()

    def on_radar_tracks(self, tracks_msg):
        points_list = []
        usable_radar_tracks = []
        for track in tracks_msg.radar_tracks:
            if track.valid_count >= ObstacleTracker.VALIDITY_THRESH and track.lng_dist > 0.0:
                points_list.append([track.lng_dist, track.lat_dist])
                usable_radar_tracks.append(track)

        self.all_obstacles = []
        if len(points_list) > 0:
            dbscan = DBSCAN(eps=3.0, min_samples=1).fit(points_list)
            clusters = [list() for i in range(len(dbscan.labels_))]
            self.get_logger().debug("Raw: {}".format(points_list))
            for i in range(len(dbscan.labels_)):
                clusters[dbscan.labels_[i]].append(usable_radar_tracks[i])

            for cluster in clusters:
                # calculate closest radar track:
                if len(cluster) > 0:
                    sorted_tracks = sorted(cluster, key=lambda track: track.lng_dist)
                    closest_track = sorted_tracks[0]
                    self.all_obstacles.append(Obstacle(point=Point(x=closest_track.lng_dist, y=closest_track.lat_dist),
                                                       relative_speed=closest_track.rel_speed))

            self.get_logger().debug("Raw Input: \n {}, obstacles: {} \n Clusters: {} "
                                   "labels: {} \n clustersize: {}\n".format(points_list, self.all_obstacles, clusters,
                                                                            dbscan.labels_, len(clusters)))

        self.calculate_and_publish_lead()

    def calculate_and_publish_lead(self):
        # filter out all tracks not in the direct path of the vehicle
        closest_obs_in_range = None
        half_width = CAR_WIDTH / 2.0
        for obstacle in self.all_obstacles:
            if -half_width < obstacle.point.y < half_width:
                if closest_obs_in_range:
                    if obstacle.point.x < closest_obs_in_range.point.x:
                        closest_obs_in_range = obstacle
                else:
                    closest_obs_in_range = obstacle

        if closest_obs_in_range:
            current_ts = time()
            if not self.last_lead_vehicle:
                self.lead_ukf.reset(i_dist=closest_obs_in_range.point.x, i_vel=closest_obs_in_range.relative_speed)
                dist = closest_obs_in_range.point.x
                vel = closest_obs_in_range.relative_speed
            else:
                dist, vel = self.lead_ukf.update(dist=closest_obs_in_range.point.x,
                                                 vel=closest_obs_in_range.relative_speed,
                                                 dt=current_ts - self.last_ukf_update)


            self.last_ukf_update = current_ts
            closest_obs_in_range.point.x = dist
            closest_obs_in_range.relative_speed = vel
            self.lead_obstacle.publish(closest_obs_in_range)
        else:
            self.lead_obstacle.publish(Obstacle(point=Point(x=DEFAULT_OBSTACLE_DISTANCE, y=0.),
                                                relative_speed=0.0))

        self.last_lead_vehicle = closest_obs_in_range


def main():
    rclpy.init()
    tracker = ObstacleTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()








