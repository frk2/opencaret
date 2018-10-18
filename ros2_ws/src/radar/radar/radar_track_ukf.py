import filterpy.kalman
import filterpy.common
import numpy as np

STD_DEV_RADAR_LNG = 1.0
STD_DEV_RADAR_VEL = 0.05
PROCESS_STD_DEV = 0.1


class RadarTrackUKF:
    DT = 0.1

    def fx(self, x, dt):
        F = np.array([[1, dt],
                      [0, 1]])
        return np.dot(F, x)

    def hx(self, x):
        return x

    def __init__(self, i_dist=0.0, i_vel=0.0):
        self.dt = RadarTrackUKF.DT
        self.reset(i_dist, i_vel)

    def reset(self, i_dist=0, i_vel=0):
        # print("reset with i:{}, v:{}".format(i_dist, i_vel))
        points = filterpy.kalman.MerweScaledSigmaPoints(2, alpha=0.1, beta=2., kappa=-1.)
        self.ukf = filterpy.kalman.UnscentedKalmanFilter(dim_x=2, dim_z=2, dt=RadarTrackUKF.DT,
                                                         fx=self.fx, hx=self.hx, points=points)
        self.ukf.x = np.array([i_dist, i_vel])
        self.ukf.P *= 2.0

        self.ukf.R = np.diag([STD_DEV_RADAR_LNG ** 2, STD_DEV_RADAR_VEL ** 2])
        self.ukf.Q = filterpy.common.Q_discrete_white_noise(dim=2, dt=RadarTrackUKF.DT,
                                                            var=PROCESS_STD_DEV ** 2)
    def update(self, dist, vel, dt=None):
        # print("dist: {}, vel: {}, dt: {}".format(dist,vel, dt))
        self.ukf.predict(dt=dt)
        self.ukf.update([dist, vel])
        return tuple(self.ukf.x)


if __name__ == '__main__':
    z_std = 2.0
    zs = [[i + np.random.randn() * z_std, i + np.random.randn() * z_std] for i in range(50)]
    track = RadarTrackUKF()
    print(track.ukf.Q)
    for z in zs:
        track.update(*z)
        print("X: {}, ll: {}".format(track.ukf.x, track.ukf.log_likelihood))