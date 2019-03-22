#!/usr/bin/python
import numpy as np
import perception.predict as predict
import matplotlib.pyplot as plt
import cv2
import pickle
import os
import rospy
from opencaret_msgs.msg import RoadSurface
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from PIL import ImageDraw, ImageFont
from cv_bridge import CvBridge, CvBridgeError
import PIL
import time

file_location = os.path.dirname(os.path.abspath(__file__))
data_location = file_location + '/../data/'
DEBUG_MODE = False
YM_PER_PIX = 0.01

class RoadDetector:
    def __init__(self, sim_mode=False):

        # self.camera = cv2.VideoCapture(0)
        self.camera = cv2.VideoCapture("/home/faraz/data/2019-03-20-192432.webm")
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 576)
        self.rate = rospy.Rate(10)
        self.transform = pickle.load(open(data_location + 'road_transform_1536.p', 'rb'))
        (self.mtx, self.dist) = pickle.load(open(data_location + 'cam_calib.p', 'rb'))
        self.predictor = predict.Prediction()
        self.road_surface_pub = rospy.Publisher('/road_surface', RoadSurface, queue_size=1)
        self.cte_pub = rospy.Publisher('/cte', Float32, queue_size=1)
        self.curvature_pub = rospy.Publisher('/curvature', Float32, queue_size=1)

        self.seg_pub = rospy.Publisher('/front_camera_seg', Image, queue_size=1)

        self.last_centroids = None
        self.last_fit = None

        if sim_mode:
            rospy.logwarn("Perception running in sim mode")
            self.bridge = CvBridge()
            rospy.Subscriber('/front_camera', Image, self.on_sim_image)
            rospy.spin()
        else:
            self.loop()

    def on_sim_image(self, image):
        self.process_frame(self.bridge.imgmsg_to_cv2(image, "bgr8"))

    def loop(self):
        while not rospy.is_shutdown():
            ret, frame = self.camera.read()
            if ret:
                frame = cv2.undistort(frame, self.mtx, self.dist)
                self.process_frame(frame)
            self.rate.sleep()


    def process_frame(self, frame):
        start = time.time()
        prediction, overlay = self.predictor.infer(frame.astype(np.float32), overlay=True)
        endpred = time.time()
        mask = prediction == 0
        prediction[mask] = 255.0
        prediction[~mask] = 0.0
        perspective = cv2.warpPerspective(prediction, self.transform, (2500, 2000))
        endwarp = time.time()
        average = None
        if self.last_centroids is not None:
            average = self.last_centroids[0][1]

        new_centroids = self.calculate_centroids(perspective, average=average)
        if new_centroids is not None:
            self.last_centroids = new_centroids

        # Fit poly to different axis as its more stable, then flip axes

        current_fit = np.poly1d(np.polyfit(self.last_centroids[:, 0],
                                             self.last_centroids[:, 1], 2))

        if self.last_fit is None:
            self.last_fit = current_fit
        else:
            self.last_fit = 0.9 * self.last_fit + current_fit * 0.1

        endfit = time.time()
        x = np.linspace(0, 2000)

        self.last_curvatures = 1. / (((1 + (2 * self.last_fit[2] * x * YM_PER_PIX +
                                            self.last_fit[1]) ** 2) ** 1.5) / np.absolute(2 * self.last_fit[2]))

        # self.last_cte = frame.shape[1] / 2.0 - self.last_fit(curvature_eval_point)
        y = self.last_fit(x)

        # x is y, y is x
        ym = np.flip(x * YM_PER_PIX)
        self.last_ctes = perspective.shape[1] / 2.0 - y
        road_message = RoadSurface()
        road_message.distance = ym
        road_message.ctes = self.last_ctes
        road_message.curvatures = self.last_curvatures
        road_message.cte = self.last_ctes[-50]
        road_message.curvature = self.last_curvatures[-50]
        self.road_surface_pub.publish(road_message)
        self.cte_pub.publish(Float32(data=self.last_ctes[-50]))
        self.curvature_pub.publish(Float32(data=self.last_curvatures[-50]))

        linefit_warp = np.zeros((x.shape[0], 2))
        linefit_warp[:, 1] = x
        linefit_warp[:, 0] = y
        centroids_swap = self.last_centroids.copy()
        centroids_swap[:, [0, 1]] = centroids_swap[:, [1, 0]]

        centwarp = cv2.perspectiveTransform(np.array([centroids_swap]),
                                            cv2.invert(self.transform)[1])[0]
        linefit_warp = cv2.perspectiveTransform(np.array([linefit_warp]),
                                                cv2.invert(self.transform)[1])[0]

        draw = ImageDraw.Draw(overlay)
        # centwarp[:, [0, 1]] = centwarp[:, [1, 0]]
        point_list_centroids = []
        point_list_linefit = []
        for i in range(centwarp.shape[0]):
            point_list_centroids.append((centwarp[i][0], centwarp[i][1]))
        for i in range(linefit_warp.shape[0]):
            point_list_linefit.append((linefit_warp[i][0], linefit_warp[i][1]))
        # draw.line(point_list_centroids, fill='blue', width=5)
        draw.line(point_list_linefit, fill='green', width=5)
        # overlay.show()
        img_to_publish = Image()
        ovr = np.array(overlay).reshape(-1)
        img_to_publish.data = ovr.tolist()
        img_to_publish.encoding = 'rgb8'
        img_to_publish.width = overlay.width
        img_to_publish.height = overlay.height
        self.seg_pub.publish(img_to_publish)
        final = time.time()
        rospy.logwarn("Pred:{} warp: {}, fit: {}, final: {}".format(endpred-start, endwarp-endpred, endfit-endwarp, final-endfit))
        if DEBUG_MODE:
            # ym_per_pix = 30.0 / 576
            # max_y = 576

            #     centwarp[:, 1] = 576 - centwarp[:,1]
            plt.imshow(perspective, cmap='gray')
            plt.plot(self.last_centroids[:, 1], self.last_centroids[:, 0], '.', self.last_fit(x), x, '-')
            #     plt.xlim(0, 2500)
            print("last CTE: {} - k: {}".format(self.last_ctes, self.last_curvatures))
            plt.show()
            plt.imshow(overlay)
            plt.plot(centwarp[:, 0], centwarp[:, 1], '.', linefit_warp[:, 0], linefit_warp[:, 1], '-')
            plt.show()
    def calculate_centroids(self, frame, average=None, step_size=20):
        centroids = []
        for y in range(frame.shape[0], 0, -step_size):
            #         print(frame[y:y+10,:])
            allpoints = np.nonzero(frame[y - step_size:y, :])
            #         print(len(allpoints[1]))
            if len(allpoints[1] > 0):
                centroid = self.centeroidnp(allpoints[1], allpoints[0])
                if average is None:
                    average = centroid[0]
                else:
                    average = average * 0.99 + centroid[0] * 0.01

                centroids.append((centroid[1] + y, average))
        if len(centroids) > 0:
            return np.array(centroids)
        else:
            return None

    def centeroidnp(self, x, y):
        length = x.shape[0]
        sum_x = np.sum(x)
        sum_y = np.sum(y)
        return sum_x / length, sum_y / length




def main():
    rospy.init_node('road_detector', anonymous=False, log_level=rospy.DEBUG)
    # Get the parameters for the LLC node.
    # interface = rospy.get_param('car-interface')
    sim_mode = rospy.get_param('sim-mode')
    RoadDetector(sim_mode)

if __name__ == '__main__':
    main()
