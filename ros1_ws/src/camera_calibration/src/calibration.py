# @Author: Jose Rojas <jrojas>
# @Date:   2018-07-19T01:38:02-07:00
# @Email:  jrojas@redlinesolutions.co
# @Last modified by:   jrojas
# @Last modified time: 2018-07-22T01:45:00-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import numpy as np
import cv2 as cv
import glob
import json
import os, argparse, sys

# taken from: https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html

def undistort_images(images, camera_model):
    ret_images = []
    for img in images:
        ret_images.append(cv.undistort(img, camera_model["K"], camera_model["D"]))
    return ret_images

def generate_calibration_features(images, measurement_data, chessboard_size):
    """
    Determines calibration features from a set of images

    Parameters:
        images -- images with chessboard calibration rig features
        chessboard_size -- shape [2 dim] of chessboard features, in rows and columns
    Returns:
        An 2D array of arrays of 3D feature points found in the image.
    """
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    total_count = chessboard_size[0] * chessboard_size[1]
    imgpoints = [] # 2d points in image plane.
    valid_images = []
    valid_data = []
    i = 0
    for img, data in zip(images, measurement_data):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            corners = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            valid_images.append(img)
            valid_data.append(data)
        else:
            print("No points found in image {}".format(i))
        i+=1

    return imgpoints, valid_images, valid_data


def calculate_control_points(measurement_data, chessboard_size):

    arr_points = []

    for data in measurement_data:
        origin_type = data['origin']
        square_size = float(data['square_size'])

        # the distance from the ground (Y is negative going upwards)
        height = float(data['height'])

        # the X distance along the YZ plane (assumes left is negative, right is positive)
        lateral = float(data['lateral'])

        points = []

        if origin_type == 'bottom_middle':
            lateral -= square_size * float(chessboard_size[1] - 1) / 2
            y = [ - i * square_size - height for i in range(1, chessboard_size[0] + 1)]
            x = [i * square_size + lateral for i in range(0, chessboard_size[1])]

            for i in range(0, chessboard_size[1]):
                for j in range(0, chessboard_size[0]):
                    points.append([x[i],y[j],data['z']])
        else:
            assert False, "Unknown origin_type {}".format(origin_type)

        arr_points.append(points)

    return np.array(arr_points, dtype=np.float32)

def calibrate_camera(controlpoints, imgpoints, image_size, intrinsics=None, distortion=None):

    if intrinsics is None:
        print("Using calibrateCamera")
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(controlpoints, imgpoints, image_size, None, None)
    else:
        print("Using solvePnp (iterative)")
        mtx = None
        dist = distortion
        controlpoints = controlpoints.reshape((-1, 1, 3))
        imgpoints = np.array(imgpoints, dtype=np.float32).reshape((-1, 1, 2))
        print(controlpoints.shape)
        print(imgpoints.shape)
        ret, rvecs, tvecs = cv.solvePnP(controlpoints, imgpoints, intrinsics, distortion, flags=cv.SOLVEPNP_ITERATIVE)

    print("Reprojection error ", ret)
    print("Camera matrix", mtx)
    print("Distortion coeffs", dist)
    print("Translation vectors", tvecs)
    print("Rotation vectors", rvecs)


def display_calibration_features(images, arr_points, chessboard_size):
    for img, corners in zip(images, arr_points):
        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboard_size, corners, True)
        cv.imshow('img', img)
        cv.waitKey()
    cv.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', help="data directory; will contain calibration images and measurement data")
    parser.add_argument('--chessboard_size', default="7x9", help="the size of the chessboard; 'rowsxcols'")
    parser.add_argument('--camera_model', default=None, help="camera parameters file")
    parser.add_argument('--display', action="store_true", default=False, help="display checkboard images")
    parser.add_argument('--undistort', action="store_true", default=False, help="undistort images before processing")

    args = parser.parse_args(sys.argv[1:])
    chessboard_size = args.chessboard_size.split("x")
    chessboard_size = (int(chessboard_size[0]), int(chessboard_size[1]))

    image_names = glob.glob('{}/*.png'.format(args.directory))

    images = []
    jsons = []
    image_size = None
    for name in image_names:
        images.append(cv.imread(name))
        if image_size is None:
            image_size = images[0].shape
        else:
            assert image_size == images[-1].shape, "Images should all be the same shape"
        json_name = name.replace(".png", ".json")
        print("Reading {} file".format(json_name))
        fp = open(json_name, 'r')
        jsons.append(json.load(fp))
        fp.close()

    camera_model = None
    if args.camera_model is not None:
        fp = open("{}/{}".format(args.directory, args.camera_model))
        camera_model = json.load(fp)
        camera_model["K"] = np.array(camera_model["K"], dtype=np.float32).reshape(3, 3)
        camera_model["D"] = np.array(camera_model["D"], dtype=np.float32)
        fp.close()

        if args.undistort is True:
            images = undistort_images(images, camera_model)
            camera_model["D"] = np.array([0, 0, 0, 0, 0], dtype=np.float32)

        if args.display is True:
            for img in images:
                # Draw and display the corners
                cv.imshow('img', img)
                cv.waitKey()

    print(camera_model)

    imgpoints, valid_images, valid_measurement_data = generate_calibration_features(images, jsons, chessboard_size)

    for imgarr in imgpoints:
        assert len(imgarr) > 0

    print(imgpoints)

    if args.display:
        display_calibration_features(valid_images, imgpoints, chessboard_size)

    controlpoints = calculate_control_points(valid_measurement_data, chessboard_size)

    print(controlpoints)

    calibrate_camera(controlpoints, imgpoints, image_size[0:2],
        intrinsics=camera_model["K"] if camera_model else None,
        distortion=camera_model["D"] if camera_model else None
    )
