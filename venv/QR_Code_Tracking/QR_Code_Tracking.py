from __future__ import print_function
import zbar
from PIL import Image, ImageColor
import cv2
import numpy as np
import math
from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import sys, os
from optparse import OptionParser
import argparse

parser = argparse.ArgumentParser(
    description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print("Arming motors")
vehicle.armed = True
time.sleep(0.5)

cap = cv2.VideoCapture(0)

scanner = zbar.ImageScanner()
scanner.parse_config('enable')

vehicle.channels.overrides[3] = 1200
time.sleep(2)
vehicle.channels.overrides[3] = 1300
time.sleep(2)
vehicle.channels.overrides[3] = 1400
time.sleep(2)
vehicle.channels.overrides[3] = 1500
time.sleep(2)

vehicle.channels.overrides[1] = 1500

while True:

    ret, im = cap.read()
    if not ret:
        continue

    size = im.shape
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY, dstCn=0)
    pil = Image.fromarray(gray)
    width, height = pil.size
    raw = pil.tobytes()
    image = zbar.Image(width, height, 'Y800', raw)
    scanner.scan(image)

    cv2.line(im, (0, 240), (640, 240), (0, 0, 255), 1)
    cv2.line(im, (320, 0), (320, 480), (0, 0, 255), 1)
    cv2.line(im, (0, 0), (0, 480), (0, 0, 255), 1)
    cv2.line(im, (0, 479), (640, 479), (0, 0, 255), 1)
    cv2.line(im, (639, 480), (639, 0), (0, 0, 255), 1)
    cv2.line(im, (0, 1), (640, 1), (0, 0, 255), 1)

    for symbol in image:
        # print('decoded', symbol.type, 'symbol', '"%s"' % symbol.data)
        topLeftCorners, bottomLeftCorners, bottomRightCorners, topRightCorners = [item for item in symbol.location]
        cv2.line(im, topLeftCorners, topRightCorners, (0, 255, 0), 3)
        cv2.line(im, topLeftCorners, bottomLeftCorners, (0, 255, 0), 3)
        cv2.line(im, topRightCorners, bottomRightCorners, (0, 255, 0), 3)
        cv2.line(im, bottomLeftCorners, bottomRightCorners, (0, 255, 0), 3)

        # 2D image points. If you change the image, you need to change vector

        image_points = np.array([
            (int((topLeftCorners[0] + topRightCorners[0]) / 2), int((topLeftCorners[1] + bottomLeftCorners[1]) / 2)),
            # Nose
            topLeftCorners,  # Left eye left corner
            topRightCorners,  # Right eye right corne
            bottomLeftCorners,  # Left Mouth corner
            bottomRightCorners  # Right mouth corner
        ], dtype="double")

        # 3D model points.
        model_points = np.array([
            (0.0, 0.0, 0.0),  # Nose
            (-225.0, 170.0, -135.0),  # Left eye left corner
            (225.0, 170.0, -135.0),  # Right eye right corne
            (-150.0, -150.0, -125.0),  # Left Mouth corner
            (150.0, -150.0, -125.0)  # Right mouth corner

        ])

        # Camera internals

        focal_length = size[1]
        center = (size[1] / 2, size[0] / 2)
        camera_matrix = np.array(
            [[focal_length, 0, center[0]],
             [0, focal_length, center[1]],
             [0, 0, 1]], dtype="double"
        )

        # print("Camera Matrix :\n {0}".format(camera_matrix))

        dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
        (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix,
                                                                      dist_coeffs, flags=cv2.CV_ITERATIVE)

        rmat = cv2.Rodrigues(rotation_vector)[0]

        # print(rmat)
        # print("Rotation Vector:\n {0}".format(rotation_vector))
        # print("Translation Vector:\n {0}".format(translation_vector))

        # Project a 3D point (0, 0, 1000.0) onto the image plane.
        # We use this to draw a line sticking out of the nose

        (nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 100.0)]), rotation_vector,
                                                         translation_vector, camera_matrix, dist_coeffs)

        for p in image_points:
            cv2.circle(im, (int(p[0]), int(p[1])), 3, (0, 0, 255), 5)

        p1 = (int(image_points[0][0]), int(image_points[0][1]))
        p2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))

        # pillars
        cv2.line(im, bottomLeftCorners, p2, (0, 255, 0), 3)
        cv2.line(im, topLeftCorners, p2, (0, 255, 0), 3)
        cv2.line(im, bottomRightCorners, p2, (0, 255, 0), 3)
        cv2.line(im, topRightCorners, p2, (0, 255, 0), 3)

        x = int(image_points[0][0])
        y = int(image_points[0][1])

        while True:
            if (0 < x < 640):
                if (0 < x < 64):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1000
                    print(int(vehicle.channels.overrides[1]))

                elif (64 < x < 128):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1100
                    print(int(vehicle.channels.overrides[1]))

                elif (128 < x < 192):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1200
                    print(int(vehicle.channels.overrides[1]))

                elif (192 < x < 256):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1300
                    print(int(vehicle.channels.overrides[1]))


                elif (256 < x < 319):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1400
                    print(int(vehicle.channels.overrides[1]))

                elif x == 320:
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1500
                    print(int(vehicle.channels.overrides[1]))

                elif (321 < x < 384):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1600
                    print(int(vehicle.channels.overrides[1]))

                elif (384 < x < 448):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1700
                    print(int(vehicle.channels.overrides[1]))

                elif (448 < x < 512):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1800
                    print(int(vehicle.channels.overrides[1]))

                elif (512 < x < 576):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 1900
                    print(int(vehicle.channels.overrides[1]))

                elif (576 < x < 640):
                    vehicle.channels.overrides[3] = 1600
                    vehicle.channels.overrides[1] = 2000
                    print(int(vehicle.channels.overrides[1]))

            else:
                print("out of vision")

            break

            # Display image
    # cv2.namedWindow("Output", cv2.WND_PROP_FULLSCREEN)
    # cv2.setWindowProperty("Output", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
    cv2.imshow("Output", im)

    # Wait for the magic key
    keypress = cv2.waitKey(1) & 0xFF
    if keypress == ord('q'):
        break

cv2.waitKey(0)
