import cv2
import argparse
from operator import xor
import pyrealsense2 as rs
import numpy as np


def callback(value):
    pass

D435 = True
DEPTH_H = 480
DEPTH_W = 640
FPS = 30

def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def main():
    range_filter = 'HSV'
    setup_trackbars(range_filter)
    if not D435:
        cap = cv2.VideoCapture(0) # TODO change this for camera
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        cap.set(cv2.CAP_PROP_EXPOSURE, -9)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640);
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480);
    else:
        pipeline_d435 = rs.pipeline()
        config_d435 = rs.config()
        config_d435.enable_device('923322071945')
        config_d435.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16,
                                  FPS)
        config_d435.enable_stream(rs.stream.color, DEPTH_W, DEPTH_H,
                                  rs.format.bgr8, FPS)
        profile_d435 = pipeline_d435.start(config_d435)

    while True:
        if not D435:
            ret, image = cap.read()
        else:
            frames = pipeline_d435.wait_for_frames()
            image = frames.get_color_frame()
            image = np.asanyarray(image.get_data())

        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        # preview = cv2.bitwise_and(image, image, mask=thresh)
        cv2.imshow("after", thresh)
        cv2.imshow("before", image)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()

# H 90 - 97
# S 149 - 255
# V 46 - 255