import math as m
import pyrealsense2 as rs
import numpy as np
import numpy.linalg as la
import cv2
from statistics import mean
import multiprocessing as mp
from queue import Full
import logging
import random

align_to = rs.stream.depth
align = rs.align(align_to)

hole_filler = rs.hole_filling_filter()

MIN_DIS = 0.3
MAX_DIS = 2.5
DEPTH_H = 240
DEPTH_W = 424
FPS = 30

DIS_RADIUS_PRODUCT_MIN = 15
DIS_RADIUS_PRODUCT_MAX = 27

hMin = 13
hMax = 40
sMin = 40
sMax = 255
vMin = 90
vMax = 255

HEIGHT = 15 * 0.0254

class D435Process(mp.Process):
    def __init__(self, frame_queue, ball_queue, xyzrpy_value):
        super().__init__()
        self.frame_queue = frame_queue
        self.ball_queue = ball_queue
        self.xyz_rpy_value = xyzrpy_value
        self.logger = logging.getLogger(__name__)

    def process_method(self):
        pipeline_d435 = rs.pipeline()
        config_d435 = rs.config()
        config_d435.enable_device('923322071945')
        config_d435.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
        config_d435.enable_stream(rs.stream.color, DEPTH_W, DEPTH_H, rs.format.bgr8, FPS)
        profile_d435 = pipeline_d435.start(config_d435)
        K, K_inv = calculate_intrinsics(profile_d435)
        normal_computer = cv2.rgbd.RgbdNormals_create(DEPTH_H, DEPTH_W, cv2.CV_32F, K)
        plane_computer = cv2.rgbd.RgbdPlane_create(
            cv2.rgbd.RgbdPlane_RGBD_PLANE_METHOD_DEFAULT,
            int(DEPTH_W * DEPTH_H / 5000), int(DEPTH_W * DEPTH_H / 15), 0.013,
            0.01, 0, 0 # quadratic error
        )
        depth_sensor = profile_d435.get_device().first_depth_sensor()
        preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
        for i in range(int(preset_range.max)):
            visulpreset = depth_sensor.get_option_value_description(
                rs.option.visual_preset, i)
            if visulpreset == 'Default':
                print('set default')
                depth_sensor.set_option(rs.option.visual_preset, i)

        try:
            while True:
                frames = pipeline_d435.wait_for_frames()
                frame_yaw = self.xyz_rpy_value[5]

                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    raise Exception(
                        "depth_frame and/or color_frame unavailable")
                color_image = np.asanyarray(color_frame.get_data())

                # Convert images to numpy arrays
                depth_frame = hole_filler.process(depth_frame)
                depth_image = np.asanyarray(depth_frame.get_data())

                frame_HSV = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
                thresh = cv2.inRange(frame_HSV, (hMin, sMin, vMin),
                                                (hMax, sMax, vMax))
                out_range_thresh = cv2.inRange(frame_HSV, (0, 0, 0), (0, 0, 0))
                thresh = cv2.bitwise_or(thresh, out_range_thresh)
                # show(thresh)
                image_3d = cv2.rgbd.depthTo3d(depth_image, K)
                normal = normal_computer.apply(image_3d)
                plane_labels, plane_coeffs = plane_computer.apply(image_3d, normal)
                dis_to_cam = la.norm(image_3d, axis=-1)
                mask = (dis_to_cam < MAX_DIS) * (dis_to_cam > MIN_DIS) * \
                       (plane_labels == 255)
                mask = mask.astype(np.uint8) * thresh
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                # show(mask)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)

                ball_dis = 1e9
                ball_angle = 0
                best_score = 1e9
                for index, contour in enumerate(contours):
                    contour_area = cv2.contourArea(contour)
                    if contour_area < 100:
                        continue
                    (x_2d, y_2d), r = cv2.minEnclosingCircle(contour)
                    if contour_area / (m.pi * r * r) < 0.55:
                        continue
                    dis = circle_sample(image_3d, x_2d, y_2d, r)
                    if dis < MIN_DIS:
                        continue
                    if not DIS_RADIUS_PRODUCT_MIN < dis * r < DIS_RADIUS_PRODUCT_MAX:
                        continue
                    pt_3d = K_inv @ [x_2d, y_2d, 1] * dis
                    angle = m.degrees(m.atan(pt_3d[0] / dis))
                    # convert dis to planer dis for output
                    dis_2d = m.sqrt(max(0.1, dis * dis - HEIGHT * HEIGHT))
                    score = dis_2d + abs(angle) / 40
                    x_2d, y_2d, r = int(x_2d), int(y_2d), int(r)
                    if score < best_score:
                        ball_dis = dis_2d
                        ball_angle = angle
                        ball_circle = ((x_2d, y_2d), r)
                        best_score = score
                    cv2.drawContours(color_image, contours, index, (200, 0, 200), 2)
                    cv2.circle(color_image, (x_2d, y_2d), r, (0, 0, 200), 2)

                # thresh = np.stack((thresh, thresh, thresh), axis=-1)
                # mask = np.stack((mask, mask, mask), axis=-1)
                # output = cv2.vconcat([thresh, mask, color_image])
                # show(output)
                if ball_dis < 10:
                    pos, r = ball_circle
                    print('good')
                    cv2.circle(color_image, pos, 10, (255, 0, 0), -1)
                try:
                    self.ball_queue.put_nowait((ball_dis, -ball_angle,
                                                frame_yaw - ball_angle))
                except Full:
                    pass
                # show(color_image)
                self.putFrame("intake", color_image)
        finally:
            print('stop')
            pipeline_d435.stop()

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")

    def putFrame(self, name, frame):
        try:
            self.frame_queue.put_nowait((name, frame))
        except Full:
            # self.logger.warning("frame_queue_full")
            pass


def circle_sample(image_3d, x, y, r):
    adjusted_r = int(1 / m.sqrt(2) * r * 0.8)
    dis_list = []
    for i in range(100):
        new_x = int(x + random.randint(0, adjusted_r * 2) - adjusted_r)
        new_y = int(y + random.randint(0, adjusted_r * 2) - adjusted_r)
        if not (0 <= new_x < DEPTH_W and 0 <= new_y < DEPTH_H):
            continue
        x_3d, y_3d, z_3d = image_3d[new_y, new_x]
        dis = m.sqrt(x_3d * x_3d + y_3d * y_3d + z_3d * z_3d)
        if not (MIN_DIS <= dis <= MAX_DIS):
            continue
        dis_list.append(dis)
    if len(dis_list) == 0:
        return 0
    return mean(dis_list)

def show(img):
    cv2.imshow('img', img)
    cv2.waitKey(1)


def calculate_intrinsics(profile):
    intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([
        [intrin.fx, 0, intrin.ppx],
        [0, intrin.fy, intrin.ppy],
        [0, 0, 1]
    ])
    K_inv = la.inv(K)
    return K, K_inv

def calculate_plane_distance(points, plane):
    return (np.dot(points, plane[:3]) + plane[3]) / np.sqrt(
        plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)
