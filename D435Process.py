import pyrealsense2 as rs
import numpy as np
import numpy.linalg as la
import cv2
import time
import multiprocessing as mp
from queue import Full
import logging

align_to = rs.stream.color
align = rs.align(align_to)

hole_filler = rs.hole_filling_filter()

MIN_DIS = 0.3
MAX_DIS = 3.0
DEPTH_H = 480
DEPTH_W = 640
FPS = 30
MAX_TOLERANCE_DIS = 0.015

hMin = 19
hMax = 30
sMin = 83
sMax = 193
vMin = 151
vMax = 255

class D435Process(mp.Process):
    def __init__(self, frame_queue, ball_queue):
        super().__init__()
        self.frame_queue = frame_queue
        self.ball_queue = ball_queue
        self.logger = logging.getLogger(__name__)

    def process_method(self):
        pipeline_d435 = rs.pipeline()
        config_d435 = rs.config()
        config_d435.enable_device('923322071945')
        config_d435.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
        config_d435.enable_stream(rs.stream.color, DEPTH_W, DEPTH_H, rs.format.bgr8, FPS)
        profile_d435 = pipeline_d435.start(config_d435)
        # K, K_inv = self.calculate_intrinsics(profile_d435)
        # normal_computer = cv2.rgbd.RgbdNormals_create(DEPTH_H, DEPTH_W, cv2.CV_32F, K)
        # plane_computer = cv2.rgbd.RgbdPlane_create(
        #     cv2.rgbd.RgbdPlane_RGBD_PLANE_METHOD_DEFAULT,
        #     int(DEPTH_W * DEPTH_H / 5000), int(DEPTH_W * DEPTH_H / 10), 0.02,
        #     0.01, 0, 0 # quadratic error
        # )
        # depth_sensor = profile_d435.get_device().first_depth_sensor()
        # preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
        # for i in range(int(preset_range.max)):
        #     visulpreset = depth_sensor.get_option_value_description(
        #         rs.option.visual_preset, i)
        #     if visulpreset == 'Default':
        #         print('set default')
        #         depth_sensor.set_option(rs.option.visual_preset, i)

        try:
            while True:
                frames = pipeline_d435.wait_for_frames()
                start_time = time.time()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    raise Exception(
                        "depth_frame and/or color_frame unavailable")
                color_image = np.asanyarray(color_frame.get_data())
                try:
                    self.ball_queue.put_nowait(False)
                except Full:
                    self.logger.warning('target_queue full')
                self.putFrame("intake", color_image)
                # Convert images to numpy arrays
                # depth_frame = hole_filler.process(depth_frame)
                # depth_image = np.asanyarray(depth_frame.get_data())
                # frame_HSV = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
                # thresh = cv2.inRange(frame_HSV, (19, 80, 105), (37, 255, 255))
                # self.show(thresh)
                #
                # if cv2.__version__.startswith('4'):
                #     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                #                                    cv2.CHAIN_APPROX_SIMPLE)
                # else:
                #     _, contours, _ = cv2.findContours(thresh,
                #         cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # for contour in contours:

                # image_3d = cv2.rgbd.depthTo3d(depth_image, K)
                # dis_to_cam = la.norm(image_3d, axis=-1)
                # normal = normal_computer.apply(image_3d)
                # labels, coeffs = plane_computer.apply(image_3d, normal)
                # if coeffs is None:
                #     plane_num = 0
                # else:
                #     plane_num = coeffs.shape[0]
                # if plane_num == 1:
                #     ground_plane = coeffs[0].flatten()
                #     dis_to_ground = self.calculate_plane_distance(image_3d,
                #                                                   ground_plane)
                #     mask = (dis_to_ground < 0.2) * (dis_to_cam > MIN_DIS) * \
                #            (dis_to_cam < MAX_DIS) * (labels == 255)
                #     mask = mask.astype(np.uint8) * 255
                #     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))
                #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                #                                    cv2.CHAIN_APPROX_SIMPLE)
                #     for index, contour in enumerate(contours):
                #         if cv2.contourArea(contour) < 100:
                #             continue
                #         cv2.drawContours(color_image, contours, index, (0, 255, 0))
                #
                #     # color_image[mask] = (255, 0, 0)
                # else:
                #     print("no ground")

                # no_label = labels == 255
                # if plane_num != 0:
                #     labels *= (128 // plane_num)
                # labels = cv2.applyColorMap(labels, cv2.COLORMAP_HSV)
                # labels[no_label] = (255, 255, 255)
                # output = cv2.addWeighted(color_image, 0.6, labels, 0.4, 0)
                # self.show(color_image)
                # print(time.time() - start_time)
                # self.show(output)
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
            self.logger.warning("frame_queue_full")

# pc.map_to(color_frame)
# points = pc.calculate(depth_frame)
# vertices = np.asanyarray(points.get_vertices(dims=2)).reshape(480, 640, 3)
# pc = rs.pointcloud()

def show(img):
    cv2.imshow('img', img)
    cv2.waitKey(1)


def calculate_intrinsics(self, profile):
    intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([
        [intrin.fx, 0, intrin.ppx],
        [0, intrin.fy, intrin.ppy],
        [0, 0, 1]
    ])
    K_inv = la.inv(K)
    return K, K_inv

def calculate_plane_distance(self, points, plane):
    return (np.dot(points, plane[:3]) + plane[3]) / np.sqrt(
        plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)
