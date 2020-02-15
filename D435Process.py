import pyrealsense2 as rs
import numpy as np
import cv2
import math
from util.PlaneFitting import fit_plane

align_to = rs.stream.color
align = rs.align(align_to)
pc = rs.pointcloud()
hole_filler = rs.hole_filling_filter()

A = 1
B = 1
C = 1
D = 1
NORM_ABC = np.linalg.norm([A, B, C])
# Ax + By + Cz + D = 0

class D435Process:
    def __init__(self):
        pass

    def show(self, img):
        cv2.imshow('img', img)
        cv2.waitKey(1)

    def run(self):
        pipeline_d435 = rs.pipeline()
        config_d435 = rs.config()
        config_d435.enable_device('923322071945')
        config_d435.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config_d435.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile_d435 = pipeline_d435.start(config_d435)
        K, K_inv = self.calculate_intrinsics(profile_d435)
        xx, yy = np.meshgrid(np.arange(640), np.arange(480))
        xx = xx.reshape((-1))
        yy = yy.reshape((-1))
        zz = np.ones(xx.size)
        image_3d_coord = np.stack((xx, yy, zz), axis=1).reshape((480, 640, 3))
        distance_to_plane = self.calculate_plane_distance(image_3d_coord)

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
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    raise Exception(
                        "depth_frame and/or color_frame unavailable")
                # Convert images to numpy arrays
                depth_frame = hole_filler.process(depth_frame)
                depth_image = np.asanyarray(depth_frame.get_data()).\
                    astype(np.float32) / 1000.0 # now in meters
                color_image = np.asanyarray(color_frame.get_data())
                image_3d_coord = np.matmul(K_inv, image_2d_coord) * depth_image

                pc.map_to(color_frame)
                points = pc.calculate(depth_frame)
                vertices = np.asanyarray(points.get_vertices(dims=2)).reshape(-1, 3)
                # TODO check vertices (h, w, 3) or (w, h, 3)
                # fit_plane(vertices[:,0], vertices[:,1], vertices[:,2])
                break
        finally:
            print('stop')
            pipeline_d435.stop()

    def calculate_intrinsics(self, profile):
        intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        K = np.array([
            [intrin.fx, 0, intrin.ppx],
            [0, intrin.fy, intrin.ppy],
            [0, 0, 1]
        ])
        K_inv = np.linalg.inv(K)
        return K, K_inv

    def calculate_plane_distance(self, points):
        return (np.dot(points, [A, B, C]) + D) / NORM_ABC