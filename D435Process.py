import pyrealsense2 as rs
import numpy as np
import cv2
import math

align_to = rs.stream.color
align = rs.align(align_to)
hole_filler = rs.hole_filling_filter()

class D435Process:
    def __init__(self):
        pass

    def show(self, img):
        cv2.imshow('img', img)
        cv2.waitKey(1)

    def check(self, cnt):
        cnt_area = cv2.contourArea(cnt)
        if cnt_area < 200:
            return False
        _, radius = cv2.minEnclosingCircle(cnt)
        ratio = cnt_area / math.pi * radius * radius
        return ratio > 0.6

    def run(self):
        pipeline_d435 = rs.pipeline()
        config_d435 = rs.config()
        config_d435.enable_device('923322071945')
        config_d435.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        config_d435.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        profile_d435 = pipeline_d435.start(config_d435)

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
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # depth_image = cv2.blur(depth_image, (5, 5))
                # depth_image = cv2.bilateralFilter(depth_image, 7, 50, 50)
                # sobelx = cv2.Sobel(depth_image, cv2.CV_16SC1, 1, 0, ksize=5)
                # sobely = cv2.Sobel(depth_image, cv2.CV_16SC1, 0, 1, ksize=5)
                # edges = cv2.Canny(sobelx, sobely, 4000, 6000)

                depth_uint8 = np.maximum(255, depth_image.copy()/5).astype(np.uint8)
                depth_uint8 = cv2.bilateralFilter(depth_uint8, 7, 50, 50)
                depth_uint8 = cv2.bilateralFilter(depth_uint8, 7, 50, 50)
                edges = cv2.Canny(depth_uint8, 800, 1200)

                # edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
                contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = [cnt for cnt in contours if self.check(cnt)]

                cv2.drawContours(color_image, contours, 0, (0,255,0), 3)
                
                edges = np.stack([edges, edges, edges], axis=-1)
                edges = np.where(edges==255, edges, color_image)
                self.show(edges)

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                # depth_image = cv2.applyColorMap(
                #     cv2.convertScaleAbs(depth_image, alpha=0.03),
                #     cv2.COLORMAP_JET
                # )
                # depth_image_blur = cv2.applyColorMap(
                #     cv2.convertScaleAbs(depth_image_blur, alpha=0.03),
                #     cv2.COLORMAP_JET
                # )
                # Stack both images horizontally
                # images = np.hstack((depth_image_blur, depth_image))
                # self.show(images)

        finally:
            print('stop')
            pipeline_d435.stop()