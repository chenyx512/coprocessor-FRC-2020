import pyrealsense2 as rs
import numpy as np
import cv2

class D435Process:
    def __init__(self):
        pass

    def run(self):
        pipeline_d435 = rs.pipeline()
        config_d435 = rs.config()
        config_d435.enable_device('923322071945')
        config_d435.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config_d435.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline_d435.start(config_d435)

        # config_t265 = rs.config()
        # config_t265.enable_device('944222110230')

        try:
            while True:
                frames = pipeline_d435.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not depth_frame or not color_frame:
                    raise Exception(
                        "depth_frame and/or color_frame unavailable")
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET)

                # Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))

                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)
        finally:
            pipeline_d435.stop()