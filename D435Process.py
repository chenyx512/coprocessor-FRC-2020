import pyrealsense2 as rs
import numpy as np
import cv2

align_to = rs.stream.color
align = rs.align(align_to)
hole_filler = rs.hole_filling_filter()

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
        config_d435.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config_d435.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        profile_d435 = pipeline_d435.start(config_d435)


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


                laplacian = cv2.Laplacian(depth_image, cv2.CV_64F)
                # self.show(laplacian)
                # sobelx = cv2.Sobel(depth_image, cv2.CV_16SC1, 1, 0, ksize=5)
                # sobely = cv2.Sobel(depth_image, cv2.CV_16SC1, 0, 1, ksize=5)
                # edges = cv2.Canny(sobelx, sobely, 900, 900)

                depth_uint8 = np.maximum(255, depth_image.copy()/5).astype(np.uint8)
                kernel = np.ones((7,7),np.uint8)
                erosion = cv2.erode(depth_uint8, kernel=kernel, iterations = 4)
                edges = cv2.Canny(erosion, 30, 120)
                dilation = cv2.dilate(erosion, kernel=kernel, iterations = 4)
                contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 50]
                cv2.drawContours(color_image, contours, -1, (0,255,0), 3)
                
                

                edges = np.stack([edges, edges, edges], axis=-1)
                # edges = np.where(edges==255, edges, color_image)
                self.show(color_image)

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET)

                # Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))

        finally:
            print('stop')
            pipeline_d435.stop()