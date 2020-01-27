import cv2
import logging
import numpy as np
import math
from queue import Full, Empty
import multiprocessing as mp

from Constants import Constants


class CVProcess(mp.Process):
    def __init__(self, target_queue, xyzrpy_value):
        super().__init__()
        self.target_queue = target_queue
        self.xyz_rpy_value = xyzrpy_value
        self.logger = logging.getLogger(__name__)

    def process_method(self):
        cap = cv2.VideoCapture(3) # change when debugging on local
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, Constants.EXPOSURE_AUTO)
        cap.set(cv2.CAP_PROP_EXPOSURE, Constants.EXPOSURE_ABS)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Constants.HEIGHT)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, Constants.WIDTH)

        while True:
            ret, frame = cap.read()
            if not ret:
                raise Exception('no frame')
            # acquire the yaw when frame is captured, more accurate if extrapolate
            frame_yaw = self.xyz_rpy_value[5]

            # HSV filtering
            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(frame_HSV, Constants.HSV_LOW, Constants.HSV_HIGH)
            # thresh = cv2.blur(thresh, (5, 5)) # may not be necessary
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)

            # find target contour
            good_contour = None
            for contour in contours:
                if cv2.contourArea(contour) < Constants.MIN_TARGET_AREA:
                    continue
                epsilon = 0.02 * cv2.arcLength(contour, closed=True)
                approx = cv2.approxPolyDP(contour, epsilon, closed=True)
                if 7 <= len(approx) <= 9:
                    if good_contour is not None:
                        self.logger.warning('two good contours found, break')
                        good_contour = None
                        break
                    good_contour = approx
            if good_contour is None:
                # self.logger.debug('no good contour')
                if Constants.DEBUG:
                    cv2.imshow('target', frame)
                    cv2.waitKey(1)
                continue

            # get the four corners
            arg_extreme_points = np.matmul(Constants.EXTREME_VECTOR,
                                           good_contour[:, 0, :].transpose()).argmax(axis=-1)
            extreme_points = np.take(good_contour, arg_extreme_points, axis=0).squeeze()
            if Constants.DEBUG:
                cv2.drawContours(frame, [good_contour], -1, (255, 0, 0))
                for point in extreme_points:
                    cv2.circle(frame, tuple(point), 4, (0, 0, 255), -1)
                cv2.imshow('target', frame)
                cv2.imshow('filter', thresh)
                cv2.waitKey(1)

            # solvePnP
            ret, rvec, tvec = cv2.solvePnP(Constants.TARGET_3D,
                                           extreme_points.astype(np.float32),
                                           Constants.CAMERA_MATRIX,
                                           Constants.DISTORTION_COEF)
            if not ret:
                self.logger.debug('target not matched')
                continue
            # transform to WPI robotics convention
            pitch, row, yaw = rvec[:, 0] / math.pi * 180
            y, z, x = tvec[:, 0] * (-1, -1, 1)
            target_relative_xyzrpy = (x, y, z, row, pitch, yaw)
            self.logger.debug("relative xyz_ypr:" +
                              ''.join(f"{v:7.2f}" for v in target_relative_xyzrpy))
            # target distance too big means it is wrong
            target_distance = math.sqrt(x * x + y * y + z * z)
            if target_distance > Constants.MAX_TARGET_DISTANCE:
                self.logger.warning(f'target distance {target_distance} too big')
                continue

            target_relative_dir_right = math.atan2(y, x) / math.pi * 180
            target_absolute_azm = frame_yaw + target_relative_dir_right
            self.logger.debug(f'distance {target_distance:5.2f} '
                              f'toleft {target_relative_dir_right:5.1f} '
                              f'abs_azm {target_absolute_azm:5.1f}')
            try:
                self.target_queue.put_nowait((target_distance,
                                              target_relative_dir_right,
                                              target_absolute_azm,
                                              target_relative_xyzrpy[0:6]))
            except Full:
                self.logger.warning('target_queue full')

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")
