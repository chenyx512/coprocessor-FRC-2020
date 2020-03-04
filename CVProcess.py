import cv2
import logging
import numpy as np
import math
from queue import Full, Empty
import multiprocessing as mp

from Constants import Constants


class CVProcess(mp.Process):
    DEBUG_PERIOD = 30

    def __init__(self, target_queue, xyzrpy_value, frame_queue,
                 ntable_queue):
        super().__init__()
        self.target_queue = target_queue
        self.xyz_rpy_value = xyzrpy_value
        self.logger = logging.getLogger(__name__)
        self.frame_queue = frame_queue
        self.ntable_queue = ntable_queue
        self.cnt = 0

    def process_method(self):
        cap = cv2.VideoCapture(0) # change when debugging on local
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, Constants.EXPOSURE_AUTO)
        cap.set(cv2.CAP_PROP_EXPOSURE, Constants.EXPOSURE_ABS)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Constants.HEIGHT)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, Constants.WIDTH)

        while True:
            self.cnt += 1
            ret, frame = cap.read()
            if not ret:
                raise Exception('no frame')
            # acquire the yaw when frame is captured, more accurate if extrapolate
            frame_yaw = self.xyz_rpy_value[5]

            # HSV filtering
            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(frame_HSV, Constants.HSV_LOW, Constants.HSV_HIGH)
            # thresh = cv2.blur(thresh, (5, 5)) # may not be necessary
            if cv2.__version__.startswith('4'):
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
            else:
                _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
            thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            thresh = cv2.addWeighted(thresh, 0.6, frame, 0.4, 0)

            # find target contour
            good_contour = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < Constants.MIN_TARGET_AREA:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if area / w / h > Constants.MAX_TARGET2RECT_RATIO:
                    continue
                epsilon = 0.004 * cv2.arcLength(contour, closed=True)
                approx = cv2.approxPolyDP(contour, epsilon, closed=True)
                cv2.drawContours(thresh, [approx], 0, (255, 0, 0), 3)
                if 7 <= len(approx):
                    if good_contour is not None:
                        self.logger.warning('two good contours found, break')
                        good_contour = None
                        break
                    good_contour = contour
            
            if good_contour is None:
                self.putFrame('shoot', thresh)
                self.debug('no good contour')
                self.put_no_target()
                continue

            # get the four corners
            arg_extreme_points = np.matmul(Constants.EXTREME_VECTOR,
                                           good_contour[:, 0, :].T).argmax(axis=-1)
            extreme_points = np.take(good_contour, arg_extreme_points, axis=0).squeeze()
            cv2.drawContours(thresh, [good_contour], -1, (255, 0, 0))
            for point in extreme_points:
                cv2.circle(thresh, tuple(point), 4, (0, 0, 255), -1)
            self.putFrame('shoot', thresh)

            # solvePnP
            ret, rvec, tvec = cv2.solvePnP(Constants.TARGET_3D,
                                           extreme_points.astype(np.float32),
                                           Constants.CAMERA_MATRIX,
                                           Constants.DISTORTION_COEF)
            if not ret:
                self.logger.warning('target not matched')
                self.put_no_target()
                continue

            # calculate field coordinate
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            field_xyz = np.matmul(rotation_matrix.T, -tvec).flatten()
            field_x, field_y, field_z = field_xyz
            self.debug("field_coord:" +
                       ''.join(f"{v:7.2f}" for v in field_xyz))

            target_distance = math.hypot(field_x, field_y)
            # target distance too big means it is wrong
            if target_distance > Constants.MAX_TARGET_DISTANCE or target_distance < 2.0:
                self.logger.warning(f'target distance {target_distance} out of range')
                self.put_no_target()
                continue

            # transform to WPI robotics convention
            y, z, x = tvec[:, 0] * (-1, -1, 1)
            target_relative_dir_left = math.atan2(y, x) / math.pi * 180
            target_t265_azm = frame_yaw + target_relative_dir_left
            field_theta = math.atan2(field_y, field_x) / math.pi * 180\
                          - 180 - target_relative_dir_left

            P = np.hstack((rotation_matrix, tvec))
            eul = -cv2.decomposeProjectionMatrix(P)[6]
            yaw = 180 * eul[1, 0] / math.pi  # warn: singularity if camera is facing perfectly upward. Value 0 yaw is given by the Y-axis of the world frame.
            pitch = 180 * ((eul[0, 0] + math.pi / 2) * math.cos(eul[1, 0])) / math.pi
            roll = 180 * ((-(math.pi / 2) - eul[0, 0]) * math.sin(eul[1, 0]) + eul[2, 0]) / math.pi
            for c, v in zip("ypr", [yaw, pitch, roll]):
                self.putNtable(f'CV/camera_{c}', v % 360)
            # reject target for bad YPR

            self.debug(f'distance {target_distance:5.2f} '
                       f'toleft {target_relative_dir_left:5.1f} '
                       f'field_theta {field_theta:5.1f} ')
            try:
                self.target_queue.put_nowait((True,
                                              target_distance,
                                              target_relative_dir_left,
                                              target_t265_azm,
                                              [field_x, field_y, field_theta]))
            except Full:
                self.logger.warning('target_queue full')

    def put_no_target(self):
        try:
            self.target_queue.put_nowait((False, None, None, None, None))
        except Full:
            self.logger.warning('target_queue full')

    def debug(self, msg):
        if self.cnt % self.DEBUG_PERIOD == 0:
            self.logger.debug(msg)

    def putFrame(self, name, frame):
        try:
            self.frame_queue.put_nowait((name, frame))
        except Full:
            self.logger.warning("frame_queue_full")

    def putNtable(self, key, value):
        try:
            self.ntable_queue.put_nowait((key, value))
        except Full:
            self.logger.warning("ntable_queue_full")

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")
