import cv2
import logging
import numpy as np
import math
from queue import Full, Empty

from Constants import Constants


class CVProcess:
    def __init__(self, target_queue, xyz_rpy_value):
        self.target_queue = target_queue
        self.xyz_rpy_value = xyz_rpy_value
        self.logger = logging.getLogger(__name__)

    def show(self, img, name='img'):
        cv2.imshow('img', img)
        cv2.waitKey(1)

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        cap.set(cv2.CAP_PROP_EXPOSURE, -9)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640);
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480);

        while True:
            ret, frame = cap.read()
            if not ret:
                raise Exception('no frame')

            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(frame_HSV, Constants.HSV_LOW, Constants.HSV_HIGH)
            # if Constants.DEBUG:
                # self.show(thresh, 'HSV')
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                contour =    max(contours, key=cv2.contourArea)
            else:
                self.logger.debug("NO TARGET")
                continue
            if cv2.contourArea(contour) < Constants.MIN_TARGET_AREA:
                self.logger.debug(f'target area {cv2.contourArea(contour)}'
                              f'<{Constants.MIN_TARGET_AREA}, ignore')
                continue
            if len(contour) < 4:
                self.logger.debug('target contour less than 4 points, ignore')
                continue

            cv2.drawContours(frame, [contour], -1, (255, 0, 0))
            arg_extreme_points = [] # TL, BL, BR, TR
            for i in range(4):
                # TODO debug
                arg_extreme_points.append(np.dot(contour[:, 0, :], Constants.EXTREME_VECTOR[i]).argmax())
                pos = (contour[arg_extreme_points[i], 0, 0], contour[arg_extreme_points[i], 0, 1])
                cv2.circle(frame, pos, 4, (0, 0, 255), -1)
            extreme_points = np.array([(contour[arg_extreme_points[i], 0, 0],
                                        contour[arg_extreme_points[i], 0, 1])
                                       for i in range(4)])

            ret, rvec, tvec = cv2.solvePnP(Constants.TARGET_3D,
                                           np.array(extreme_points).astype(np.float32),
                                           Constants.CAMERA_MATRIX,
                                           Constants.DISTORTION_COEF)
            rvec = rvec / math.pi * 180
            pitch, row, yaw = rvec[:, 0]
            y, z, x = tvec[:, 0]
            y, z = -y, -z
            if ret:
                self.logger.debug(f"yaw {yaw:4.2f} row {row:4.2f} pitch {pitch:4.2f}")
                self.logger.debug(f"x {x:4.2f} y {y:4.2f} z {z:4.2f}")
                target_relative_xyz_rpy = (x, y, z, row, pitch, yaw)
                target_distance = math.sqrt(x*x + y*y + z*z)
                target_relative_dir_right = math.atan2(y, x) / math.pi * 180
                target_absolute_azm = self.xyz_rpy_value[5] + target_relative_dir_right
                self.logger.debug(f'distance {target_distance} '
                                  f'toleft {target_relative_dir_right} '
                                  f'abs_azm {target_absolute_azm}')
                try:
                    self.target_queue.put_nowait((target_distance,
                                                  target_relative_dir_right,
                                                  target_absolute_azm,
                                                  target_relative_xyz_rpy[5]))
                except Full:
                    # self.logger.warning('target_queue full')
                    pass
            else:
                self.logger.debug('target not matched')
            self.show(frame)
