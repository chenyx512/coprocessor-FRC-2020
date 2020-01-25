import cv2
from Constants import Constants
import logging
import numpy as np


class CVProcess:
    def __init__(self, target_queue):
        self.target_queue = target_queue
        self.logger = self.logger.getLogger(__name__)

    def show(self, img):
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

            extreme_points = [] # TL, BL, BR, TR
            for i in range(4):
                # TODO debug
                extreme_points.append(np.dot(Constants.EXTREME_VECTOR, contour).argmax())
                cv2.circle(frame, contour[extreme_points[-1]], 4, (0, 0, 255), -1)
            ret, rvec, tvec = cv2.solvePnP(Constants.TARGET_3D, extreme_points,
                                           Constants.CAMERA_MATRIX, Constants.DISTORTION_COEF)
            if ret:
                self.logger.debug(f'matched target rvec{rvec} tvec{tvec}')
            else:
                self.logger.debug('target not matched')
            self.show(frame)

