import logging
import cv2
try:
    from cscore import CameraServer
    WORKING = True
except ModuleNotFoundError:
    WORKING = False

from Constants import Constants


class CameraServerWrapper:
    WIDTH = 320
    HEIGHT = 240
    """
    This wraps around CameraServer so that it is optional
    My development computer cannot run CameraServer but Nano can
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.video_server = CameraServer.getInstance(). \
                putVideo('nano_output', self.HEIGHT, self.WIDTH)
        self.output = ""
        if not WORKING:
            self.logger.warning('CameraServer not installed')

    def set_output(self, name):
        self.output = name

    def put_frame(self, name, frame):
        if not WORKING:
            return
        if name in self.output:
            self.video_server.putFrame(cv2.resize(frame,
                                                  (self.WIDTH, self.HEIGHT)))
