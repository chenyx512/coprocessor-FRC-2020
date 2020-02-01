import logging

from Constants import Constants


class CameraServerWrapper:
    """
    This wraps around CameraServer so that it is optional
    My development computer cannot run CameraServer but Nano can
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        try:
            from cscore import CameraServer
            self.working = True
            self.video_server = {}
        except ModuleNotFoundError:
            self.working = False
            self.logger.warning('CameraServer not installed')

    def put_frame(self, name, frame):
        if not self.working:
            return
        if not name in self.video_server:
            self.video_server[name] = CameraServer.getInstance().\
                putVideo('frame', frame.shape[1], frame.shape[0])
        self.video_server[name].putFrame(frame)
