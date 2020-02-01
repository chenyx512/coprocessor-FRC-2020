import pyrealsense2 as rs
import math as m
from queue import Full, Empty
import multiprocessing as mp
import logging


class T265Process(mp.Process):
    def __init__(self, xyz_rpy_queue, xyz_rpy_value, encoder_v_queue):
        super().__init__()
        self.xyz_rpy_queue = xyz_rpy_queue
        self.xyz_rpy_value = xyz_rpy_value
        self.encoder_v_queue = encoder_v_queue
        self.logger = logging.getLogger(__name__)
        self.daemon = True

    def process_method(self):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        profile = cfg.resolve(pipe)
        dev = profile.get_device()
        tm2 = dev.as_tm2()
        if not tm2:
            raise Exception("t265 not found")

        pose_sensor = tm2.first_pose_sensor()
        wheel_odometer = pose_sensor.as_wheel_odometer()

        with open("config/calibration_odometry.json") as f:
            chars = []
            for line in f:
                for c in line:
                    chars.append(ord(c))  # char to uint8
        wheel_odometer.load_wheel_odometery_config(chars)
        # this method up to here is from example, don't modify

        cnt = 0
        self.logger.info("start pipeline")
        pipe.start(cfg)
        try:
            while True:
                # get pose
                frames = pipe.wait_for_frames(timeout_ms=1000)
                pose = frames.get_pose_frame()
                if not pose:
                    raise Exception('no t265 pose')
                xyzrpy = get_xyzrpy(pose.get_pose_data())

                cnt += 1
                if cnt % 400 == 0:
                    self.logger.debug("xyz_ypr:" +
                                      "".join(f"{v:7.2f}" for v in xyzrpy))

                self.xyz_rpy_value[0:6] = xyzrpy
                try:
                    self.xyz_rpy_queue.put_nowait(xyzrpy)
                except Full:
                    pass

                try:
                    speed = self.encoder_v_queue.get_nowait()
                    v = rs.vector()
                    v.z = -speed # this in camera frame
                    wheel_odometer.send_wheel_odometry(0, 0, v)
                except Empty:
                    pass
        finally:
            pipe.stop()

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")

def get_xyzrpy(data):
    w = data.rotation.w
    x = -data.rotation.z
    y = data.rotation.x
    z = -data.rotation.y

    pitch = -m.asin(2.0 * (x * z - w * y)) * 180.0 / m.pi
    roll = m.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z) * 180.0 / m.pi
    yaw = m.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z) * 180.0 / m.pi

    xyzrpy = (-data.translation.z, -data.translation.x, data.translation.y,
               roll, pitch, -yaw)
    return xyzrpy

"""
given t265  xvec0, theta0
      field xvec1, theta1
calibrated_dtheta = theta1 - theta0
let R be the rotation vector of calibrated_dtheta
calibrated_dt = xvec1 - R dxvec0
field coord = R xvec_t265 + calibrated_dt
ntable store calibrated_dtheta and 
"""