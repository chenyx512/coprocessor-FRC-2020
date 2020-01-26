import pyrealsense2 as rs
import math as m
from queue import Full, Empty
import logging

class T265Process:
    def __init__(self, xyz_rpy_queue, xyz_rpy_value, encoder_v_queue):
        self.xyz_rpy_queue = xyz_rpy_queue
        self.xyz_rpy_value = xyz_rpy_value
        self.encoder_v_queue = encoder_v_queue
        self.logger = logging.getLogger(__name__)

    def run(self):
        pipeline_t265 = rs.pipeline()
        config_t265 = rs.config()
        # config_t265.enable_device('944222110230')
        profile_t265 = pipeline_t265.start(config_t265)

        try:
            while True:
                frames = pipeline_t265.wait_for_frames()
                pose = frames.get_pose_frame()
                if not pose:
                    raise Exception('no t265 pose')
                data = pose.get_pose_data()
                w = data.rotation.w
                x = -data.rotation.z
                y = data.rotation.x
                z = -data.rotation.y

                pitch = -m.asin(2.0 * (x * z - w * y)) * 180.0 / m.pi;
                roll = m.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z) * 180.0 / m.pi;
                yaw = m.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z) * 180.0 / m.pi;

                xyz_rpy = (-data.translation.z, -data.translation.x, data.translation.y,
                           roll, pitch, yaw)
                # self.logger.debug(f'xyz_ypr: {xyz_rpy}')
                self.xyz_rpy_value[0:6] = xyz_rpy
                try:
                    self.xyz_rpy_queue.put_nowait((xyz_rpy))
                except Full:
                    pass
                    # self.logger.warning('xyz_rpy_queue full')
        finally:
            print("stop")
            pipeline_t265.stop()