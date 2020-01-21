import pyrealsense2 as rs
import math as m

class T265Process:
    def __init__(self):
        pass

    def run(self):
        pipeline_t265 = rs.pipeline()
        config_t265 = rs.config()
        config_t265.enable_stream(rs.stream.pose)
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

                print("Frame #{}".format(pose.frame_number))
                print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))
                print("Position: {}".format(data.translation))
        finally:
            print("stop")
            pipeline_t265.stop()