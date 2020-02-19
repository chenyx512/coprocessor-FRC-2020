import logging
import logging.config
import multiprocessing as mp
import time
from datetime import datetime
from networktables import NetworkTables
from queue import Full, Empty
import yaml
import numpy as np
import numbers
import math

from T265Process import T265Process
from CVProcess import CVProcess
from Constants import Constants
from ProcessManager import ProcessManager
from util.Smoothers import MedianSmoother, MedianAngleSmoother
from util.CameraServerWrapper import CameraServerWrapper
import util.PoseTracker as PoseTracker


# logging setup
with open('config/logging_config.yaml', 'r') as yaml_file:
    logging_config = yaml.safe_load(yaml_file)
for _, handler in logging_config['handlers'].items():
    if 'filename' in handler:
        timestamp = datetime.now().strftime(r'%m%d_%H%M%S')
        handler['filename'] = str(f'log/{timestamp}.log')
logging.config.dictConfig(logging_config)

# shared
encoder_v_queue = mp.Queue(1)
xyz_rpy_queue = mp.Queue(1)
frame_queue = mp.Queue(10)
ntable_queue = mp.Queue(100)
xyzrpy_value = mp.Array('f', 6)
target_queue = mp.Queue(1)

# target
last_target_found_time = 0
target_dis_smoother = MedianSmoother(Constants.TARGET_SMOOTH_NUM)
target_field_theta_smoother = MedianAngleSmoother(Constants.TARGET_SMOOTH_NUM)


def encoder_callback(entry, key, value, is_new):
    if key == '/odom/encoder_v':
        try:
            encoder_v_queue.put_nowait(value)
        except Full:
            logging.warning('encoder_v queue full')

# NTable
logging.info(f'start networktable client for 3566')
NetworkTables.startClientTeam(3566)
odom_table = NetworkTables.getTable('odom')
NetworkTables.getEntry('/odom/encoder_v').addListener(
    encoder_callback,
    NetworkTables.NotifyFlags.UPDATE
)
camera_server = CameraServerWrapper()

# status flag
is_t265_connected = False
last_t265_connect_time = time.time()
is_CV_connected = False
last_CV_connect_time = time.time()
slave_handshake_cnt = 0

pose_tracker = PoseTracker.PoseTracker()

# processes
def t265_update():
    try:
        xyz_rpy_queue.get_nowait()
        xyz_rpy = xyzrpy_value[0:6]
        pose_tracker.update_t265_pose(xyz_rpy[0], xyz_rpy[1], xyz_rpy[5])
        for v, c in zip(pose_tracker.field_xyt, 'xyt'):
            odom_table.putNumber(f'field_{c}', v)
        for v, c in zip(pose_tracker.robot_xyt, 'xyt'):
            odom_table.putNumber(f'robot_{c}', v)
        return True
    except Empty:
        return False
t265_process_manager = ProcessManager(
    lambda: T265Process(xyz_rpy_queue, xyzrpy_value, encoder_v_queue),
    t265_update,
)

def cv_update():
    global last_target_found_time
    try:
        target_found, target_dis, target_relative_dir_left, \
                target_t265_azm, camera_xyt = target_queue.get_nowait()
        # TODO check if target is on opposite side (may not need to)
        if not target_found:
            pose_tracker.clear_calibration()
            if time.time() > last_target_found_time + Constants.HOLD_TARGET_TIME:
                odom_table.putBoolean('target_found', False)
                field_xyt = pose_tracker.field_xyt
                odom_table.putNumber(
                    "target_field_theta",
                    target_field_theta_smoother.update(
                        math.degrees(math.atan2(field_xyt[1], field_xyt[0]))
                        - 180 - PoseTracker.CV_THETA
                    )
                )
                odom_table.putNumber(
                    'target_dis',
                    target_dis_smoother.update(math.hypot(*field_xyt[0:2]))
                )
            return True

        odom_table.putBoolean('target_found', True)
        last_target_found_time = time.time()
        pose_tracker.update_CV_in_field(*camera_xyt)
        if odom_table.getBoolean('field_calibration_start', False):
            ret = pose_tracker.calibrate()
            logging.info(("GOOD" if ret else "BAD") + " field calibration")
            odom_table.putBoolean('field_calibration_start', False)
            odom_table.putBoolean('field_calibration_good', ret)

        for i, c in enumerate('xyt'):
            odom_table.putNumber(f'camera_field_{c}', camera_xyt[i])
        odom_table.putNumber(
            'target_field_theta',
            target_field_theta_smoother.update(
                target_t265_azm + pose_tracker.dtheta_r2f)
        )
        odom_table.putNumber('target_dis',
                             target_dis_smoother.update(target_dis))
        odom_table.putNumber('target_relative_dir_left',
                             target_relative_dir_left)

        if odom_table.getBoolean('field_calibration_good', False):
            odom_table.putNumber('error_xy', pose_tracker.calibration_error_x)
            odom_table.putNumber('error_theta', pose_tracker.calibration_error_theta)
        return True
    except Empty:
        return False

cv_process_manager = ProcessManager(
    lambda: CVProcess(target_queue, xyzrpy_value, frame_queue, ntable_queue),
    cv_update,
)

# main loop
while True:
    # get pose
    t265_process_manager.update()
    odom_table.putBoolean('pose_good', t265_process_manager.is_connected)

    # get CV
    cv_process_manager.update()
    odom_table.putBoolean('target_good', cv_process_manager.is_connected)

    # handshake
    odom_table.putNumber('client_time', time.time())

    # update camera server
    try:
        while True:
            name, frame = frame_queue.get_nowait()
            camera_server.put_frame(name, frame)
    except Empty:
        pass

    # update networktable
    try:
        while True:
            key, value = ntable_queue.get_nowait()
            if isinstance(value, numbers.Number):
                odom_table.putNumber(key, value)
            elif type(value) == bool:
                odom_table.putBoolean(key, value)
    except Empty:
        pass

    NetworkTables.flush()
    time.sleep(0.01)
