import logging
import logging.config
import multiprocessing as mp
import time
from datetime import datetime
from networktables import NetworkTables
from queue import Full, Empty
import yaml
import numpy as np
import math

from T265Process import T265Process
from CVProcess import CVProcess
from Constants import Constants
from ProcessManager import ProcessManager
from util.transformation import getRotationMatrix2d
from util.CameraServerWrapper import CameraServerWrapper


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
frame_queue = mp.Queue(1)
xyzrpy_value = mp.Array('f', 6)
target_queue = mp.Queue(1)


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

# transformation of coord
calibrated_dt = calibrated_dtheta = 0
field_xyt = [0.0, 0.0, 0.0]

# processes
def t265_update():
    try:
        xyz_rpy_queue.get_nowait()
        xyz_rpy = xyzrpy_value[0:6]
        for i, c in enumerate('xyzrpt'):
            odom_table.putNumber(f't265_pose_{c}', xyz_rpy[i])

        field_xvec = calibrated_dt + np.matmul(getRotationMatrix2d(calibrated_dtheta),
                                               [xyz_rpy[0], xyz_rpy[1]])
        global field_xyt
        field_xyt = field_xvec.tolist()
        field_xyt.append(xyz_rpy[-1] + calibrated_dtheta)
        for i, c in enumerate('xyt'):
            odom_table.putNumber(f'field_pose_{c}', field_xyt[i])
        odom_table.putNumber('target_field_azm',
                             math.degrees(math.atan2(field_xyt[1], field_xyt[0])) - 180)
        return True
    except Empty:
        return False
t265_process_manager = ProcessManager(
    lambda: T265Process(xyz_rpy_queue, xyzrpy_value, encoder_v_queue),
    t265_update,
)

calibration_cnt_left = 0
def cv_update():
    global calibration_cnt_left, dtheta_array, dt_array, calibrated_dt,\
        calibrated_dtheta
    try:
        while True:
            name, frame = frame_queue.get_nowait()
            camera_server.put_frame(name, frame)
    except Empty:
        pass
    try:
        target_found, target_dis, target_relative_dir_right, \
                target_t265_azm, camera_xyt = target_queue.get_nowait()
        # TODO check if target is on opposite side (may not need to)
        odom_table.putBoolean('target_found', target_found)
        if not target_found:
            odom_table.putBoolean('target_found', False)
            return True

        if odom_table.getBoolean('field_calibration_start', False):
            logging.info("start field calibration")
            calibration_cnt_left = 10
            odom_table.putBoolean('field_calibration_start', False)
            dtheta_array = []
            dt_array = []

        if calibration_cnt_left != 0:
            dtheta = camera_xyt[-1] - xyzrpy_value[-1]
            field_xvec = np.array([camera_xyt[0], camera_xyt[1]])
            camera_xvec = np.array(xyzrpy_value[0:2])
            dt = field_xvec - np.matmul(getRotationMatrix2d(dtheta), camera_xvec)
            dtheta_array.append(dtheta)
            dt_array.append(dt)
            calibration_cnt_left -= 1
            if calibration_cnt_left == 0:
                calibrated_dt = np.median(dt_array)
                calibrated_dtheta = np.median(dtheta_array)
                logging.info("end field calibration with"
                             f"dt {calibrated_dt}, dtheta {calibrated_dtheta}")
                odom_table.putBoolean('field_calibration_good', True)

        for i, c in enumerate('xyt'):
            odom_table.putNumber(f'field_{c}', camera_xyt[i])
        odom_table.putNumber('target_t265_azm', target_t265_azm)
        odom_table.putNumber('target_dis', target_dis)
        odom_table.putNumber('target_relative_dir_right',
                             target_relative_dir_right)

        if odom_table.getBoolean('field_calibration_good', False):
            error_xy = math.hypot(camera_xyt[0] - field_xyt[0],
                                  camera_xyt[1] - field_xyt[1])
            error_theta = camera_xyt[2] - field_xyt[2]
            odom_table.putNumber('error_xy', error_xy)
            odom_table.putNumber('error_theta', error_theta)

        return True
    except Empty:
        odom_table.putBoolean('target_found', False)
        return False

cv_process_manager = ProcessManager(
    lambda: CVProcess(target_queue, xyzrpy_value, frame_queue),
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

    NetworkTables.flush()
    time.sleep(0.01)
