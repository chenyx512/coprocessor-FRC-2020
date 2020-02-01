import logging
import logging.config
import multiprocessing as mp
import time
from datetime import datetime
from networktables import NetworkTables
from queue import Full, Empty
import yaml
import numpy as np

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

dtheta = dt = 0

# processes
def t265_update():
    try:
        xyz_rpy_queue.get_nowait()
        xyz_rpy = xyzrpy_value[0:6]
        for i, c in enumerate('xyzrpt'):
            odom_table.putNumber(f'pose_{c}', xyz_rpy[i])
        return True
    except Empty:
        return False

def cv_update():
    try:
        while True:
            name, frame = frame_queue.get_nowait()
            camera_server.put_frame(name, frame)
    except Empty:
        pass
    try:
        target_found, target_dis, target_relative_dir_right, \
                world_xyt = target_queue.get_nowait()
        # TODO update with t265 data to calibrate
        # TODO check if target is on opposite side (may not need to)
        odom_table.putBoolean('target_found', target_found)
        if target_found:
            global dtheta, dt
            dtheta = world_xyt[-1] - xyzrpy_value[-1]
            world_xvec = np.array([world_xyt[0], world_xyt[1]])
            camera_xvec = np.array(xyzrpy_value[0:2])
            dt = world_xvec - np.matmul(getRotationMatrix2d(dtheta), camera_xvec)
            print(dt)
            for i, c in enumerate('xyt'):
                odom_table.putNumber(f'world_{c}',
                                     world_xyt[i])
            odom_table.putNumber('target_dis', target_dis)
            odom_table.putNumber('target_relative_dir_right',
                                 target_relative_dir_right)
        return True
    except Empty:
        return False

t265_process_manager = ProcessManager(
    lambda: T265Process(xyz_rpy_queue, xyzrpy_value, encoder_v_queue),
    t265_update,
)
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
    odom_table.putNumber('slave_time', time.time())

    NetworkTables.flush()
    time.sleep(0.01)
