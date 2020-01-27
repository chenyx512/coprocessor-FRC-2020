import logging
import logging.config
import multiprocessing as mp
import time
from datetime import datetime
from networktables import NetworkTables
from queue import Full, Empty
import yaml

from T265Process import T265Process
from CVProcess import CVProcess
from Constants import Constants


# logging setup
with open('logging_config.yaml', 'r') as yaml_file:
    logging_config = yaml.safe_load(yaml_file)
for _, handler in logging_config['handlers'].items():
    if 'filename' in handler:
        timestamp = datetime.now().strftime(r'%m%d_%H%M%S')
        handler['filename'] = str(f'log/{timestamp}.log')
logging.config.dictConfig(logging_config)


# shared
encoder_v_queue = mp.Queue(1)
xyz_rpy_queue = mp.Queue(10)
xyz_rpy_value = mp.Array('f', 6)
target_queue = mp.Queue(1)

def encoder_callback(entry, key, value, is_new):
    if key == '/odom/encoder_v':
        try:
            encoder_v_queue.put_nowait(value)
        except Full:
            logging.warning('encoder_v queue full')

# NTable
ip = 'roborio-3566-frc.local'
logging.info(f'starting NTable with ip [{ip}]')
NetworkTables.initialize(ip if ip != 'host' else None)
odom_table = NetworkTables.getTable('odom')
NetworkTables.getEntry('/odom/encoder_v').addListener(
    encoder_callback,
    NetworkTables.NotifyFlags.UPDATE
)

# status flag
is_master_connected = False
last_master_connect_time = 0.0
is_t265_connected = False
last_t265_connect_time = 0.0
is_CV_connected = False
last_CV_connect_time = 0.0
slave_handshake_cnt = 0

# processes
t265_process = T265Process(xyz_rpy_queue, xyz_rpy_value, encoder_v_queue)
t265_process.start()
# mp.Process(target=t265_process.start).start()
# cv_process = CVProcess(target_queue, xyz_rpy_value)
# mp.Process(target=cv_process.run).start()

while True:
    # get pose
    try:
        xyz_rpy = xyz_rpy_queue.get_nowait()
        try: # empty the queue to retrieve the latest value
            while True:
                xyz_rpy = xyz_rpy_queue.get_nowait()
        except Empty:
            pass
        for i, c in enumerate('xyzrpy'):
            odom_table.putNumber(f'pose_{c}', xyz_rpy[i])
        last_t265_connect_time = time.time()
        is_t265_connected = True
    except Empty:
        if time.time() - last_t265_connect_time > Constants.DISCONNECT_TIME:
            is_t265_connected = False
    odom_table.putBoolean('pose_good', is_t265_connected)

    # get CV
    try:
        target_dis, target_relative_dir_right, target_abs_azm, target_relative_xyzrpy = \
            target_queue.get_nowait()
        for i, c in enumerate('xyzrpy'):
            odom_table.putNumber(f'target_relative_{c}', target_relative_xyzrpy[i])
        odom_table.putNumber(f'target_relative_dir_right', target_relative_dir_right)
        odom_table.putNumber(f'target_abs_azm', target_abs_azm)
        last_CV_connect_time = time.time()
        is_CV_connected = True
    except Empty:
        if time.time() - last_CV_connect_time > Constants.DISCONNECT_TIME:
            is_CV_connected = False
    odom_table.putBoolean('target_good', is_CV_connected)

    # handshake
    odom_table.putNumber('slave_time', time.time())

    NetworkTables.flush()
    time.sleep(0.01)
