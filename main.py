import logging
import logging.config
import multiprocessing as mp
import time
from datetime import datetime
from networktables import NetworkTables
from queue import Full, Empty
import yaml

from ProcessManager import ProcessManager
from T265Process import T265Process
from CVProcess import CVProcess
from Constants import Constants

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
xyzrpy_value = mp.Array('f', 6)
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
is_t265_connected = False
last_t265_connect_time = time.time()
is_CV_connected = False
last_CV_connect_time = time.time()
slave_handshake_cnt = 0


# processes
def t265_update():
    try:
        xyz_rpy_queue.get_nowait()
        xyz_rpy = xyzrpy_value[0:6]
        for i, c in enumerate('xyzrpy'):
            odom_table.putNumber(f'pose_{c}', xyz_rpy[i])
        return True
    except Empty:
        return False

def cv_update():
    try:
        target_found, target_dis, target_relative_dir_right, \
        target_abs_azm, target_relative_xyzrpy = target_queue.get_nowait()
        odom_table.putBoolean('target_found', target_found)
        if target_found:
            for i, c in enumerate('xyzrpy'):
                odom_table.putNumber(f'target_relative_{c}',
                                     target_relative_xyzrpy[i])
            odom_table.putNumber('target_dis', target_dis)
            odom_table.putNumber('target_relative_dir_right',
                                 target_relative_dir_right)
            odom_table.putNumber('target_abs_azm', target_abs_azm)
        return True
    except Empty:
        return False

t265_process_manager = ProcessManager(
    lambda: T265Process(xyz_rpy_queue, xyzrpy_value, encoder_v_queue),
    t265_update,
)
cv_process_manager = ProcessManager(
    lambda: CVProcess(target_queue, xyzrpy_value),
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
