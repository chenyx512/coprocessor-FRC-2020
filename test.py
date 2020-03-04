import multiprocessing as mp
import logging

from T265Process import T265Process
from D435Process import D435Process
from CVProcess import CVProcess

encoder_v_queue = mp.Queue(1)
xyz_rpy_queue = mp.Queue(1)
frame_queue = mp.Queue(10)
ntable_queue = mp.Queue(100)
xyzrpy_value = mp.Array('f', 6)
target_queue = mp.Queue(1)
ball_queue = mp.Queue(1)

# process = D435Process(frame_queue, ball_queue, xyzrpy_value)
process = CVProcess(target_queue, xyzrpy_value, frame_queue, ntable_queue)
process.logger.setLevel(50)
process.daemon = True
# process = T265Process(frame_queue, frame_queue, frame_queue)
process.run()