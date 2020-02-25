import multiprocessing as mp

from T265Process import T265Process
from D435Process import D435Process


frame_queue = mp.Queue(10)
ball_queue = mp.Queue(1)
xyzrpy_value = mp.Array('f', 6)

process = D435Process(frame_queue, ball_queue, xyzrpy_value)
process.daemon = True
# process = T265Process(frame_queue, frame_queue, frame_queue)
process.run()