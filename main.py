import multiprocessing as mp
import time
from networktables import NetworkTables
from D435Process import D435Process
from T265Process import T265Process

d435_process = D435Process()
t265_process = T265Process()
d435_process.run()

# mp.Process(target=t265_process.run).start()
# time.sleep(2)
# mp.Process(target=d435_process.run).start()


# TODO handshaker
# TODO t265