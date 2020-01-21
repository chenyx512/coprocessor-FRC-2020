import multiprocessing as mp
from networktables import networktables
from D435Process import D435Process

d435_process = D435Process()
mp.Process(target=d435_process.run).start()

# TODO handshaker
# TODO t265