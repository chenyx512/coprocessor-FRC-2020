import math as m
from collections import deque
from statistics import median

class MedianSmoother:
    def __init__(self, max_size):
        self.max_size = max_size
        self.value_queue = deque(maxlen=max_size)

    def update(self, value):
        self.value_queue.append(value)
        return median(self.value_queue)

    def clear(self):
        self.value_queue.clear()

class MedianAngleSmoother:
    def __init__(self, max_size):
        self.max_size = max_size
        self.value_c_queue = deque(maxlen=max_size)
        self.value_s_queue = deque(maxlen=max_size)

    def update(self, angle):
        self.value_c_queue.append(m.cos(m.radians(angle)))
        self.value_s_queue.append(m.sin(m.radians(angle)))
        return m.atan2(median(self.value_s_queue), median(self.value_c_queue))

    def clear(self):
        self.value_s_queue.clear()
        self.value_c_queue.clear()
