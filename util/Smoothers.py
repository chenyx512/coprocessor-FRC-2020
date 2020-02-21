import math as m
from collections import deque
import numpy as np

class Smoother:
    def __init__(self, max_size, smooth_func=np.mean):
        self.max_size = max_size
        self.value_queue = deque(maxlen=max_size)
        self.smooth_func = smooth_func

    def update(self, value):
        self.value_queue.append(value)
        return self.get()

    def get(self):
        return self.smooth_func(self.value_queue, axis=0)

    def clear(self):
        self.value_queue.clear()

    def __len__(self):
        return len(self.value_queue)


class AngleSmoother:
    def __init__(self, max_size, smooth_func=np.mean):
        self.c_smoother = Smoother(max_size, smooth_func)
        self.s_smoother = Smoother(max_size, smooth_func)

    def update(self, angle):
        c = self.c_smoother.update(m.cos(m.radians(angle)))
        s = self.s_smoother.update(m.sin(m.radians(angle)))
        return m.degrees(m.atan2(s, c))

    def get(self):
        return m.degrees(m.atan2(self.s_smoother.get(), self.c_smoother.get()))

    def clear(self):
        self.s_smoother.clear()
        self.c_smoother.clear()

    def __len__(self):
        return len(self.c_smoother)
