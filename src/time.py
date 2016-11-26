import time

from .singleton import SingletonABCMeta

class CrudeTimer(object, metaclass=SingletonABCMeta):
    Clock = CrudeTimer()

    def __init__(self):
        self._start_time = time.time()

    def get_current_time(self):
        return time.time() - self._start_time

