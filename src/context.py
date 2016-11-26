from .singleton import SingletonABCMeta


class Context(object, metaclass=SingletonABCMeta):
    def __init__(self):
        self._clock = None

    def get_clock(self):
        return self._clock

    def set_clock(self, clock):
        self._clock = clock

    clock = property(get_clock, set_clock)
