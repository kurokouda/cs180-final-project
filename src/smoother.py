from collections import deque
from copy import copy

class Smoother(object):
    def __init__(self, sample_size, zero_value):
        self._history = deque(
                [copy(zero_value) for i in range(sample_size)],
                maxlen=sample_size)
        self._zero_value = copy(zero_value)

    def update(self, most_recent_value):
        self._history.append(most_recent_value)
        return sum(self._history) / len(self._history)
