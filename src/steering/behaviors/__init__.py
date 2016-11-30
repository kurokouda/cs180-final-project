from abc import ABCMeta, abstractmethod


class SteeringBehaviors(object):
    class Type(metaclass=ABCMeta):
        def __init__(self, weight, container):
            self._container = container
            self._weight = weight
            self._is_on = False

        @abstractmethod
        def __call__(self, *args, **kwargs):
            pass

        def is_on(self):
            return self._is_on

        def on(self):
            self._is_on = True

        def off(self):
            self._is_on = False
