from collections.abc import Iterable
from itertools import count, cycle
from copy import deepcopy
import math

from src.common.utils import random_in_range
from src.common.math2d.transformation import vector_rotate_around_origin
from src.common.math2d.vector2d import Vector2D


_ROUNDING = True


class Path(Iterable):
    __slots__ = (
        '_looped',
        '_waypoints',
        '_current_waypoint_it',
        '_current_index',
    )

    def __init__(self, num_waypoints=None, min_x=None, min_y=None, max_x=None,
            max_y=None, looped=None):

        self._looped = False
        self._waypoints = []
        self._current_waypoint_it = None
        self._current_index = -1

        if (num_waypoints is not None and
                min_x is not None and
                min_y is not None and
                max_x is not None and
                max_y is not None and
                looped is not None):
            self._looped = looped
            self.create_random_path(
                num_waypoints,
                min_x,
                min_y,
                max_x,
                max_y)

    def _initialize_waypoint_it(self):
        self._current_waypoint_it = count()
        self._current_index = -1

    def _goto_next_waypoint(self):
        self._current_index = next(self._current_waypoint_it)

    def get_current_waypoint(self):
        if self._current_waypoint_it is None:
            raise ValueError("{}._current_waypoint is None"
                    .format(self.__class__.__name__))
        return self._waypoints[self._current_index]

    def get_path(self):
        return deepcopy(self._waypoints)

    def set_path(self, new_path):
        try:
            self.set_path(new_path.path)
        except AttributeError:
            self._waypoints = deepcopy(new_path)
            self._initialize_waypoint_it()
            self._goto_next_waypoint()


    path = property(get_path, set_path)
    current_waypoint = property(get_current_waypoint)

    def is_finished(self):
        return len(self._waypoints) - 1 == self._current_index

    def set_next_waypoint(self):
        assert len(self._waypoints) > 0, "No waypoints set"

        if self.is_finished():
            if self._looped:
                self._initialize_waypoint_it()

        if not self.is_finished():
            self._goto_next_waypoint()

    def create_random_path(self, num_waypoints, min_x, min_y, max_x, max_y):
        self._waypoints.clear()

        mid_x = (max_x + min_x) / 2.0
        mid_y = (max_y + min_y) / 2.0
        smaller = min(mid_x, mid_y)
        spacing = math.pi / num_waypoints

        for i in range(num_waypoints):
            radial_dist = random_in_range(smaller * 0.2, smaller)
            temp = Vector2D(radial_dist, 0.0)
            vector_rotate_around_origin(temp, i * spacing)
            temp.x += mid_x
            temp.y += mid_y
            if _ROUNDING:
                temp = Vector2D.round(temp)
            self._waypoints.append(temp)

        self._initialize_waypoint_it()
        if not self.is_finished():
            self._goto_next_waypoint()

        return self._waypoints

    def loop_on(self):
        self._looped = True

    def loop_off(self):
        self._looped = False

    def clear(self):
        self._waypoints.clear()

    def render(self):
        raise NotImplementedError

    def __iter__(self):
        if self._looped:
            it = cycle(self._waypoints)
        else:
            it = iter(self._waypoints)
        return it


    def to_string(self):
        return '<Path [{}]>'.format(',\n    '.join(map(str, self._waypoints)))

    def __str__(self):
        return self.to_string()

    def __repr__(self):
        return self.to_string()

def _main():
    path = Path(4, 0, 0, 30, 30, False)
    print(path)
    print()

    for i in range(20):
        waypoint = path.current_waypoint
        print(waypoint)
        path.set_next_waypoint()
    print()

    for wp in path:
        print(wp)

if __name__ == '__main__':
    _main()
