from copy import copy
from uuid import uuid4

from .d2.vector2d import Vector2D


class Wall2D(object):
    __slots__ = (
        '_id',
        '_from_pt',
        '_to_pt',
        '_normal',
    )

    def __init__(self, from_pt=Vector2D(), to_pt=Vector2D(), normal=None,
            instance_id=None):
        self._id = instance_id or uuid4()
        self._from_pt = copy(from_pt)
        self._to_pt = copy(to_pt)
        self._normal = normal or Vector2D()

        if normal is None:
            self._calculate_normal()

    def _calculate_normal(self):
        temp = (self._to_pt - self._from_pt).normalize()
        self._normal.x = -temp.y
        self._normal.y = temp.x

    def get_instance_id(self):
        return self._id

    def get_from_pt(self):
        return self._from_pt

    def set_from_pt(self, value):
        self._from_pt = copy(value)

    def get_to_pt(self):
        return self._to_pt

    def set_to_pt(self, value):
        self._to_pt = copy(value)

    def get_normal(self):
        return self._normal

    def set_normal(self, value):
        self._normal = copy(value)

    def get_center(self):
        return (self._from_pt + self._to_pt) / 2

    instance_id = property(get_instance_id)
    from_pt = property(get_from_pt, set_from_pt)
    to_pt = property(get_to_pt, set_to_pt)
    normal = property(get_normal, set_normal)
    center = property(get_center)

    def __repr__(self):
        return '<Wall2D from:({}, {}) to:({}, {})>'.format(
                *self._from_pt, *self._to_pt)

    def render(self, render_normals):
        raise NotImplementedError


def __main__():
    from_pt = Vector2D(12, 34)
    to_pt = Vector2D(56, 89)
    wallA = Wall2D(from_pt, to_pt)
    print(wallA)
    print(round(wallA.normal))

if __name__ == '__main__':
    __main__()
