from math import isclose
from types import SimpleNamespace
from collections.abc import Iterable, Hashable
from ..utils import cantor
from .vector2d import Vector2D

class Line2D(Iterable, Hashable):
    @classmethod
    def from_points(cls, pt_a, pt_b):
        return Line2D(*pt_a, *pt_b)

    def __init__(self, a, b, c=None, d=None):
        self._from_pt = None
        self._to_pt = None
        self.__sub_init(a, b, c, d)

    def __sub_init(self, a, b, c, d):
        try:
            self.from_pt = Vector2D(*a)
            self.to_pt = Vector2D(*b)
        except TypeError:
            try:
                self.from_pt = Vector2D(a, b)
                self.to_pt = Vector2D(c, d)
            except Exception as e:
                raise e

    def __get_from_pt(self):
        return self._from_pt

    def __set_from_pt(self, value):
        if self._from_pt is None:
            self._from_pt = Vector2D()
        self._from_pt.set(value)

    def __get_to_pt(self):
        return self._to_pt

    def __set_to_pt(self, value):
        if self._to_pt is None:
            self._to_pt = Vector2D()
        self._to_pt.set(value)

    from_pt = property(__get_from_pt, __set_from_pt)
    to_pt = property(__get_to_pt, __set_to_pt)

    def __iter__(self):
        yield self._from_pt
        yield self._to_pt

    def __hash__(self):
        return cantor(hash(self._from_pt), hash(self._to_pt))

    def __eq__(self, other):
        return self._from_pt == other.from_pt and self._to_pt == other.to_pt

    def __repr__(self):
        return '<Line2D ({}, {}), ({}, {})>'.format(
                *self._from_pt, *self._to_pt)
