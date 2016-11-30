from math import isclose
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


class LineSegment2D(Line2D):
    @staticmethod
    def distance_from_point(line, p):
        a = line.from_pt
        b = line.to_pt
        dot_a = ((p[0] - a[0]) * (b[0] - a[0]) +
                (p[1] - a[1]) * (b[1] - a[1]))
        if dot_a <= 0:
            return Vector2D.distance(a, p)

        dot_b = ((p[0] - b[0]) * (a[0] - b[0]) +
                (p[1] - b[1]) * (a[1] - b[1]))
        if dot_b <= 0:
            return Vector2D(*b).distance(p)

        pt = Vector2D(*b).sub(a)
        pt.mul(dot_a)
        pt.div(dot_a + dot_b)
        pt.add(a)

        return Vector2D(*p).distance(pt)

    @staticmethod
    def distance_from_point_sq(line, p):
        a = line.from_pt
        b = line.to_pt
        dot_a = ((p[0] - a[0]) * (b[0] - a[0]) +
                (p[1] - a[1]) * (b[1] - a[1]))
        if dot_a <= 0:
            return Vector2D(*a).distance_sq(p)

        dot_b = ((p[0] - b[0]) * (a[0] - b[0]) +
                (p[1] - b[1]) * (a[1] - b[1]))
        if dot_b <= 0:
            return Vector2D(*b).distance_sq(p)

        pt = Vector2D(*b).sub(a)
        pt.mul(dot_a)
        pt.div(dot_a + dot_b)
        pt.add(a)

        return Vector2D(*p).distance_sq(pt)


    @staticmethod
    def line_intersection(lhs, rhs, output=None):
        a, b = lhs
        c, d = rhs
        r_top = (a[1] - c[1]) * (d[0] - c[0]) - (a[0] - c[0]) * (d[1] - c[1])
        r_bot = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0])
        s_top = (a[1] - c[1]) * (b[0] - a[0]) - (a[0] - c[0]) * (b[1] - a[1])
        s_bot = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0])

        has_ip = False
        if not (isclose(r_bot, 0) or isclose(s_bot, 0)):
            r = r_top / r_bot
            s = s_top / s_bot

            if r > 0 and r < 1 and s > 0 and s < 1:
                if output is not None:
                    output['dist_to_ip'] = Vector2D.distance(a, b) * r
                    output['ip'] = Vector2D(*b).sub(a).mul(r).add(a)
                has_ip = True
            elif output is not None:
                output['dist_tp_ip'] = 0.0

        return has_ip

    def __repr__(self):
        return '<LineSegment2D ({}, {}), ({}, {})>'.format(
                *self._from_pt, *self._to_pt)


def main():
    l1 = LineSegment2D(1, 1, 4, 6)
    l2 = LineSegment2D((5, 7), (3, 1))
    pt = Vector2D(1, 5)
    print(l2)
    info = {}
    print(LineSegment2D.line_intersection(l1, l2, info), info)
    print(LineSegment2D.distance_from_point(l1, pt))

if __name__ == '__main__':
    main()
