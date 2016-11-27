from collections.abc import Iterable, Hashable, Container, Sized
from enum import IntEnum, unique
from numbers import Real
import sys
import math


@unique
class Rotation(IntEnum):
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


class Vector2D(Sized, Iterable, Hashable, Container):
    __slots__ = ('_x', '_y')

    @classmethod
    def from_vec(cls, vec):
        return cls(vec.x, vec.y)

    @staticmethod
    def wrap_around_ip(vec, max_x, max_y):
        if vec.x > max_x:
            vec.x = 0
        if vec.x < 0:
            vec.x = max_x
        if vec.y > max_y:
            vec.y = 0
        if vec.y < 0:
            vec.y = max_y

    @staticmethod
    def wrap_around(vec, max_x, max_y):
        new_vec = Vector2D.from_vec(vec)
        Vector2D.wrap_around_ip(new_vec, max_x, max_y)
        return new_vec

    @staticmethod
    def make_int(vec):
        new_vec = Vector2D.from_vec(vec)
        new_vec.x = int(vec.x)
        new_vec.y = int(vec.y)
        return new_vec

    @staticmethod
    def inside_region_vec(vec, vec_top_left, vec_bottom_right):
        return (vec.x >= vec_top_left.x and vec.y >= vec_top_left.y and
                vec.x <= vec_bottom_right.x and vec.x <= vec_bottom_right.y)

    @staticmethod
    def inside_region_pt(vec, top, left, bottom, right):
        return (vec.x >= left and vec.y >= top and
                vec.x <= right and vec.x <= bottom)

    @staticmethod
    def in_FOV(source, direction, target, fov):
        to_target = (target - source).normalize()
        return direction * to_target >= math.cos(fov / 2.0)

    def __init__(self, x=0, y=0):
        self._x = x
        self._y = y

    def get_x(self):
        return self._x

    def set_x(self, value):
        self._x = value

    def get_y(self):
        return self._y

    def set_y(self, value):
        self._y = value

    x = property(get_x, set_x)
    y = property(get_y, set_y)

    def set(self, vec):
        self._x = vec.x
        self._y = vec.y

    def zero(self):
        '''Sets x and y to zero
        '''
        self._x = 0
        self._y = 0

    def is_zero(self):
        '''Returns true if both x and y are zero
        '''
        return self._x**2 + self._y**2 < sys.float_info.min

    def length(self):
        '''Returns the length of a 2D vector
        '''
        return (self._x**2 + self._y**2)**0.5

    def length_sq(self):
        '''Returns the squared length of the vector (thereby avoiding the sqrt)
        '''
        return self._x**2 + self._y**2

    def normalize_ip(self):
        '''Normalizes a 2D vector in place
        '''
        vector_length = self.length()

        if vector_length > sys.float_info.min:
            self._x /= vector_length
            self._y /= vector_length

    def normalize(self):
        '''Returns the normalized 2D vector for this vector
        '''
        new_vector = Vector2D.from_vec(self)
        new_vector.normalize_ip()
        return new_vector

    def dot(self, other):
        '''Calculates the dot product
        '''
        return self._x * other.x + self._y * other.y

    def sign(self, other):
        '''Returns positive if <other> is clockwise of this vector,
        negative if anticlockwise (assuming the Y axis is pointing down,
        X axis to right like a pygame app)
        '''
        return (Rotation.COUNTERCLOCKWISE
                if self._y * other.x > self._x * other.y
                else Rotation.CLOCKWISE)

    def perpendicular(self):
        '''Returns the vector that is perpendicular to this one
        '''
        return Vector2D(-self._y, self._x)

    def truncate_ip(self, max_length):
        '''Truncates the vector in-place
        '''
        if self.length() > max_length:
            self.normalize_ip()
            self *= max_length

    def truncate(self, max_length):
        '''Adjusts x and y so that the length of the vector does not exceed max
        Truncates a vector so that its length does not exceed max
        '''
        new_vector = Vector2D.from_vec(self)
        new_vector.truncate_ip(max_length)
        return new_vector

    def distance(self, other):
        '''Calculates the euclidean distance between two vectors
        '''
        return ((self._x - other.x)**2 + (self._y - other.y)**2)**0.5

    def distance_sq(self, other):
        '''Squared version of distance.
        Calculates the euclidean distance squared between two vectors
        '''
        x_dist = self._x - other.x
        y_dist = self._y - other.y
        return x_dist**2 + y_dist**2

    def get_reverse(self):
        '''The vector that is the reverse of this vector
        '''
        return Vector2D.from_vec(-self)

    def reflect_ip(self, norm):
        '''In-place reflection
        '''
        self += 2 * (self * norm) * norm.get_reverse()

    def reflect(self, norm):
        '''Given a normalized vector this method reflects the vector it
        is operating upon. (like the path of a ball bouncing off a wall)
        '''
        new_vector = Vector2D.from_vec(self)
        new_vector.reflect_ip(norm)
        return new_vector

    def __getitem__(self, key):
        if key == 'x' or key == 0:
            return self._x
        elif key == 'y' or key == 1:
            return self._y
        else:
            raise KeyError('Invalid key \'{}\' supplied'.format(key))

    def __setitem__(self, key, value):
        if not isinstance(value, Real):
            raise ValueError('Invalid value of type {}'.format(type(value)))
        if key == 'x' or key == 0:
            self._x = value
        elif key == 'y' or key == 1:
            self._y = value
        else:
            raise KeyError('Invalid key \'{}\' supplied'.format(key))

    def __add__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        return Vector2D(self._x + other.x, self._y + other.y)

    def __iadd__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        self._x += other.x
        self._y += other.y

    def __sub__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        return Vector2D(self._x - other.x, self._y - other.y)

    def __isub__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        self._x -= other.x
        self._y -= other.y

    def __mul__(self, other):
        if isinstance(other, Real):
            return Vector2D(self._x * other, self._y * other)
        elif isinstance(other, Vector2D):
            return self._x * other.x + self._y * other.y
        else:
            return NotImplemented

    def __imul__(self, other):
        if isinstance(other, Real):
            self._x *= other
            self._y *= other
        else:
            return NotImplemented

    def __rmul__(self, other):
        if isinstance(other, Real):
            return Vector2D(self._x * other, self._y * other)
        else:
            return NotImplemented

    def __truediv__(self, other):
        if isinstance(other, Real):
            return Vector2D(self._x / other, self._y / other)
        else:
            return NotImplemented

    def __itruediv__(self, other):
        if isinstance(other, Real):
            self._x /= other
            self._y /= other
        else:
            return NotImplemented

    def __floordiv__(self, other):
        if isinstance(other, Real):
            return Vector2D(self._x // other, self._y // other)
        else:
            return NotImplemented

    def __ifloordiv__(self, other):
        if isinstance(other, Real):
            self._x //= other
            self._y //= other
        else:
            return NotImplemented

    def __neg__(self):
        return Vector2D(-self._x, -self._y)

    def __abs__(self):
        return self.length()

    def __round__(self, n=0):
        return Vector2D(round(self._x, n), round(self._y, n))

    def __lt__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        return self.length_sq() < other.length_sq()

    def __ge__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        return self.length_sq() >= other.length_sq()

    def __eq__(self, other):
        if not isinstance(other, Vector2D):
            return NotImplemented
        return self._x == other.x and self._y == other.y

    def __hash__(self):
        return hash((self._x, self._y)) * 123

    def __iter__(self):
        yield self._x
        yield self._y

    def __contains__(self, value):
        return value == self._x or value == self._y

    def __len__(self):
        return 2

    def __str__(self):
        return self.to_string()

    def __repr__(self):
        return self.to_string()

    def to_string(self):
        return '<Vector2D ({}, {})>'.format(self._x, self._y)
