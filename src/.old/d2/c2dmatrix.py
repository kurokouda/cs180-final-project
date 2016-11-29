from collections.abc import Iterable
from copy import copy, deepcopy
import math

import numpy as np

from .vector2d import Vector2D


class C2DMatrix(object):
    def __init__(self):
        self.identity_ip()

    def identity_ip(self):
        self._matrix = np.matrix(np.identity(3))

    def __getitem__(self, key):
        key_x, key_y = key
        if key_x >= 0 and key_x < 3 and key_y >= 0 and key_y < 3:
            return self._matrix[key]
        else:
            raise KeyError('Key out of range')

    def _transform_vector_ip(self, vec):
        matrix = self._matrix
        temp_x = matrix[0, 0] * vec.x + matrix[1, 0] * vec.y + matrix[2, 0]
        temp_y = matrix[0, 1] * vec.x + matrix[1, 1] * vec.y + matrix[2, 1]
        vec.x = temp_x
        vec.y = temp_y

    def transform_vector_ip(self, vectors):
        if isinstance(vectors, Vector2D):
            self._transform_vector_ip(vectors)
        elif isinstance(vectors, Iterable):
            for vec in vectors:
                self._transform_vector_ip(vec)

    def transform_vector(self, vectors):
        new_vectors = deepcopy(vectors)
        self.transform_vector_ip(new_vectors)
        return new_vectors

    def translate_ip(self, pos_x, pos_y):
        mat = np.matrix(np.identity(3))
        mat[2, 0] = pos_x
        mat[2, 1] = pos_y
        self._matrix *= mat

    def translate(self, pos_x, pos_y):
        new_matrix = copy(self._matrix)
        new_matrix.translate_ip(pos_x, pos_y)
        return new_matrix

    def scale_ip(self, scale_x, scale_y):
        mat = np.matrix(np.identity(3))
        mat[0, 0] = scale_x
        mat[1, 1] = scale_y
        self._matrix *= mat

    def scale(self, scale_x, scale_y):
        new_matrix = copy(self._matrix)
        new_matrix.scale_ip(scale_x, scale_y)
        return new_matrix

    def _rotate_from_angle_ip(self, *args, **kwargs):
        angle_rad = kwargs.get('angle_rad') or args[0]

        mat = np.matrix(np.identity(3))
        sin = math.sin(angle_rad)
        cos = math.cos(angle_rad)

        mat[0, 0] = cos
        mat[0, 1] = sin
        mat[1, 0] = -sin
        mat[1, 1] = cos

        self._matrix *= mat

    def _rotate_from_vector_ip(self, *args, **kwargs):
        forward = kwargs.get('forward') or args[0]
        side = kwargs.get('side') or args[1]

        mat = np.matrix(np.identity(3))

        mat[0, 0] = forward.x
        mat[0, 1] = forward.y
        mat[1, 0] = side.x
        mat[1, 1] = side.y

        self._matrix *= mat

    def rotate_ip(self, *args, **kwargs):
        len_args = len(args)
        assert len_args >= 1 and len_args < 3
        if len_args == 1:
            self._rotate_from_angle_ip(*args, **kwargs)
        else:
            self._rotate_from_vector_ip(*args, **kwargs)

    def rotate(self, *args, **kwargs):
        new_matrix = deepcopy(self)
        new_matrix.rotate_ip(*args, **kwargs)
        return new_matrix

    def to_string(self):
        return str(self._matrix)

    def __str__(self):
        return self.to_string()

    def __repr__(self):
        return self.to_string()
