import math

import numpy as np


class C2DMatrix(object):
    def __init__(self):
        self.identity()

    def identity(self):
        self.__matrix = np.matrix(np.identity(3))
        return self

    def __getitem__(self, key):
        key_x, key_y = key
        assert key_x >= 0 and key_x < 3 and key_y >= 0 and key_y < 3
        return self.__matrix[key]

    def ___transform_vector(self, vec):
        matrix = self.__matrix
        temp_x = matrix[0, 0] * vec.x + matrix[1, 0] * vec.y + matrix[2, 0]
        temp_y = matrix[0, 1] * vec.x + matrix[1, 1] * vec.y + matrix[2, 1]
        vec.x = temp_x
        vec.y = temp_y

    def transform_vector(self, vectors):
        try:
            for vec in vectors:
                self.___transform_vector(vec)
        except (TypeError, AttributeError):
            self.___transform_vector(vectors)

    def translate(self, pos):
        mat = np.matrix(np.identity(3))
        mat[2, 0], mat[2, 1] = pos
        self.__matrix *= mat
        return self

    def scale(self, scale):
        mat = np.matrix(np.identity(3))
        mat[0, 0], mat[1, 1] = scale
        self.__matrix *= mat
        return self

    def __rotate_from_angle(self, *args, **kwargs):
        angle_rad = kwargs.get('angle_rad') or args[0]

        mat = np.matrix(np.identity(3))
        sin = math.sin(angle_rad)
        cos = math.cos(angle_rad)

        mat[0, 0] = cos
        mat[0, 1] = sin
        mat[1, 0] = -sin
        mat[1, 1] = cos

        self.__matrix *= mat

    def __rotate_from_vector(self, *args, **kwargs):
        forward = kwargs.get('forward') or args[0]
        side = kwargs.get('side') or args[1]

        mat = np.matrix(np.identity(3))

        mat[0, 0] = forward[0]
        mat[0, 1] = forward[1]
        mat[1, 0] = side[0]
        mat[1, 1] = side[1]

        self.__matrix *= mat

    def rotate(self, *args, **kwargs):
        len_args = len(args)
        assert len_args >= 1 and len_args < 3
        if len_args == 1:
            self.__rotate_from_angle(*args, **kwargs)
        else:
            self.__rotate_from_vector(*args, **kwargs)
        return self

    def __repr__(self):
        return str(self.__matrix)
