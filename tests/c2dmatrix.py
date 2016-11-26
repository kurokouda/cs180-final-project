#pylint: disable-all
import math

from src.c2dmatrix import C2DMatrix
from src.d2.vector2d import Vector2D

def _main():
    v1 = Vector2D(12, 2)
    mtrans = C2DMatrix()

    print(v1)
    mtrans.rotate_ip(60 * math.pi / 180)
    mtrans.transform_vector_ip(v1)
    print(v1)

if __name__ == '__main__':
    _main()
