from src.d2.vector2d import Vector2D


def main():
    v1 = Vector2D(12, 23)
    v2 = Vector2D(22.0/7.0, -17.0/11.0)
    v3 = Vector2D()

    print('v1:', v1)
    print('v2:', v2)
    print('v3:', v3)
    print()


    print('v3.set(v1)')
    v3.set(v1)
    print('v1:', v1)
    print('v3:', v3)
    print()

    print('v3.zero()')
    v3.zero()
    print('v3:', v3)
    print()

    print('v2.length()')
    print(v2.length())
    print()

    print('v2.length_sq()')
    print(v2.length_sq())
    print()

    print('v2.normalize()')
    print(v2.normalize())
    print(v2)
    print()

    print('v1.dot(v2)')
    print(v1.dot(v2))
    print()

    print('v1 * v2')
    print(v1 * v2)
    print()

    print('3.14 * v2')
    print(3.14 * v2)
    print()

    print('v2 * 3.14')
    print(v2 * 3.14)
    print()

    print('v2.sign(v1)')
    print(v2.sign(v1))
    print()

    print('v2.perpendicular()')
    print(v2.perpendicular())
    print()

    print('v2.truncate(2)')
    print(v2)
    print(v2.truncate(2))
    print()

    print('v2.distance(v1)')
    print(v2.distance(v1))
    print()

    v_given = Vector2D(-2, -1)
    v_norm = Vector2D(0, -1)
    print('v_given.reflect(v_norm)')
    print(v_given.reflect(v_norm))
    print()

    print(*v1)

    # print(v2[0], v2[1])
    # print(v2['x'], v2['y'])
    # print(v2.x, v2.y)
    # v2.x = -v2.x * 2
    # print(v2.x, v2.y)
    # print(v2 - v1)
    # v2 -= v1
    # print(v2)
    # print(v2 / 10)
    # print(v2 // 10)
    # print(-v2)
    # print(round(v2))
    # print(v2 < v1)
    # print(v2 == v1)
    # print(hash(v2))
    # print(hash(v2) == hash(v2))
    # print()

if __name__ == '__main__':
    main()
