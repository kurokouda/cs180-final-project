from random import random


def __cantor_biject(k):
    return k * 2 if k >= 0 else -k * 2 - 1

def cantor(a, b):
    k1 = __cantor_biject(a)
    k2 = __cantor_biject(b)
    return round(0.5 * (k1 + k2) * (k1 + k2 + 1) + k2)

def random_in_range(a, b):
    return a + random() * (b - a)
