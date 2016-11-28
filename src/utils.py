from random import random

def random_in_range(low, high):
    '''Return a random double from low to high'''
    return low + random() * (high - low)

def random_clamped():
    '''Return a random double from -1 to 1'''
    return random() - random()

def random_point_in_tri(pt_a, pt_b, pt_c):
    r1 = random()
    r2 = random()
    return (((1 - r1**0.5) * pt_a) + ((r1**0.5 * (1 - r2)) * pt_b) +
            ((r1**0.5 * r2) * pt_c))
