from random import random

def random_in_range(low, high):
    '''Return a random double from low to high'''
    return low + random() * (high - low)

def random_clamped():
    '''Return a random double from -1 to 1'''
    return random() - random()
