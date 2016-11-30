from pygame.rect import Rect

class InvertedAABBox2D(Rect):
    def __init__(self, top_left, bottom_right):
        super().__init__(top_left, bottom_right - top_left)

    def is_overlapped_with(self, other):
        return self.colliderect(other)
