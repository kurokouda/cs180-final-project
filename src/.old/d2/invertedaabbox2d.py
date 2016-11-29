from pygame.rect import Rect

class InvertedAABBox2D(Rect):

    def __init__(self, top_left, bottom_right):
        super().__init__(top_left, bottom_right - top_left)
        # self._top_left = top_left
        # self._bottom_right = bottom_right
        # self._center = (top_left + bottom_right) / 2.0

    def is_overlapped_with(self, other):
        return self.colliderect(other)
        # return not (other.top > self.bottom or
        #         other.bottom < self.top or
        #         other.left > self.right or
        #         other.right < self.left)

    # @property
    # def top_left(self):
    #     return self._top_left
    #
    # @property
    # def bottom_right(self):
    #     return self._bottom_right
    #
    # @property
    # def top(self):
    #     return self._top_left.y
    #
    # @property
    # def left(self):
    #     return self._top_left.x
    #
    # @property
    # def bottom(self):
    #     return self._bottom_right.y
    #
    # @property
    # def right(self):
    #     return self._bottom_right.x
    #
    # @property
    # def center(self):
    #     return self._center

    def render(self, render_center=False):
        # Do 4 lines here for bounding box

        if render_center:
            # draw circle here
            pass
