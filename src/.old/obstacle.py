'''Simple obstacle class
'''

from .entity.basegameentity import BaseGameEntity
from .entity.entitytype import EntityType

class Obstacle(BaseGameEntity):

    def __init__(self, position, bounding_radius):
        super().__init__(EntityType.OBSTACLE, position, bounding_radius)

    def update(self, time_elapsed):
        return NotImplemented

    def render(self, pr):
        return NotImplemented

    def __repr__(self):
        return '<Obstacle pos:({}, {}) rad:>'.format(*self._position,
                self.bounding_radius)
