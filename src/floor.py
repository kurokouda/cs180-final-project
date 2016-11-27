from pprint import pprint

import pygame

from .config import Config
from .gameworld import GameWorld
from .cellspacepartition import CellSpacePartition
from .d2.vector2d import Vector2D

from . import wall2d

class Wall2D(wall2d.Wall2D):
    __slots__ = wall2d.Wall2D.__slots__
    def draw(self, surface):
        pygame.draw.line(surface, pygame.Color('black'), self._from_pt,
                self._to_pt, 2)


class Floor(GameWorld):

    def __init__(self, map_data):
        self._num_people = map_data['num_people']
        self._cell_space = None
        super().__init__(Config().window.WIDTH // 2,
                Config().window.HEIGHT // 2)

        point_scale = self._window_width / map_data['dim']['width']

        points = dict()
        for point_data in map_data['points']:
            x = point_data['x'] * point_scale
            y = point_data['y'] * point_scale
            points[point_data['id']] = Vector2D(x, y)

        for wall_data in map_data['walls']:
            from_pt = points[wall_data['from']]
            to_pt = points[wall_data['to']]
            instance_id = wall_data['id']
            self._walls.append(Wall2D(from_pt, to_pt, instance_id=instance_id))

        self._default_surface = pygame.Surface((self._window_width,
                self._window_height))
        self._default_surface.convert()
        self._default_surface.fill(pygame.Color('white'))
        for wall in self._walls:
            wall.draw(self._default_surface)

        pprint(self._walls)

    def _set_cell_space(self):
        self._cell_space = CellSpacePartition(
                width=self._window_width,
                height=self._window_height,
                cells_x=Config().NUM_CELLS_X,
                cells_y=Config().NUM_CELLS_Y,
                max_entities=self._num_people
                )

    def render(self):
        floor_surface = pygame.Surface.copy(self._default_surface)

        return floor_surface
