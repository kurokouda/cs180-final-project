from collections import OrderedDict
from collections.abc import Sequence
from pprint import pprint

import pygame

from .config import Config
from .gameworld import GameWorld
from .cellspacepartition import CellSpacePartition
from .d2.vector2d import Vector2D

from . import wall2d

class Wall2D(wall2d.Wall2D, object):
    __slots__ = wall2d.Wall2D.__slots__
    def draw(self, surface):
        pygame.draw.line(surface, pygame.Color('black'), self._from_pt,
                self._to_pt, 2)

class Door2D(wall2d.Wall2D, object):
    __slots__ = wall2d.Wall2D.__slots__
    DOOR_COLOR = pygame.Color('orange')

    @property
    def center(self):
        return (self._from_pt + self._to_pt) // 2

    def draw(self, surface):
        center = (int(self.center.x), int(self.center.y))
        pygame.draw.circle(surface, type(self).DOOR_COLOR, center, 2)

class ExitDoor2D(Door2D, object):
    __slots__ = Door2D.__slots__
    DOOR_COLOR = pygame.Color('darkgreen')

class Triangle2D(Sequence):
    __slots__ = ('_points', '_id')
    def __init__(self, a, b, c, id_):
        self._id = id_
        self._points = (Vector2D.make_int(a), Vector2D.make_int(b),
                Vector2D.make_int(c))

    def __getitem__(self, key):
        assert isinstance(key, int), "Key should be of type 'int'"
        assert key >= 0 and key < 3, "Index out of bounds"
        return self._points[key]

    def __len__(self):
        return 3

    def __repr__(self):
        return '<Tringle [{}]>'.format(', '.join(
                [str(tuple(v)) for v in self._points]))

    @property
    def instance_id(self):
        return self._id

    def draw(self, surface):
        pygame.draw.aalines(surface, pygame.Color(200, 200, 200), True,
            self._points)


class Room(object):
    __slots__ = (
            '_id',
            '_walls',
            '_doors',
            '_triangles',
            )

    def __init__(self, walls=(), doors=(), triangles=(), id_=0):
        self._walls = walls
        self._doors = doors
        self._triangles = triangles
        self._id = id_

    @property
    def walls(self):
        return self._walls

    @property
    def doors(self):
        return self._doors

    @property
    def triangles(self):
        return self._triangles

    @property
    def instance_id(self):
        return self._id

    @property
    def name(self):
        return self._id

    def __repr__(self):
        return '<Room:{}>'.format(self._id)

    def draw(self, surface):
        for tri in self._triangles:
            tri.draw(surface)
        for wall in self._walls:
            wall.draw(surface)
        for door in self._doors:
            door.draw(surface)


class Floor(GameWorld):

    instances = {}

    def __init__(self, map_data):
        Floor.instances[map_data['id']] = self
        self._id = map_data['id']
        self._num_people = map_data['num_people']
        self._cell_space = None
        super().__init__(Config().window.WIDTH // 2,
                Config().window.HEIGHT // 2)

        point_scale = self._window_width / map_data['dim']['width']

        points = OrderedDict()
        for point_data in map_data['points']:
            x = point_data['x'] * point_scale
            y = point_data['y'] * point_scale
            points[point_data['id']] = Vector2D(x, y)

        self._walls = OrderedDict()
        for wall_data in map_data['walls']:
            from_pt = points[wall_data['from']]
            to_pt = points[wall_data['to']]
            instance_id = wall_data['id']
            self._walls[instance_id] = (
                    Wall2D(from_pt, to_pt, instance_id=instance_id))

        self._doors = OrderedDict()
        for door_data in map_data['doors']:
            from_pt = points[door_data['from']]
            to_pt = points[door_data['to']]
            instance_id = door_data['id']
            if instance_id in map_data['safe_doors']:
                self._doors[instance_id] = ExitDoor2D(from_pt, to_pt,
                    instance_id=instance_id)
            else:
                self._doors[instance_id] = Door2D(from_pt, to_pt,
                    instance_id=instance_id)

        self._triangles = OrderedDict()
        for tri_data in map_data['triangles']:
            instance_id = tri_data['id']
            tri_points = tuple(points[i] for i in tri_data['points'])
            self._triangles[instance_id] = Triangle2D(*tri_points, instance_id)

        self._rooms = OrderedDict()
        for room_data in map_data['rooms']:
            instance_id = room_data['id']
            self._rooms[instance_id] = Room(
                    tuple(self._walls[k] for k in room_data['wall_ids']),
                    tuple(self._doors[k] for k in room_data['door_ids']),
                    tuple(self._triangles[k]
                        for k in room_data['triangle_ids']),
                    instance_id
                    )


        self._default_surface = pygame.Surface((self._window_width,
                self._window_height))
        self._default_surface.convert()
        self._default_surface.fill(pygame.Color('white'))

        for wall in self._walls.values():
            wall.draw(self._default_surface)

        for door in self._doors.values():
            door.draw(self._default_surface)


    def get_walls(self):
        return self._walls.values()

    walls = property(get_walls)

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
