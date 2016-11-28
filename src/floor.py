from collections import OrderedDict
from collections.abc import Sequence
from pprint import pprint
from random import choice as randchoice, random
from math import pi as PI

import pygame

from .config import Config
from .gameworld import GameWorld
from .d2.vector2d import Vector2D
from .utils import random_point_in_tri
from .entity import vehicle
from . import wall2d, cellspacepartition

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
        # assert key >= 0 and key < 3, "Index {} out of bounds".format(key)
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
            '_prob'
            )

    def __init__(self, walls=(), doors=(), triangles=(), prob=0.0, id_=0):
        self._walls = walls
        self._doors = doors
        self._triangles = triangles
        self._prob = prob
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

    @property
    def prob(self):
        return self._prob

    def __repr__(self):
        return '<Room:{}>'.format(self._id)

    def draw(self, surface):
        for tri in self._triangles:
            tri.draw(surface)
        for wall in self._walls:
            wall.draw(surface)
        for door in self._doors:
            door.draw(surface)


class Person(vehicle.Vehicle):
    def draw(self, surface):
        pygame.draw.circle(surface, (255, 0, 0), self._position, 2)

class CellSpacePartition(cellspacepartition.CellSpacePartition):
    def render(self, surface):
        x_step = self._space_width // self._num_cells_x
        y_step = self._space_height // self._num_cells_y

        # draw vertical lines
        for i in range(self._num_cells_x - 1):
            from_pt = ((i + 1) * x_step, 0)
            to_pt = ((i + 1) * x_step, self._space_height)
            pygame.draw.aaline(surface, pygame.Color('gray90'), from_pt, to_pt)

        for i in range(self._num_cells_y - 1):
            from_pt = (0, (i + 1) * y_step)
            to_pt = (self._space_width, (i + 1) * y_step)
            pygame.draw.aaline(surface, pygame.Color('gray90'), from_pt, to_pt)


class Floor(GameWorld):

    instances = {}

    def __init__(self, map_data):
        super().__init__(Config().window.WIDTH // 2,
                Config().window.HEIGHT // 2)

        Floor.instances[map_data['id']] = self
        self._id = map_data['id']
        self._num_people = map_data['num_people']
        self._cell_space = CellSpacePartition(
                width=self._window_width,
                height=self._window_height,
                cells_x=Config().NUM_CELLS_X,
                cells_y=Config().NUM_CELLS_Y,
                max_entities=self._num_people
                )
        if not self.is_render_neighbors_on():
            self.toggle_render_neighbors()

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
                    room_data['pr'],
                    instance_id
                    )

        self._generate_default_surface()

        while len(self._vehicles) < self._num_people:
            room = randchoice(tuple(self._rooms.values()))
            if not room.triangles:
                continue
            if random() > room.prob:
                continue
            tri = randchoice(room.triangles)
            spawn_pos = Vector2D.make_int(random_point_in_tri(*tri))

            person = Person(
                    world=self,
                    position=spawn_pos,
                    rotation=random() * 2 * PI,
                    velocity=Vector2D(),
                    mass=Config().VEHICLE_MASS,
                    max_force=Config().STEERING_FORCE,
                    max_speed=Config().MAX_SPEED,
                    max_turn_rate=Config().MAX_TURN_RATE_PER_SECOND,
                    scale=Config().VEHICLE_SCALE
                    )
            person.steering.wander_on()

            self._vehicles.append(person)


    def get_walls(self):
        return self._walls.values()

    walls = property(get_walls)

    def _generate_default_surface(self):
        self._default_surface = pygame.Surface((self._window_width,
                self._window_height))
        self._default_surface.convert()
        self._default_surface.fill(pygame.Color('white'))

        ## Uncomment to draw triangles
        # for tri in self._triangles.values():
        #     tri.draw(self._default_surface)

        if self.is_render_neighbors_on():
            self._cell_space.render(self._default_surface)

        for wall in self._walls.values():
            wall.draw(self._default_surface)

        for door in self._doors.values():
            door.draw(self._default_surface)

        ## Uncomment to draw rooms
        # COUNT = 0
        # for k, room in enumerate(self._rooms.values()):
        #     if k == COUNT:
        #         room.draw(self._default_surface)
        #         break

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

        for person in self._vehicles:
            person.draw(floor_surface)

        return floor_surface

    def update(self, time_elapsed):
        for person in self._vehicles:
            person.update(time_elapsed)
