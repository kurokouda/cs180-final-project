from copy import copy
from random import random
from math import pi as PI

from .config import Config
from .d2.vector2d import Vector2D
from .cellspacepartition import CellSpacePartition
from .path import Path
from .entity.entityfunctiontemplates import (
    enforce_non_penetration_constraint,
    tag_neighbors,
)
from .utils import random_clamped
from .entity.vehicle import Vehicle
from .smoother import Smoother


class GameWorld(object):
    __slots__ = (
        '_vehicles',
        '_obstacles',
        '_walls',
        '_cell_space',
        '_path',
        '_paused',
        '_window_height',
        '_window_width',
        '_crosshair',
        '_average_frame_time',
        '_show_walls',
        '_show_obstacles',
        '_show_path',
        '_show_detection_box',
        '_show_wander_circle',
        '_show_feelers',
        '_show_steering_force',
        '_show_fps',
        '_render_neighbors',
        '_view_keys',
        '_show_cell_space_info',
    )

    SAMPLE_RATE = 10
    _frame_rate_smoother = Smoother(SAMPLE_RATE, 0.0)

    def __init__(self, window_width, window_height):
        self._vehicles = []
        self._obstacles = []
        self._walls = []
        self._window_width = window_width
        self._window_height = window_height
        self._paused = False
        self._crosshair = Vector2D(window_width // 2, window_height // 2)
        self._show_walls = False
        self._show_obstacles = False
        self._show_path = False
        self._show_wander_circle = False
        self._show_steering_force = False
        self._show_feelers = False
        self._show_detection_box = False
        self._show_fps = True
        self._average_frame_time = 0
        self._path = None
        self._render_neighbors = False
        self._view_keys = False
        self._show_cell_space_info = False

        # NOTE: Code below won't apply for project implementation. Encapsulate
        # in a function for overriding.
        self._cell_space = CellSpacePartition(
                width=window_width,
                height=window_height,
                cells_x=Config().NUM_CELLS_X,
                cells_y=Config().NUM_CELLS_Y,
                max_entities=Config().NUM_AGENTS
                )

        border = 30
        self._path = Path(5, border, border, window_width - border,
                window_height - border, True)

        for _ in range(Config().NUM_AGENTS):
            x = window_width // 2 + random_clamped() * window_width // 2
            y = window_height // 2 + random_clamped() * window_height // 2
            spawn_pos = Vector2D(x, y)

            vehicle = Vehicle(
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

            vehicle.steering.flocking_on()
            self._vehicles.append(vehicle)
            self._cell_space.add_entity(vehicle)

        SHOAL = False
        if SHOAL:
            LAST = Config().NUM_AGENTS - 1
            self._vehicles[LAST].steering.flocking_off()
            self._vehicles[LAST].scale = Vector2D(10, 10)
            self._vehicles[LAST].steering.wander_on()
            self._vehicles[LAST].max_speed = 70

            for i in range(LAST):
                self._vehicles[i].steering.evade_on(self._vehicles[LAST])


    def non_penetration_constraint(self, vehicle):
        enforce_non_penetration_constraint(vehicle, self._vehicles)

    def tag_vehicles_within_view_range(self, vehicle, range_):
        tag_neighbors(vehicle, self._vehicles, range_)

    def tag_obstacles_within_range(self, vehicle, range_):
        tag_neighbors(vehicle, self._obstacles, range_)

    def get_walls(self):
        return self._walls

    def get_cell_space(self):
        return self._cell_space

    def get_obstacles(self):
        return self._obstacles

    def get_agents(self):
        return self._vehicles

    def get_crosshair(self):
        return self._crosshair

    def set_crosshair(self, value):
        self._crosshair = copy(value)

    def get_window_width(self):
        return self._window_width

    def get_window_height(self):
        return self._window_height

    wall = property(get_walls)
    cell_space = property(get_cell_space)
    obstacles = property(get_obstacles)
    agents = property(get_agents)
    crosshair = property(get_crosshair, set_crosshair)
    window_width = property(get_window_width)
    window_height = property(get_window_height)

    def toggle_pause(self):
        self._paused = not self._paused

    def toggle_show_fps(self):
        self._show_fps = not self._show_fps

    def is_paused(self):
        return self._paused

    def is_render_wall_on(self):
        return self._show_walls

    def is_render_obstacles_on(self):
        return self._show_obstacles

    def is_render_path_on(self):
        return self._show_path

    def is_render_detection_box_on(self):
        return self._show_detection_box

    def is_render_wander_circle_on(self):
        return self._show_wander_circle

    def is_render_feelers_on(self):
        return self._show_feelers

    def is_render_steering_force_on(self):
        return self._show_steering_force

    def is_render_fps_on(self):
        return self._show_fps

    def is_view_keys_on(self):
        return self._view_keys

    def update(self, time_elapsed):
        if self._paused:
            return

        self._average_frame_time = (GameWorld
                ._frame_rate_smoother.update(time_elapsed))

        for vehicle in self._vehicles:
            vehicle.update(time_elapsed)

    def create_walls(self):
        return NotImplemented

    def create_obstacles(self):
        return NotImplemented

    def render(self):
        return NotImplemented


def __main__():
    g = GameWorld(12, 12)

if __name__ == '__init__':
    __main__()
