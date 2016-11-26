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
        '_average_fps',
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

    def __init__(self, window_width, window_height):
        self._vehicles = []
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
        self._average_fps = 0
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
