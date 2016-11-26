from .d2.vector2d import Vector2D
from .entity.entityfunctiontemplates import (
    enforce_non_penetration_constraint,
    tag_neighbors,
)

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
        self._window_width = window_width
        self._window_height = window_height
        self._paused = False
        self._crosshair = Vector2D(window_width / 2,)

    def non_penetration_constraint(self, vehicle):
        enforce_non_penetration_constraint(vehicle, self._vehicles)

    def tag_vehicles_within_view_range(self, vehicle, range_):
        tag_neighbors(vehicle, self._vehicles, range_)
