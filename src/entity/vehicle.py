from copy import error as CopyError
import math

from ..d2.vector2d import Vector2D
from ..smoother import Smoother
from ..steeringbehaviors import SteeringBehaviors
from ..config import Config
from .movingentity import MovingEntity

class Vehicle(MovingEntity):
    _NUM_VEHICLE_VERTS = 3

    def __init__(self,
            world,
            position,
            rotation,
            velocity,
            mass,
            max_force,
            max_speed,
            max_turn_rate,
            scale):
        super().__init__(
                position=position,
                bounding_radius=scale,
                velocity=velocity,
                max_speed=max_speed,
                heading=Vector2D(math.sin(rotation), -math.cos(rotation)),
                mass=mass,
                scale=Vector2D(scale, scale),
                max_turn_rate=max_turn_rate,
                max_force=max_force)
        self._world = world
        self._smoothed_heading = Vector2D()
        self._smoothing_on = False
        self._time_elapsed = 0.0

        self._vehicle_vertex_buffer = []
        self._initialize_buffer()

        self._steering = SteeringBehaviors(self)
        self._heading_smoother = Smoother(Config().NUM_SAMPLES_FOR_SMOOTHING,
                Vector2D())

    def _initialize_buffer(self):
        self._vehicle_vertex_buffer.extend([
                Vector2D(-1.0, 0.6),
                Vector2D(1.0, 0.0),
                Vector2D(-1.0, -0.6),
                ])

    def get_steering_behavior(self):
        return self._steering

    def get_world(self):
        return self._world

    def get_smoothed_heading(self):
        return Vector2D.from_vec(self._smoothed_heading)

    def is_smoothing_on(self):
        return self._smoothing_on

    def smoothing_on(self):
        self._smoothing_on = True

    def smoothing_off(self):
        self._smoothing_on = False

    def toggle_smoothing(self):
        self._smoothing_on = not self._smoothing_on

    def get_time_elapsed(self):
        return self._time_elapsed

    steering = property(get_steering_behavior)
    world = property(get_world)
    smoothed_heading = property(get_smoothed_heading)
    time_elapsed = property(get_time_elapsed)

    def update(self, time_elapsed):
        self._time_elapsed = time_elapsed
        old_pos = self.position

        steering_force = self.steering.calculate()
        acceleration = steering_force / self.mass
        self.velocity += acceleration * time_elapsed
        self.velocity.truncate_ip(self.max_speed)
        self.position += self.velocity * time_elapsed

        if self.velocity.length_sq() > 1e-8:
            self.heading = self.velocity.normalize()

        # self.enforce_non_penetration_constraint(self,
        #     self.game_world.agnets())

        Vector2D.wrap_around(self.position, Config().window.WIDTH,
                Config().window.HEIGHT)

        if self.steering.is_space_partition_on():
            self.world.cell_space.update_entity(self, old_pos)

        if self.is_smoothing_on():
            self._smoothed_heading = self._heading_smoother.update(
                    self.heading)

    def render(self, pr):
        pass

    def __copy__(self):
        raise CopyError('Copying not permitted')

    def __deepcopy__(self, memo=None):
        raise CopyError('Copying not permitted')

def _main():
    v1 = Vehicle(None, Vector2D(), 0, Vector2D(), 12, 12, 12, 12, 1)

if __name__ == '__main__':
    _main()
