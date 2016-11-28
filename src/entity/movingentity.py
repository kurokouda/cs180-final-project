from abc import ABCMeta
import math

from ..d2.vector2d import Vector2D
from ..d2.c2dmatrix import C2DMatrix
from .basegameentity import BaseGameEntity
from .entitytype import EntityType


class MovingEntity(BaseGameEntity, metaclass=ABCMeta):
    def __init__(self,
            position,
            bounding_radius,
            velocity,
            max_speed,
            heading,
            mass,
            scale,
            max_turn_rate,
            max_force):
        super().__init__(EntityType.MOVING_ENTITY, position, bounding_radius)
        self._heading = Vector2D.from_vec(heading)
        self._velocity = Vector2D.from_vec(velocity)
        self._mass = mass
        self._side = self._heading.perpendicular()
        self._max_speed = max_speed
        self._max_turn_rate = max_turn_rate
        self._max_force = max_force
        self._scale = Vector2D.from_vec(scale)

    # @abstractmethod
    # def update(self, time_elapsed):
    #     pass
    #
    # @abstractmethod
    # def render(self, pr):
    #     pass
    #
    # @abstractmethod
    # def handle_message(self, telegram_msg):
    #     pass

    def get_velocity(self):
        return Vector2D.from_vec(self._velocity)

    def set_velocity(self, value):
        self._velocity.set(value)

    def get_mass(self):
        return self._mass

    def get_side(self):
        return Vector2D.from_vec(self._side)

    def get_max_speed(self):
        return self._max_speed

    def set_max_speed(self, value):
        self._max_speed = value

    def get_max_force(self):
        return self._max_force

    def set_max_force(self, value):
        self._max_force = value

    def get_speed(self):
        return self._velocity.length()

    def get_speed_sq(self):
        return self._velocity.length_sq()

    def get_heading(self):
        return self._heading

    def set_heading(self, value):
        if value.length_sq() - 1.0 >= 1e-5:
            raise ValueError('Value is vector of zero length')

        self._heading = value
        self._side = self._heading.perpendicular()

    def get_max_turn_rate(self):
        return self._max_turn_rate

    def set_max_turn_rate(self, value):
        self._max_turn_rate = value

    velocity = property(get_velocity, set_velocity)
    mass = property(get_mass)
    side = property(get_side)
    max_speed = property(get_max_speed, set_max_speed)
    max_force = property(get_max_force, set_max_force)
    speed = property(get_speed)
    speed_sq = property(get_speed_sq)
    heading = property(get_heading, set_heading)
    max_turn_rate = property(get_max_turn_rate, set_max_turn_rate)

    def is_speed_maxed_out(self):
        return self.max_speed * self.max_speed >= self.velocity.length_sq()

    def rotate_heading_to_face_target(self, target):
        to_target = (target - self.position).normalize()

        angle = math.acos(self.heading * target)
        if math.isnan(angle):
            angle = 0

        if angle < 1e-5:
            return True

        if angle > self.max_turn_rate:
            angle = self._max_turn_rate

        rotation_matrix = C2DMatrix()
        rotation_matrix.rotate_ip(angle * self.heading.sign(to_target))
        self.heading = rotation_matrix.transform_vector(self.heading)
        self.velocity = rotation_matrix.transform_vector(self.velocity)
        self.side = self.heading.perpendicular()
        return False
