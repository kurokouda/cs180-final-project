from abc import ABCMeta, abstractmethod
from uuid import uuid4
from numbers import Real

from .entitytype import EntityType
from ..d2.vector2d import Vector2D


class BaseGameEntity(metaclass=ABCMeta):
    @classmethod
    def with_forced_id(cls, entity_type, forced_id):
        new_entity = cls(entity_type)
        #pylint: disable=W0212
        new_entity._instance_id = forced_id
        #pylint: enable=W0212
        return new_entity

    def __init__(self, entity_type=EntityType.DEFAULT, position=Vector2D(),
            bounding_radius=0):
        self._instance_id = uuid4()
        self._bounding_radius = bounding_radius
        self._position = position
        self._scale = Vector2D(1.0, 1.0)
        self._entity_type = entity_type
        self._tag = False

    @abstractmethod
    def update(self, time_elapsed):
        pass

    @abstractmethod
    def render(self, pr):
        pass

    # @abstractmethod
    # def handle_message(self, telegram_msg):
    #     pass

    def get_position(self):
        return Vector2D.from_vec(self._position)

    def set_position(self, value):
        self._position.set(value)

    def get_bounding_radius(self):
        return self._bounding_radius

    def set_bounding_radius(self, value):
        self._bounding_radius = value

    def get_id(self):
        return self._instance_id

    def is_tagged(self):
        return self._tag

    def tag(self):
        self._tag = True

    def untag(self):
        self._tag = False

    def get_scale(self):
        return Vector2D.from_vec(self._scale)

    def set_scale(self, value):
        if isinstance(value, Real):
            self.bounding_radius *= value / max(*self.scale)
            self.scale = Vector2D(value, value)
        elif isinstance(value, Vector2D):
            self.bounding_radius *= max(*value) / max(*self.scale)
            self.scale = Vector2D.from_vec(value)

    def get_entity_type(self):
        return self._entity_type

    def set_entity_type(self, value):
        self._entity_type = value

    position = property(get_position, set_position)
    bounding_radius = property(get_bounding_radius, set_bounding_radius)
    instance_id = property(get_id)
    scale = property(get_scale, set_scale)
    entity_type = property(get_entity_type, set_entity_type)

    # def read(self, data):
    #     pass

    def to_string(self):
        return '<{} ({}, {})>'.format(self.__class__.__name__,
                self._position.x, self._position.y)

    def __eq__(self, other):
        return self.instance_id == other.instance_id

    def __hash__(self):
        return int(self._instance_id)

    def __repr__(self):
        return self.to_string()

    def __str__(self):
        return self.to_string()
