from collections import OrderedDict
from .singleton import SingletonABCMeta

class EntityManager(object, metaclass=SingletonABCMeta):
    def __init__(self):
        self._entity_map = OrderedDict()

    def register_entity(self, entity):
        self._entity_map[entity.instance_id] = entity

    def get_entity_from_id(self, entity_id):
        return self._entity_map[entity_id]

    def remove_entity(self, entity):
        del self._entity_map[entity.instance_id]
