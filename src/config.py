from pathlib import Path
from math import pi as PI
import os
import json

from .singleton import SingletonABCMeta

_CWD = os.path.dirname(os.path.realpath(__file__))
_CONFIGFILE = 'config.json'
_COMMENT_KEY = '__comment__'


#pylint: disable=R0903
class _Struct(object):
    def __init__(self, entries):
        self.__dict__.update(entries)
        for key, value in entries.items():
            if isinstance(value, dict):
                self.__dict__[key] = _Struct(value)
            if isinstance(value, list):
                for i, value2 in enumerate(value):
                    if isinstance(value2, dict):
                        self.__dict__[key][i] = _Struct(value2)


class Config(_Struct, metaclass=SingletonABCMeta):
    def __init__(self, config_path=os.path.join(_CWD, _CONFIGFILE)):
        path_obj = Path(config_path)

        with path_obj.open('r') as file_obj:
            json_data = json.load(file_obj)

        if json_data.get(_COMMENT_KEY) is not None:
            del json_data[_COMMENT_KEY]

        self._apply_steering_force_tweaker(json_data)
        self._add_extras(json_data)
        super().__init__(json_data)

    def _apply_steering_force_tweaker(self, json_data):
        tweaker = json_data['STEERING_FORCE_TWEAKER']
        json_data['STEERING_FORCE'] *= tweaker
        for key in json_data['weights']:
            json_data['weights'][key] *= tweaker

    def _add_extras(self, json_data):
        json_data['MAX_TURN_RATE_PER_SECOND'] = PI
        json_data['CWD'] = _CWD

#pylint: enable=R0903
