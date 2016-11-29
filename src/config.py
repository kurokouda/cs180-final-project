import os
import json
from pathlib import Path
from types import SimpleNamespace

from .common.singleton import singleton

_CWD = os.path.dirname(os.path.abspath(__file__))
_FILENAME = 'config.json'

@singleton
class Config(object):
    def __init__(self):
        configpath = Path(os.path.join(_CWD, _FILENAME))
        print('Load config from:', configpath)

        if not configpath.is_file():
            raise FileNotFoundError('File not found: ' + _FILENAME)

        with configpath.open() as cfgfileobj:
            self._cfg = json.load(cfgfileobj,
                    object_hook=lambda obj: SimpleNamespace(**obj))

    def __getattr__(self, name):
        return getattr(self._cfg, name)
