from pathlib import Path
import sys
import os
import json

import pygame

from .config import Config
from .floor import Floor
# from .context import Context


BLIT_CORNERS = [
    (0, 0),
    (Config().window.WIDTH // 2, 0),
    (Config().window.WIDTH // 2, Config().window.HEIGHT // 2),
]

class Main(object):
    def __init__(self):
        pygame.init()
        self._clock = pygame.time.Clock()
        self._window_width = Config().window.WIDTH
        self._window_height = Config().window.HEIGHT
        self._screen = pygame.display.set_mode(
                (Config().window.WIDTH, Config().window.HEIGHT))
        self._running = False
        self._canvases = []
        self._floors = []

        self._load_floors()

    def run(self):
        self._running = True
        while self._running:
            self._clock.tick(Config().window.FPS)
            pygame.display.set_caption('FPS: {:02.2f}, Time: {:.2f}'.format(
                        self._clock.get_fps(), pygame.time.get_ticks() * 0.001
                        ))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False

            for corner, floor in zip(BLIT_CORNERS, self._floors):
                self._screen.blit(floor.render(), corner)

            pygame.display.flip()
        pygame.quit()


    def _load_floors(self):
        directory = os.path.join(Config().CWD, Config().maps.DIRECTORY)
        for filename in Config().maps.FILENAMES:
            file_obj = Path(os.path.join(directory, filename))

            if not file_obj.is_file():
                raise FileNotFoundError('File {} not found'.format(file_obj))

            with file_obj.open() as json_file:
                json_data = json.load(json_file)

            self._floors.append(Floor(json_data))






if __name__ == '__main__':
    Main().run()
