import pygame

from .config import Config
# from .context import Context


class Main(object):
    def __init__(self):
        pygame.init()
        self._clock = pygame.time.Clock()
        self._window_width = Config().window.WIDTH
        self._window_height = Config().window.HEIGHT
        self._screen = pygame.display.set_mode(
                (Config().window.WIDTH, Config().window.HEIGHT))
        self._running = False

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

            self._screen.fill(pygame.Color('gray20'))
            pygame.display.flip()

if __name__ == '__main__':
    Main().run()
