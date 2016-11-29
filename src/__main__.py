import pygame
from .config import Config as Cfg


class App(object):
    def __init__(self):
        pygame.init()
        pygame.display.set_caption(Cfg().title)
        self._clock = pygame.time.Clock()

        window_dim = (Cfg().window.width, Cfg().window.height)
        self._screen = pygame.display.set_mode(window_dim)
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            self._clock.tick(Cfg().window.fps)
            for event in pygame.event.get():
                if (event.type == pygame.QUIT or
                        (event.type == pygame.KEYDOWN
                            and event.key == pygame.K_ESCAPE)):
                    self._running = False

            self._screen.fill(pygame.Color('#272430'))
            pygame.display.update()
        pygame.quit()

def main():
    App().run()

if __name__ == '__main__':
    main()
