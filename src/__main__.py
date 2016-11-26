import pygame

from .config import Config
from .context import Context


def _main():
    pygame.init()
    Context().clock = pygame.time.Clock()
    print(Config().__dict__)

    screen = pygame.display.set_mode((Config().window.WIDTH,
            Config().window.HEIGHT))
    running = True

    while running:
        Context().clock.tick(Config().window.FPS)

        pygame.display.set_caption('FPS: {:02.2f}, Time: {:.2f}'.format(
                    Context().clock.get_fps(), pygame.time.get_ticks() * 0.001
        ))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(pygame.Color('gray20'))
        pygame.display.flip()

if __name__ == '__main__':
    _main()
