from collections import namedtuple
from pathlib import Path
import json
import pygame


GRID_WIDTH = 320
GRID_HEIGHT = 240
# MAP_FILENAME = 'map_1st.json'
MAP_FILENAME = 'map_3.json'
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
FPS = 60

POINT_SCALE = int(WINDOW_WIDTH / GRID_WIDTH)

class Color(object):
    BLACK = pygame.Color('black')
    WHITE = pygame.Color('white')
    GREEN = pygame.Color('green')
    BLUE = pygame.Color('blue')
    GRAY = pygame.Color('gray')


class Point(object):
    instances = dict()

    def __init__(self, id, grid_x, grid_y):
        Point.instances[id] = self
        self.id = id
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.x = grid_x * POINT_SCALE
        self.y = grid_y * POINT_SCALE

    @property
    def grid_pos(self):
        return self.grid_x, self.grid_y

    @property
    def pos(self):
        return self.x, self.y


class Wall(object):
    instances = dict()

    def __init__(self, id, fromPt, toPt):
        Wall.instances[id] = self

        self.id = id
        self.fromPt = Point.instances[fromPt]
        self.toPt = Point.instances[toPt]

    def draw(self, surface):
        fromPt = self.fromPt.pos
        toPt = self.toPt.pos
        pygame.draw.line(surface, Color.BLACK, fromPt, toPt)


class Door(object):
    instances = dict()
    crosshair_rad = 4

    def __init__(self, id, fromPt, toPt):
        Door.instances[id] = self

        self.id = id
        self.fromPt = Point.instances[fromPt]
        self.toPt = Point.instances[toPt]

    def draw(self, surface):
        x1, y1 = self.fromPt.pos
        x2, y2 = self.toPt.pos
        midPt = abs(x2 + x1) // 2, abs(y2 + y1) // 2
        mx, my = midPt

        r = Door.crosshair_rad * 2
        s = pygame.Surface((r, r))
        s.convert_alpha()
        s.set_colorkey(Color.BLACK)

        pos_in_s = (Door.crosshair_rad, Door.crosshair_rad)
        rad = Door.crosshair_rad - 2
        pygame.draw.circle(s, Color.GREEN, pos_in_s, rad)

        surface.blit(s, (mx - Door.crosshair_rad, my - Door.crosshair_rad))

class Triangle(object):
    instances = dict()

    def __init__(self, id, pts):
        Triangle.instances[id] = self
        self.id = id
        self.pts = [Point.instances[i] for i in pts]

    def draw(self, surface):
        pts = list(map(lambda pt: pt.pos, self.pts))
        pygame.draw.polygon(surface, Color.GRAY, pts, 1)




def load_objects():
    map_file = Path(MAP_FILENAME)

    if not map_file.is_file():
        return

    with map_file.open() as file:
        data = json.load(file)

    for pt in data['points']:
        Point(pt['id'], pt['x'], pt['y'])

    for wall in data['walls']:
        Wall(wall['id'], wall['from'], wall['to'])

    for door in data['doors']:
        Door(door['id'], door['from'], door['to'])

    for tri in data['triangles']:
        Triangle(tri['id'], tri['points'])






def main():
    load_objects()

    pygame.init()

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    running = True

    map_surf = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
    map_surf.convert()
    map_surf.fill(Color.WHITE)

    for door in Door.instances.values():
        door.draw(map_surf)

    for tri in Triangle.instances.values():
        tri.draw(map_surf)

    for wall in Wall.instances.values():
        wall.draw(map_surf)


    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and
                        event.key == pygame.K_ESCAPE)
                    ):
                running = False

        screen.blit(map_surf, (0, 0))
        pygame.display.flip()



    pygame.quit()


if __name__ == '__main__':
    main()