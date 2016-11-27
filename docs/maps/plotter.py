
from collections import OrderedDict
from pprint import pprint
import math
import json
import pygame

MAP_FILE = 'edited_map_1st.png'
POINTS_FILE = 'map_1st.json'
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
FPS = 60

SIDE = WINDOW_WIDTH / 320
FONT = None


class Point(object):
    _id = 0
    radius = 2

    @staticmethod
    def get_grid_pt(loc):
        x, y = loc
        return int(math.floor(x / SIDE)), int(math.floor(y / SIDE))

    def __init__(self, x, y):
        self.id = Point._id
        Point._id += 1
        self.grid_x, self.grid_y = Point.get_grid_pt((x, y))
        self.x = self.grid_x * 4
        self.y = self.grid_y * 4

    @property
    def pos(self):
        return self.x, self.y

    @property
    def grid_pos(self):
        return self.grid_x, self.grid_y

    def draw(self, surface):
        s = pygame.Surface((30, 30))
        s.convert_alpha()
        s.set_colorkey(pygame.Color('black'))

        pos_in_s = (Point.radius, Point.radius)
        pygame.draw.circle(s, pygame.Color('red'), pos_in_s, Point.radius)

        text = FONT.render(str(self.id), False, pygame.Color('red'))
        place = Point.radius*2 + 1
        s.blit(text, (place, place))

        surface.blit(s, (self.x, self.y))

    def get_data(self):
        d = OrderedDict()
        d['id'] = self.id
        d['x'] = self.grid_x
        d['y'] = self.grid_y
        return d

    def __str__(self):
        return str(self.grid_pos)

    def __repr__(self):
        d = dict()
        d['id'] = self.id
        d['x'] = self.grid_x
        d['y'] = self.grid_x
        return str(d)

def main():

    points = dict((i, dict()) for i in range(WINDOW_HEIGHT))

    try:
        istream = open(POINTS_FILE, 'r')
        data = json.loads(istream.read())
        istream.close()

        max_id = 0
        for pt in data['points']:
            new_pt = Point(pt['x'] * 4, pt['y'] * 4)
            new_pt.id = pt['id']
            points[new_pt.y][new_pt.x] = new_pt

            max_id = max_id if max_id > new_pt.id else new_pt.id

        Point._id = max_id + 1
    except FileNotFoundError:
        pass



    global FONT
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    running = True

    font_name = pygame.font.get_default_font()
    FONT = pygame.font.Font(font_name, 10)

    dcs_map = pygame.image.load(MAP_FILE)
    dcs_map = pygame.transform.scale(dcs_map, (WINDOW_WIDTH, WINDOW_HEIGHT))

    grid = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
    grid.convert_alpha()
    grid.set_colorkey(pygame.Color('black'))

    # for i in range(320):
    #     src = ((i + 1) * 4, 0)
    #     dest = ((i + 1) * 4, WINDOW_HEIGHT-1)
    #     pygame.draw.line(grid, pygame.Color('red'), src, dest)


    d_pressed = False

    rad_check = 2
    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN and not d_pressed:
                if event.key == pygame.K_d:
                    d_pressed = True

            if event.type ==  pygame.MOUSEBUTTONDOWN:
                loc = pygame.mouse.get_pos()
                x, y = loc

                pressed = pygame.mouse.get_pressed()

                if pressed[0]:
                    new_pt = Point(x, y)
                    print('New pt @ {} | In grid (320x240) >> {}'
                            .format(new_pt.pos, new_pt.grid_pos))
                    points[new_pt.y][new_pt.x] = new_pt

                if pressed[2]:
                    for i in range(-rad_check, rad_check+1):
                        for j in range(-rad_check, rad_check+1):
                            if points[y+j].get(x+i):
                                del points[y+j][x+i]

        grid.fill(pygame.Color('black'))
        for row in points.values():
            for pt in row.values():
                pt.draw(grid)

        screen.blit(dcs_map, (0, 0))
        screen.blit(grid, (0, 0))
        pygame.display.flip()

    output_points = []
    for row in points.values():
        for pt in row.values():
            output_points.append(pt.get_data())
    output_points.sort(key=lambda x: x['id'])

    json_dict = dict(points=output_points)
    print(json.dumps)

    ostream = open(POINTS_FILE, 'w')
    print(json.dumps(json_dict, sort_keys=True, indent=4,
            separators=(',', ': ')), file=ostream)
    ostream.close()

    pygame.quit()

if __name__ == '__main__':
    main()
