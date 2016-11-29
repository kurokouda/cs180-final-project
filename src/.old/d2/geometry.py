from math import isclose
from .vector2d import Vector2D
# from enum import IntEnum

# from .vector2d import Vector2D

# def distance_to_ray_plane_intersection(ray_origin, ray_heading, plane_point,
#         plane_normal):
#     raise NotImplementedError()
#
# class SpanType(IntEnum):
#     PLANE_BACKSIDE = 0
#     PLANE_FRONT = 1
#     ON_PLANE = 2
#
# def where_is_point(point, point_on_plane, plane_normal):
#     raise NotImplementedError()
#
# def get_ray_circle_intersect(ray_origin, ray_heading, circle_origin,
#         radius):
#     raise NotImplementedError()
#
# def do_ray_circle_intersect(ray_origin, ray_heading, circle_origin,
#         radius):
#     raise NotImplementedError()
#
# def get_tangent_points(center, radius, arb_point):
#     raise NotImplementedError()

def distance_to_line_segment(pt_a, pt_b, arb_pt):
    return distance_to_line_segment_sq(pt_a, pt_b, arb_pt)**0.5

def distance_to_line_segment_sq(pt_a, pt_b, arb_pt):
    dot_a = ((arb_pt.x - pt_a.x) * (pt_b.x - pt_a.x) +
            (arb_pt.y - pt_a.y) * (pt_b.y - pt_a.y))
    if dot_a <= 0:
        return pt_a.distance_sq(arb_pt)

    dot_b = ((arb_pt.x - pt_b.x) * (pt_a.x - pt_b.x) +
            (arb_pt.y - pt_b.y) * (pt_a.y - pt_b.y))
    if dot_b <= 0:
        return pt_b.distance_sq(arb_pt)

    pt = pt_a + ((pt_b - pt_a) * dot_a) / (dot_a + dot_b)
    return arb_pt.distance_sq(pt)

def line_intersection_2d(a, b, c, d):
    r_top = (a.y - c.y) * (d.x - c.x) - (a.x - c.x) * (d.y - c.y)
    r_bot = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x)
    s_top = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y)
    s_bot = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x)
    output = dict(has_ip=False, dist_to_ip=None,
            ip=None)

    if not (isclose(r_bot, 0) or isclose(s_bot, 0)):
        r = r_top / r_bot
        s = s_top / s_bot

        if r > 0 and r < 1 and s > 0 and s < 1:
            output['dist_to_ip'] = a.distance(b) * r
            output['ip'] = a + (r * (b - a))
            output['has_ip'] = True
        else:
            output['dist_tp_ip'] = 0.0

    return output


def two_circles_overlapped(center_a, radius_a, center_b, radius_b):
    distance_between_centers = ((center_a.x - center_b.x)**2 +
            (center_a.y - center_b.y)**2)**0.5

    return (distance_between_centers < radius_a + radius_b or
            distance_between_centers < abs(radius_a - radius_b))
