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


# def line_intersection_2d(line_1_pt_a, line_1_pt_b, line_2_pt_a, line_2_pt_b):
#     raise NotImplementedError()

def two_circles_overlapped(center_a, radius_a, center_b, radius_b):
    distance_between_centers = ((center_a.x - center_b.x)**2 +
            (center_a.y - center_b.y)**2)**0.5

    return (distance_between_centers < radius_a + radius_b or
            distance_between_centers < abs(radius_a - radius_b))
