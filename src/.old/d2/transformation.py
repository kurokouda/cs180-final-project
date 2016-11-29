from copy import deepcopy

from .c2dmatrix import C2DMatrix
from .vector2d import Vector2D

def world_transform(points, pos, forward, side, scale=Vector2D(1.0, 1.0)):
    '''Given a list of 2D vectors, a position, orientation and scale,
    this function transforms the 2D vectors into the object's world space
    '''
    transformed_vectors = deepcopy(points)
    matrix_transform = C2DMatrix()

    if scale.x != 1.0 or scale.y != 1.0:
        matrix_transform.scale(*scale)

    matrix_transform.rotate_ip(forward, side)
    matrix_transform.translate_ip(*pos)
    matrix_transform.transform_vector_ip(transformed_vectors)

    return transformed_vectors

def point_to_world_space(point, agent_heading, agent_side, agent_position):
    '''Transforms a point from the agent's local space into world space
    '''
    transformed_point = Vector2D.from_vec(point)
    matrix_transform = C2DMatrix()

    matrix_transform.rotate_ip(agent_heading, agent_side)
    matrix_transform.translate_ip(*agent_position)
    matrix_transform.transform_vector_ip(transformed_point)

    return transformed_point

def vector_to_world_space(vec, agent_heading, agent_side):
    '''Transforms a vector from the agent's local space into world space
    '''
    transformed_vector = Vector2D.from_vec(vec)
    matrix_transform = C2DMatrix()

    matrix_transform.rotate_ip(agent_heading, agent_side)
    matrix_transform.transform_vector_ip(transformed_vector)

    return transformed_vector

def point_to_local_space(point, agent_heading, agent_side, agent_position):
    transformed_point = Vector2D.from_vec(point)
    matrix_transform = C2DMatrix()

    trans_x = -(agent_position * agent_heading)
    trans_y = -(agent_position * agent_side)

    matrix_transform[0, 0] = agent_heading.x
    matrix_transform[0, 1] = agent_side.x
    matrix_transform[1, 0] = agent_heading.y
    matrix_transform[1, 1] = agent_side.y
    matrix_transform[2, 0] = trans_x
    matrix_transform[2, 1] = trans_y

    matrix_transform.transform_vector_ip(transformed_point)
    return transformed_point

def vector_to_local_space(vec, agent_heading, agent_side):
    transformed_point = Vector2D(vec)
    matrix_transform = C2DMatrix()

    matrix_transform[0, 0] = agent_heading.x
    matrix_transform[0, 1] = agent_side.x
    matrix_transform[1, 0] = agent_heading.y
    matrix_transform[1, 1] = agent_side.y

    matrix_transform.transform_vector_ip(transformed_point)
    return transformed_point

def vector_rotate_around_origin_ip(vec, angle_rad):
    matrix_transform = C2DMatrix()
    matrix_transform.rotate_ip(angle_rad)
    matrix_transform.transform_vector_ip(vec)

def vector_rotate_around_origin(vec, angle_rad):
    new_vector = Vector2D.from_vec(vec)
    vector_rotate_around_origin_ip(new_vector, angle_rad)
    return new_vector

def create_whiskers(whisker_count, whisker_length, fov, facing, origin):
    '''Given an origin, a facing direction, a 'field of view' describing the
    limit of the outer whiskers, a whisker length and the number of whiskers
    this method returns a vector containing the end positions of a series
    of whiskers radiating away from the origin and with equal distance between
    them. (like the spokes of a wheel clipped to a specific segment size)
    '''
    sector_size = fov / (whisker_count - 1)
    whiskers = []
    angle = -fov * 0.5

    for _ in range(whisker_count):
        temp = Vector2D.from_vec(facing)
        vector_rotate_around_origin_ip(temp, angle)
        whiskers.append(origin + whisker_length * temp)
        angle += sector_size

    return whiskers
