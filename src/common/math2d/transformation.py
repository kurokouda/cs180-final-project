from copy import deepcopy
from .c2dmatrix import C2DMatrix
from .vector2d import Vector2D

def world_transform(points, pos, forward, side, scale=(1.0, 1.0)):
    '''world_transform(points, pos, forward, side, scale=(1.0, 1.0))
        -> List<MutableSequence>

    Given a list of 2D vectors, a position, orientation and scale,
        this function transforms the 2D vectors into the object's world space

    Keyword arguments:
    points -- Iterable
    pos -- Iterable; len=2
    forward -- Iterable; len=2
    side -- Iterable; len=2
    scale -- Iterable; len=2
    '''
    transformed_vectors = deepcopy(points)
    matrix_transform = C2DMatrix()

    if scale[0] != 1.0 or scale[1] != 1.0:
        matrix_transform.scale(*scale)

    matrix_transform.rotate(forward, side)
    matrix_transform.translate(*pos)
    matrix_transform.transform_vector(transformed_vectors)

    return transformed_vectors

def point_to_world_space(point, agent_heading, agent_side, agent_position):
    '''point_to_world_space(point, agent_heading, agent_side, agent_position)
        -> Vector2D

    Transforms a point from the agent's local space into world space

    Keyword arguments:
    point -- Iterable; len=2
    agent_heading -- Iterable; len=2
    agent_side -- Iterable; len=2
    agent_position -- Iterable; len=2
    '''
    transformed_point = Vector2D(*point)
    matrix_transform = C2DMatrix()

    matrix_transform.rotate(agent_heading, agent_side)
    matrix_transform.translate(*agent_position)
    matrix_transform.transform_vector(transformed_point)

    return transformed_point

def vector_to_world_space(vec, agent_heading, agent_side):
    '''vector_to_world_space(vec, agent_heading, agent_side) -> Vector2D

    Transforms a vector from the agent's local space into world space

    Keyword arguments:
    vec -- Iterable; len=2
    agent_heading -- Iterable; len=2
    agent_side -- Iterable; len=2
    '''
    transformed_vector = Vector2D(*vec)
    matrix_transform = C2DMatrix()

    matrix_transform.rotate(agent_heading, agent_side)
    matrix_transform.transform_vector(transformed_vector)

    return transformed_vector

def point_to_local_space(point, agent_heading, agent_side, agent_position):
    '''point_to_local_space(point, agent_heading, agent_side, agent_position)
        -> Vector2D

    Keyword arguments:
    point -- Iterable; len=2
    agent_heading -- Iterable; len=2
    agent_side -- Iterable; len=2
    agent_position -- Iterable; len=2
    '''
    transformed_point = Vector2D(*point)
    matrix_transform = C2DMatrix()

    trans_x = -agent_position.dot(agent_heading)
    trans_y = -agent_position.dot(agent_side)

    matrix_transform[0, 0] = agent_heading[0]
    matrix_transform[0, 1] = agent_side[0]
    matrix_transform[1, 0] = agent_heading[1]
    matrix_transform[1, 1] = agent_side[1]
    matrix_transform[2, 0] = trans_x
    matrix_transform[2, 1] = trans_y

    matrix_transform.transform_vector(transformed_point)
    return transformed_point

def vector_to_local_space(vec, agent_heading, agent_side):
    '''vector_to_local_space(vec, agent_heading, agent_side) -> Vector2D

    Keyword arguments:
    vec -- Iterable; len=2
    agent_heading -- Iterable; len=2
    agent_side -- Iterable; len=2
    '''
    transformed_point = Vector2D(*vec)
    matrix_transform = C2DMatrix()

    matrix_transform[0, 0] = agent_heading[0]
    matrix_transform[0, 1] = agent_side[0]
    matrix_transform[1, 0] = agent_heading[1]
    matrix_transform[1, 1] = agent_side[1]

    matrix_transform.transform_vector(transformed_point)
    return transformed_point

def vector_rotate_around_origin(vec, angle_rad):
    '''vector_rotate_around_origin(vec, angle_rad) -> MutableSequence; len=2

    Keyword arguments:
    vec -- Iterable; len=2
    angle_rad -- Iterable; len=2
    '''
    matrix_transform = C2DMatrix()
    matrix_transform.rotate(angle_rad)
    matrix_transform.transform_vector(vec)
    return vec

def create_whiskers(whisker_count, whisker_length, fov, facing, origin):
    '''create_whiskers(whisker_count, whisker_length, fov, facing, origin)
        -> Iterable<Vector2D>

    Given an origin, a facing direction, a 'field of view' describing the
        limit of the outer whiskers, a whisker length and the number of
        whiskers this method returns a vector containing the end positions of
        a series of whiskers radiating away from the origin and with equal
        distance between them. (like the spokes of a wheel clipped to a
        specific segment size)

    Keyword arguments:
    whisker_count -- Integer
    whisker_length -- Integer
    fov -- Real
    facing -- Iterable; len=2
    origin -- Iterable; len=2
    '''
    sector_size = fov / (whisker_count - 1)
    whiskers = []
    angle = -fov * 0.5

    for _ in range(whisker_count):
        temp = Vector2D(*facing)
        vector_rotate_around_origin(temp, angle)
        whiskers.append(temp.mul(whisker_length).add(origin))
        angle += sector_size

    return whiskers
