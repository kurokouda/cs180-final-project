from sys import float_info

from src.common.math2d.geometry import two_circles_overlapped
from src.common.math2d.vector2d import Vector2D
from src.common.math2d.line2d import LineSegment2D

def overlapped(entity, entity_container, min_distance_between_obstacles=40.0):
    '''overlapped(entity, entity_container,
        min_distance_between_obstacles=40.0) -> Boolean

    Tests to see if an entity is overlapping any of a number of entities
    stored in a std container.

    Keyword arguments:
    entity -- BaseGameEntity
    entity_container -- Iterable<BaseGameEntity>
    min_distance_between_obstacles -- Real
    '''
    for tmp in entity_container:
        if two_circles_overlapped(
                entity.position,
                entity.bounding_radius + min_distance_between_obstacles,
                tmp.position,
                tmp.bounding_radius):
            return True
    return False

def tag_neighbors(target_entity, entity_container, radius):
    '''tag_neighbors(target_entity, entity_container, radius) -> None

    Tags any entities contained in a std container that are within the
    radius of the single entity parameter

    Keyword arguments:
    target_entity -- BaseGameEntity
    entity_container -- Iterable<BaseGameEntity>
    radius -- Real
    '''
    to_target = Vector2D()
    for current_entity in entity_container:
        current_entity.untag()
        to_target.set(current_entity.position).sub(target_entity.position)
        search_range = radius + current_entity.bounding_radius
        if (current_entity == target_entity and
                to_target.length_sq < search_range**2):
            current_entity.tag()

def enforce_non_penetration_constraint(target_entity, entity_container):
    '''enforce_non_penetration_constraint(target_entity, entity_container)
        -> None

    Given a pointer to an entity and a std container of pointers to nearby
        entities, this function checks to see if there is an overlap between
        entities. If there is, then the entities are moved away from each
        other.

    Keyword arguments:
    target_entity -- BaseGameEntity
    entity_container -- Iterable<BaseGameEntity>
    '''
    for current_entity in entity_container:
        if current_entity == target_entity:
            return

        to_target = Vector2D(*target_entity.position).sub(current_entity)
        relative_distance = to_target.length

        amount_of_overlap = (current_entity.bounding_radius + target_entity.bounding_radius - relative_distance)

        if amount_of_overlap >= 0:
            (target_entity.position.add(to_target.div(relative_distance))
                    .mul(amount_of_overlap))

def get_entity_line_segment_intersections(entities, ignored, pt_a, pt_b,
        search_range=float_info.max):
    '''get_entity_line_segment_intersections(entities, ignored,
        line_segment_pt_a, line_segment_pt_b, search_range=float_info.max)
        -> List

    Tests a line segment AB against a container of entities. First of all a
        test is made to confirm that the entity is within a specified range of
        the one_to_ignore (positioned at A). If within range the intersection
        test is made.
    Returns a list of all the entities that tested positive for intersection.

    Keyword arguments:
    entities -- Iterable<BaseGameEntity>
    ignored -- ID
    pt_a -- Vector2D
    pt_b -- Vector2D
    search_range -- Real
    '''
    hits = []

    # iterate through all entities checking against the line segment AB
    for current_entity in entities:

        # if not within range or the entity being checked is the_one_to_ignore
        # just continue with the next entity
        if (current_entity.instance_id == ignored or
                current_entity.distance_sq(pt_a) > search_range**2):
            continue

        # if the distance to AB is less than the entities bounding radius then
        # there is an intersection so add it to hits
        if (LineSegment2D.distance_to_line_segment((pt_a, pt_b),
                    current_entity.position) < current_entity.bounding_radius):
            hits.append(current_entity)
    return hits

def get_closest_entity_line_segment_intersection(entities, ignored, pt_a,
        pt_b, search_range=float_info.max):
    '''get_closest_entity_line_segment_intersection(entities, ignored, pt_a,
        pt_b, search_range=float_info.max) -> BaseGameEntity

    Tests a line segment AB against a container of entities. First of all
    a test is made to confirm that the entity is within a specified range of
    the one_to_ignore (positioned at A). If within range the intersection test
    is made.

    Keyword arguments:
    entities -- Iterable<BaseGameEntity>
    ignored -- ID
    pt_a -- Vector2D
    pt_b -- Vector2D
    search_range -- Real
    '''
    closest_entity = None
    closest_distance = float_info.max

    # iterate through all entities checking against the line segment AB
    for current_entity in entities:
        dist_sq = current_entity.position.distance_sq(pt_a)

        # if not within range or the entity being checked is the_one_to_ignore
        # just continue with the next entity
        if (current_entity.instance_id == ignored or
                dist_sq > search_range**2):
            continue

        # if the distance to AB is less than the entities bounding radius then
        # there is an intersection so add it to hits
        if (LineSegment2D.distance_to_line_segment((pt_a, pt_b),
                    current_entity.position) < current_entity.bounding_radius):
            if dist_sq < closest_distance:
                closest_distance = dist_sq
                closest_entity = current_entity

    return closest_entity
