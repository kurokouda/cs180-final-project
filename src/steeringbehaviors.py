from collections import OrderedDict
from sys import float_info
from enum import IntEnum, unique
from random import random
import math

from .config import Config
from .d2.vector2d import Vector2D
from .path import Path
from .utils import random_clamped
from .d2.transformation import (
        point_to_world_space,
        point_to_local_space,
        vector_to_world_space,
        create_whiskers,
        )
from src.common.math2d.line2d import LineSegment2D

Cfg = Config()

# The radius of the constraining circle for the wander behavior
WANDER_RADIUS = 1.0

# Distance the wander circle is projected in front of the agent
WANDER_DISTANCE = 1.0

# The maximum amount of displacement along the circle each frame
WANDER_JITTER_PER_SEC = 1.0

# Used in path following
WAYPOINT_SEEK_DIST = 1.0


@unique
class SummingMethod(IntEnum):
    WEIGHTED_AVERAGE = 0
    PRIORITIZED = 1
    DITHERED = 2


@unique
class BehaviorType(IntEnum):
    NONE = 1 << 0
    SEEK = 1 << 1
    FLEE = 1 << 2
    ARRIVE = 1 << 3
    WANDER = 1 << 4
    COHESION = 1 << 5
    SEPARATION = 1 << 6
    ALIGNMENT = 1 << 7
    OBSTACLE_AVOIDANCE = 1 << 8
    WALL_AVOIDANCE = 1 << 9
    FOLLOW_PATH = 1 << 10
    PURSUIT = 1 << 11
    EVADE = 1 << 12
    INTERPOSE = 1 << 13
    HIDE = 1 << 14
    FLOCK = 1 << 15
    OFFSET_PURSUIT = 1 << 16

    @property
    def flag(self):
        return self.value

    def is_on(self, flag):
        return self & flag == self

    # def switch_on(self, flag):
    #     return self | flag
    #
    # def switch_off(self, flag):
    #     return flag ^ self


class Deceleration(IntEnum):
    SLOW = 3
    NORMAL = 2
    FAST = 1


class SteeringBehaviors(object):
    '''SteeringBehaviors(vehicle) -> SteeringBehaviors

    A class that encapsulates all steering behaviors for an agent.

    vehicle         -- Vehicle
    '''
    __slots__ = (
        '_vehicle',
        '_steering_force',

        '_target_agent_1',
        '_target_agent_2',
        '_target',

        '_detection_box_length',
        '_feelers',
        '_wall_detection_feeler_length',

        '_wander_target',
        '_wander_jitter',
        '_wander_radius',
        '_wander_distance',

        '_weight_separation',
        '_weight_cohesion',
        '_weight_alignment',
        '_weight_wander',
        '_weight_obstacle_avoidance',
        '_weight_wall_avoidance',
        '_weight_seek',
        '_weight_flee',
        '_weight_arrive',
        '_weight_pursuit',
        '_weight_offset_pursuit',
        '_weight_interpose',
        '_weight_hide',
        '_weight_evade',
        '_weight_follow_path',

        '_view_distance',
        '_path',
        '_waypoint_seek_distance_sq',
        '_offset',
        '_flags',
        '_deceleration',
        '_cell_space_on',
        '_summing_method',
    )

    def __init__(self, vehicle):
        # A pointer to the owner of this instance
        self._vehicle = vehicle

        self._init_weights()
        self._init_wander_variables()

        # The steering force created by the combined effect of all
        # the selected behaviors
        self._steering_force = Vector2D()

        # Binary flags to indicate whether or not a behavior should be active
        self._flags = BehaviorType.NONE

        # Length of the 'detection box' utilized in obstacle avoidance
        self._detection_box_length = Cfg.MIN_DETECTION_BOX_LENGTH

        # How far the agent can see
        self._view_distance = Cfg.VIEW_DISTANCE

        # The length of the 'feeler/s' used in wall detection
        self._wall_detection_feeler_length = (
                Cfg.WALL_DETECTION_FELLER_LENGTH)

        # A vertex buffer to contain the feelers rqd for wall avoidance
        self._feelers = []
        self._deceleration = Deceleration.NORMAL
        self._target = Vector2D()

        # Will be used to keep track of friends, pursuers, or prey
        self._target_agent_1 = None
        self._target_agent_2 = None

        self._cell_space_on = False
        self._summing_method = SummingMethod.PRIORITIZED

        # Any offset used for formations or offset pursuit
        self._offset = None

        # Any current path
        self._path = Path()
        self._path.loop_on()

    def _init_weights(self):
        '''Initialize weights for agents steering behaviors.

        Multipliers. These can be adjusted to effect strength of the
        appropriate behavior. Useful to get flocking the way you require
        for example.
        '''
        self._weight_cohesion = Cfg.weights.COHESION
        self._weight_alignment = Cfg.weights.ALIGNMENT
        self._weight_separation = Cfg.weights.SEPARATION
        self._weight_obstacle_avoidance = Cfg.weights.OBSTACLE_AVOIDANCE
        self._weight_wander = Cfg.weights.WANDER
        self._weight_wall_avoidance = Cfg.weights.WALL_AVOIDANCE
        self._weight_seek = Cfg.weights.SEEK
        self._weight_flee = Cfg.weights.FLEE
        self._weight_arrive = Cfg.weights.ARRIVE
        self._weight_pursuit = Cfg.weights.PURSUIT
        self._weight_offset_pursuit = Cfg.weights.OFFSET_PURSUIT
        self._weight_interpose = Cfg.weights.INTERPOSE
        self._weight_hide = Cfg.weights.HIDE
        self._weight_evade = Cfg.weights.EVADE
        self._weight_follow_path = Cfg.weights.FOLLOW_PATH

    def _init_wander_variables(self):
        self._wander_distance = WANDER_DISTANCE
        self._wander_jitter = WANDER_JITTER_PER_SEC
        self._wander_radius = WANDER_RADIUS

        # The distance (squared) a vehicle has to be from a path waypoint
        # before it starts seeking to the next waypoint
        self._waypoint_seek_distance_sq = (WAYPOINT_SEEK_DIST *
                WAYPOINT_SEEK_DIST)

        # The current position on the wander circle the agent is
        # attempting to steer towards
        theta = random() * math.pi
        self._wander_target = Vector2D(self._wander_radius * math.cos(theta),
                self._wander_radius * math.sin(theta))

    def _accumulate_force(self, running_total, force_to_add):
        '''This function calculates how much of its max steering force the
            vehicle has left to apply and then applies that amount of the
            force to add.
        '''
        output = OrderedDict()
        output['has_force_left'] = False
        output['running_total'] = Vector2D(*running_total)

        # calculate how much steering force the vehicle has used so far
        magnitude_so_far = running_total.length

        # calculate how much steering force remains to be used by this vehicle
        magnitude_remaining = self._vehicle.max_force - magnitude_so_far

        # return false if there is no more force left to use
        if magnitude_remaining > 0:

            # calculate the magnitude of the force we want to add
            magnitude_to_add = force_to_add.length
            output['has_force_left'] = True

            # if the magnitude of the sum of ForceToAdd and the running total
            # does not exceed the maximum force available to this vehicle, just
            # add together. Otherwise add as much of the ForceToAdd vector is
            # possible without going over the max.
            if magnitude_to_add < magnitude_remaining:
                output['running_total'].add(force_to_add)
            else:
                output['running_total'].add(Vector2D(*force_to_add).normalize()
                            .mul(magnitude_remaining))

        return output

    def _create_feelers(self):
        '''Creates the antenna utilized by WallAvoidance
        '''
        # NOTE: Used <transformation.create_whiskers>, instead of
        # implementation in java code version. Test first.
        self._feelers.clear()
        vehicle = self._vehicle
        facing = Vector2D(**vehicle.position).add(
                Vector2D(**vehicle.heading)
                    .mul(self._wall_detection_feeler_length)
                )
        origin = vehicle.position
        self._feelers.extend(create_whiskers(
                    whisker_count=3,
                    whisker_length=1,
                    fov=math.pi / 2.0,
                    facing=facing,
                    origin=origin))

    def _is_on(self, behavior):
        return (self._flags & behavior) == behavior

    def flee_on(self):
        self._flags |= BehaviorType.FLEE

    def seek_on(self):
        self._flags |= BehaviorType.SEEK

    def arrive_on(self):
        self._flags |= BehaviorType.ARRIVE

    def wander_on(self):
        self._flags |= BehaviorType.WANDER

    def pursuit_on(self, vehicle):
        self._flags |= BehaviorType.PURSUIT
        self._target_agent_1 = vehicle

    def evade_on(self, vehicle):
        self._flags |= BehaviorType.EVADE
        self._target_agent_1 = vehicle

    def cohesion_on(self):
        self._flags |= BehaviorType.COHESION

    def separation_on(self):
        self._flags |= BehaviorType.SEPARATION

    def alignment_on(self):
        self._flags |= BehaviorType.ALIGNMENT

    def obstacle_avoidance_on(self):
        self._flags |= BehaviorType.OBSTACLE_AVOIDANCE

    def wall_avoidance_on(self):
        self._flags |= BehaviorType.WALL_AVOIDANCE

    def follow_path_on(self):
        self._flags |= BehaviorType.FOLLOW_PATH

    def interpose_on(self, vehicle_1, vehicle_2):
        self._flags |= BehaviorType.INTERPOSE
        self._target_agent_1 = vehicle_1
        self._target_agent_2 = vehicle_2

    def hide_on(self, vehicle):
        self._flags |= BehaviorType.HIDE
        self._target_agent_1 = vehicle

    def offset_pursuit_on(self, vehicle, offset):
        self._flags |= BehaviorType.OFFSET_PURSUIT
        self._offset = offset
        self._target_agent_1 = vehicle

    def flocking_on(self):
        self.cohesion_on()
        self.alignment_on()
        self.separation_on()
        self.wander_on()

    def flee_off(self):
        behavior_type = BehaviorType.FLEE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def seek_off(self):
        behavior_type = BehaviorType.SEEK
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def arrive_off(self):
        behavior_type = BehaviorType.ARRIVE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def wander_off(self):
        behavior_type = BehaviorType.WANDER
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def pursuit_off(self):
        behavior_type = BehaviorType.PURSUIT
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def evade_off(self):
        behavior_type = BehaviorType.EVADE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def cohesion_off(self):
        behavior_type = BehaviorType.COHESION
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def separation_off(self):
        behavior_type = BehaviorType.SEPARATION
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def alignment_off(self):
        behavior_type = BehaviorType.ALIGNMENT
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def obstacle_avoidance_off(self):
        behavior_type = BehaviorType.OBSTACLE_AVOIDANCE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def wall_avoidance_off(self):
        behavior_type = BehaviorType.WALL_AVOIDANCE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def follow_path_off(self):
        behavior_type = BehaviorType.FOLLOW_PATH
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def interpose_off(self):
        behavior_type = BehaviorType.INTERPOSE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def hide_off(self):
        behavior_type = BehaviorType.HIDE
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def offset_pursuit_off(self):
        behavior_type = BehaviorType.OFFSET_PURSUIT
        if behavior_type.is_on(self._flags):
            self._flags ^= behavior_type

    def flocking_off(self):
        self.cohesion_off()
        self.alignment_off()
        self.separation_off()
        self.wander_off()

    def is_flee_on(self):
        return BehaviorType.FLEE.is_on(self._flags)

    def is_seek_on(self):
        return BehaviorType.SEEK.is_on(self._flags)

    def is_arrive_on(self):
        return BehaviorType.ARRIVE.is_on(self._flags)

    def is_wander_on(self):
        return BehaviorType.WANDER.is_on(self._flags)

    def is_pursuit_on(self):
        return BehaviorType.PURSUIT.is_on(self._flags)

    def is_evade_on(self):
        return BehaviorType.EVADE.is_on(self._flags)

    def is_cohesion_on(self):
        return BehaviorType.COHESION.is_on(self._flags)

    def is_separation_on(self):
        return BehaviorType.SEPARATION.is_on(self._flags)

    def is_alignment_on(self):
        return BehaviorType.ALIGNMENT.is_on(self._flags)

    def is_obstacle_avoidance_on(self):
        return BehaviorType.OBSTACLE_AVOIDANCE.is_on(self._flags)

    def is_wall_avoidance_on(self):
        return BehaviorType.WALL_AVOIDANCE.is_on(self._flags)

    def is_follow_path_on(self):
        return BehaviorType.FOLLOW_PATH.is_on(self._flags)

    def is_interpose_on(self):
        return BehaviorType.INTERPOSE.is_on(self._flags)

    def is_hide_on(self):
        return BehaviorType.HIDE.is_on(self._flags)

    def is_offset_pursuit_on(self):
        return BehaviorType.OFFSET_PURSUIT.is_on(self._flags)

    def set_target(self, target):
        self._target.set(target)

    def set_target_agent_1(self, agent):
        self._target_agent_1 = agent

    def set_target_agent_2(self, agent):
        self._target_agent_2 = agent

    def set_offset(self, offset):
        self._offset = offset

    def get_offset(self):
        return self._offset

    def set_path(self, new_path):
        self._path.path = new_path

    def create_random_path(self, num_waypoints=None, min_x=None, min_y=None,
            max_x=None, max_y=None):
        self._path.create_random_path(num_waypoints, min_x, min_y, max_x,
                max_y)

    def get_force(self):
        return self._steering_force

    def toggle_space_partition(self):
        self._cell_space_on = not self._cell_space_on

    def is_space_partitioning_on(self):
        return self._cell_space_on

    def set_summing_method(self, summing_method):
        assert isinstance(summing_method, SummingMethod)
        self._summing_method = summing_method

    def get_detection_box_length(self):
        return self._detection_box_length

    def get_feelers(self):
        return deepcopy(self._feelers)

    def get_wander_jitter(self):
        return self._wander_jitter

    def get_wander_distance(self):
        return self._wander_distance

    def get_wander_radius(self):
        return self._wander_radius

    def get_separation_weight(self):
        return self._weight_separation

    def get_alignment_weight(self):
        return self._weight_alignment

    def get_cohesion_weight(self):
        return self._weight_cohesion

    target = property(fset=set_target)
    target_agent_1 = property(fset=set_target_agent_1)
    target_agent_2 = property(fset=set_target_agent_2)
    offset = property(get_offset, set_offset)
    path = property(fset=set_path)
    force = property(get_force)
    detection_box_length = property(get_detection_box_length)
    feelers = property(get_feelers)
    wander_jitter = property(get_wander_jitter)
    wander_distance = property(get_wander_distance)
    wander_radius = property(get_wander_radius)
    separation_weight = property(get_separation_weight)
    alignment_weight = property(get_alignment_weight)
    cohesion_weight = property(get_cohesion_weight)

# =============================================================================
#                             STEERING BEHAVIORS
# =============================================================================

    def _seek(self, target_pos):
        '''SteeringBehaviors._seek(self, target_pos) -> Vector2D

        Given a target, this behavior returns a steering force which will
            direct the agent towards the target.

        Keyword arguments:
        target_pos -- Vector2D
        '''
        desired_velocity = (Vector2D().set(target_pos)
                .sub(self._vehicle.position)
                .normalize()
                .mul(self._vehicle.max_speed))
        return desired_velocity.sub(self._vehicle.velocity)



    def _flee(self, target_pos):
        '''SteeringBehaviors._flee(self, target_pos) -> Vector2D

        Does the opposite of Seek. Steers away from the target.

        Keyword arguments:
        target_pos -- Vector2D
        '''
        # Only flee if the target is within 'panic distance'.
        # Work in distance squared space.
        # if self._vehicle.position.distance_sq(target_pos) > 0:
        #     return Vector2D()

        desired_velocity = (Vector2D().set(self._vehicle.position)
                .sub(target_pos)
                .normalize()
                .mul(self._vehicle.max_speed))
        return desired_velocity.sub(self._vehicle.velocity)



    def _arrive(self, target_pos, deceleration):
        '''SteeringBehaviors._arrive(self, target_pos, deceleration)
            -> Vector2D

        This behavior is similar to seek but it attempts to arrive at the
            target with a zero velocity.

        Keyword arguments:
        target_pos -- Vector2D
        deceleration -- Deceleration
        '''
        to_target = Vector2D().set(target_pos).sub(self._vehicle.position)
        distance = to_target.length

        if distance > 0:
            # because Deceleration is enumerated as an int, this value is
            # required to provide fine tweaking of the deceleration..
            deceleration_tweaker = 0.3

            # calculate the speed required to reach the target given the
            # desired deceleration
            speed = distance / deceleration * deceleration_tweaker

            # make sure the velocity does not exceed the max
            speed = min(speed, self._vehicle.max_speed)

            # from here proceed just like Seek except we don't need to
            # normalize the ToTarget vector because we have already gone
            # to the trouble of calculating its length: dist.
            desired_velocity = to_target.mul(speed / distance)

            return desired_velocity.sub(self._vehicle.velocity)

        return Vector2D()



    def _pursuit(self, evader):
        '''SteeringBehaviors._pursuit(self, evader) -> Vector2D

        This behavior creates a force that steers the agent towards the
            evader.

        Keyword arguments:
        evader -- Vehicle
        '''
        # if the evader is ahead and facing the agent then we can just seek
        # for the evader's current position.
        to_evader = evader.position - self._vehicle.position
        relative_heading = self._vehicle.heading * evader.heading

        to_evader = Vector2D().set(evader.position).sub(self._vehicle.position)
        relative_heading = self._vehicle.heading.dot(evader.heading)

        if (to_evader.dot(self._vehicle.heading) > 0 and
                relative_heading < -0.95): # acos(0.95) = 18 deg
            return self._seek(evader.position)

        # Not considered ahead so we predict where the evader will be.

        # the lookahead time is propotional to the distance between the evader
        # and the pursuer; and is inversely proportional to the sum of the
        # agent's velocities
        look_ahead_time = (to_evader.length /
                (self._vehicle.max_speed + evader.speed))

        # now seek to the predicted future position of the evader
        return self._seek(Vector2D(*evader.position)
                .add(Vector2D(*evader.velocity).mul(look_ahead_time)))



    def _evade(self, pursuer):
        '''SteeringBehaviors._wander(self) -> Vector2D

        Similar to pursuit except the agent Flees from the estimated future
            position of the pursuer.

        Keyword arguments:
        pursuer -- Vehicle
        '''
        # Not necessary to include the check for facing direction this time
        to_pursuer = (Vector2D().set(pursuer.position)
                .sub(self._vehicle.position))

        # uncomment the following two lines to have Evade only consider
        # pursuers within a 'threat range'
        threat_range = 100.0
        if to_pursuer.length_sq > threat_range * threat_range:
            return Vector2D()

        # the lookahead time is propotional to the distance between the pursuer
        # and the pursuer; and is inversely proportional to the sum of the
        # agents' velocities
        look_ahead_time = (to_pursuer.length() /
                (self._vehicle.max_speed + pursuer.speed))

        # now flee away from predicted future position of the pursuer
        return self._flee(Vector2D(*pursuer.position)
                .add(Vector2D(pursuer.velocity).mul(look_ahead_time)))



    def _wander(self):
        '''SteeringBehaviors._wander(self) -> Vector2D

        This behavior makes the agent wander about randomly.
        '''
        # this behavior is dependent on the update rate, so this line must
        # be included when using time independent framerate.
        jitter_this_time_slice = (self._wander_jitter *
                self._vehicle.time_elapsed)

        rand_x = random_clamped() * jitter_this_time_slice
        rand_y = random_clamped() * jitter_this_time_slice

        # first, add a small random vector to the target's position
        self._wander_target.add((rand_x, rand_y))

        # reproject this new vector back on to a unit circle
        self._wander_target.normalize()

        # increase the length of the vector to the same as the radius
        # of the wander circle
        self._wander_target.mul(self._wander_radius)

        # move the target into a position WanderDist in front of the agent
        target_local = (Vector2D(*self._wander_target)
                .add((self._wander_distance, 0)))

        # project the target into world space
        target_world = point_to_world_space(target_local,
            self._vehicle.heading,
            self._vehicle.side,
            self._vehicle.position)

        # and steer towards it
        return target_world.sub(self._vehicle.position)



    def _obstacle_avoidance(self, obstacles):
        '''SteeringBehaviors._obstacle_avoidance(self, obstacles) -> Vector2D

        Given a vector of obstacles, this method returns a steering force
            that will prevent the agent colliding with the closest obstacle.

        Keyword arguments:
        obstacles -- collections.abc.MutableSequence<BaseGameEntity>
        '''
        # the detection box length is proportional to the agent's velocity
        self._detection_box_length = (Cfg.MIN_DETECTION_BOX_LENGTH +
                (self._vehicle.speed / self._vehicle.max_speed) *
                Cfg.MIN_DETECTION_BOX_LENGTH)

        # tag all obstacles within range of the box for processing
        self._vehicle.world.tag_obstacles_within_view_range(self._vehicle,
                self._detection_box_length)

        # this will keep track of the closest intersecting obstacle (CIB)
        closest_intersecting_obstacle = None

        # this will be used to track the distance to the CIB
        dist_to_closest_ip = float_info.max

        # this will record the transformed local coordinates of the CIB
        local_pos_of_closest_obstacle = Vector2D()

        for current_obstacle in obstacles:
            # if the obstacle has been tagged within range proceed
            if current_obstacle.is_tagged():

                # calculate this obstacle's position in local space
                local_pos = point_to_local_space(
                        current_obstacle.position,
                        self._vehicle.heading,
                        self._vehicle.side,
                        self._vehicle.position
                        )

                # if the local position has a negative x value then it must lay
                # behind the agent. (in which case it can be ignored)
                if local_pos.x >= 0:

                    # if the distance from the x axis to the object's position
                    # is less than its radius + half the width of the
                    # detection box then there is a potential intersection.
                    expanded_radius = (current_obstacle.bounding_radius +
                            self._vehicle.bounding_radius)
                    if abs(local_pos.y) < expanded_radius:

                        # now to do a line/circle intersection test. The
                        # center of the circle is represented by (cX, cY). The
                        # intersection points are given by the formula x = cX
                        # +/-sqrt(r^2-cY^2) for y=0. We only need to look at
                        # the smallest positive value of x because that will
                        # be the closest point of intersection.
                        circle_center = Vector2D(*local_pos)

                        # we only need to calculate the sqrt part of the above
                        # equation once
                        sqrt_part = (expanded_radius**2 -
                                circle_center.y**2)**0.5
                        ip = circle_center.x - sqrt_part
                        if ip <= 0.0:
                            ip = circle_center.x + sqrt_part

                        # test to see if this is the closest so far. If it is
                        # keep a record of the obstacle and its local
                        # coordinates
                        if ip < dist_to_closest_ip:
                            dist_to_closest_ip = ip
                            closest_intersecting_obstacle = current_obstacle
                            local_pos_of_closest_obstacle.set(local_pos)

        # if we have found an intersecting obstacle, calculate a steering
        # force away from it
        steering_force = Vector2D()
        if closest_intersecting_obstacle is not None:

            # the closer the agent is to an object, the stronger the
            # steering force should be
            multiplier = 1.0 + (self._detection_box_length -
                    local_pos_of_closest_obstacle.x /
                    self._detection_box_length)

            # calculate the lateral force
            steering_force.y = (
                    closest_intersecting_obstacle.bounding_radius -
                    local_pos_of_closest_obstacle.y) * multiplier

            # apply a braking force proportional to the obstacles distance
            # from the vehicle.
            BREAKING_WEIGHT = 0.2
            steering_force.y = (
                    closest_intersecting_obstacle.bounding_radius -
                    local_pos_of_closest_obstacle.x) * BREAKING_WEIGHT

        # finally, convert the steering vector from local to world space
        return vector_to_world_space(steering_force, self._vehicle.heading,
                self._vehicle.side)



    def _wall_avoidance(self, walls):
        '''SteeringBehaviors._wall_avoidance(self, walls) -> Vector2D

        This returns a steering force that will keep the agent away from any
            walls it may encounter.

        Keyword arguments:
        walls -- collections.abc.Sequence<Wall2D>
        '''
        # the feelers are contained in a list self._feelers
        self._create_feelers()
        dist_to_closest_ip = float_info.max

        # this will hold an index into the vector of walls
        closest_wall = None
        steering_force = Vector2D()
        closest_ip = Vector2D()

        # examine each feeler in turn
        for feeler in self._feelers:

            # run through each wall checking for any intersection points
            for wall in walls:
                # lines = line_intersection_2d(
                #         self._vehicle.position,
                #         feeler,
                #         wall.from_pt,
                #         wall.to_pt)
                intersectinfo = {}
                has_ip = LineSegment2D.line_intersection(
                        self._vehicle.position,
                        feeler,
                        intersectinfo
                        )
                if has_ip:

                    # is this the closest found so far? If so keep a record
                    if intersectinfo['dist_to_ip'] < dist_to_closest_ip:
                        closest_wall = wall
                        closest_ip.set(intersectinfo['ip'])

            # if an intersection point has been detected, calculate a force
            # that will direct the agent away
            if closest_wall is not None:

                # calculate by what distance the projected position of the
                # agent will overshoot the wall
                overshoot = Vector2D(*feeler).sub(closest_ip)

                # create a force in the direction of the wall normal, with a
                # magnitude of the overshoot
                steering_force.set(Vector2D(*closest_wall.normal)
                        .mul(overshoot.length))

        return steering_force



    def _separation(self, neighbors):
        '''SteeringBehaviors._separation(self, neighbors) -> Vector2D

        This calculates a force repelling from the other neighbors.

        Keyword arguments:
        neighbors -- collections.abc.Sequence<Wall2D>
        '''
        steering_force = Vector2D()
        for neighbor in neighbors:

            # make sure this agent isn't included in the calculations and that
            # the agent being examined is close enough. ***also make sure it
            # doesn't include the evade target ***
            if (neighbor != self._vehicle and
                        neighbor.is_tagged() and
                    neighbor != self._target_agent_1):
                to_agent = (Vector2D(*self._vehicle.position)
                        .sub(neighbor.position))

                # scale the force inversely proportional to the agents
                # distance from its neighbor.
                steering_force.add(Vector2D(to_agent).normalize()
                            .div(to_agent.length))
        return steering_force



    def _alignment(self, neighbors):
        '''SteeringBehaviors._alignment(self, neighbors) -> Vector2D

        Returns a force that attempts to align this agents heading with that
            of its neighbors.

        Keyword arguments:
        neighbors -- collections.abc.Sequence<Wall2D>
        '''
        # used to record the average heading of the neighbors
        average_heading = Vector2D()

        # used to count the number of vehicles in the neighborhood
        neighbor_count = 0

        # iterate through all the tagged vehicles and sum their heading
        # vectors
        for neighbor in neighbors:

            # make sure *this* agent isn't included in the calculations and
            # that the agent being examined  is close enough ***also make sure
            # it doesn't include any evade target ***
            if (neighbor != self._vehicle and
                        neighbor.is_tagged() and
                    neighbor != self._target_agent_1):
                average_heading.add(neighbor.heading)
                neighbor_count += 1

        # if the neighborhood contained one or more vehicles, average their
        # heading vectors.
        if neighbor_count > 0:
            average_heading.div(neighbor_count).sub(self._vehicle.heading)

        return average_heading



    def _cohesion(self, neighbors):
        '''SteeringBehaviors._cohesion(self, neighbors) -> Vector2D

        Returns a steering force that attempts to move the agent towards the
            center of mass of the agents in its immediate area.

        Keyword arguments:
        neighbors -- collections.abc.Sequence<Wall2D>
        '''
        # first find the center of mass of all the agents
        center_of_mass = Vector2D()
        neighbor_count = 0

        # iterate through all the tagged vehicles and sum their heading
        # vectors
        for neighbor in neighbors:

            # make sure *this* agent isn't included in the calculations and
            # that the agent being examined  is close enough ***also make sure
            # it doesn't include any evade target ***
            if (neighbor != self._vehicle and
                        neighbor.is_tagged() and
                    neighbor != self._target_agent_1):
                center_of_mass.add(neighbor.position)
                neighbor_count += 1

        steering_force = Vector2D()
        if neighbor_count > 0:

            # the center of mass is the average of the sum of positions
            center_of_mass.div(neighbor_count)

            # now seek towards that position
            steering_force.set(self._seek(center_of_mass))

        # the magnitude of cohesion is usually much larger than separation or
        # allignment so it usually helps to normalize it.
        return steering_force.normalize()



    # NOTE: The next three behaviors are the same as the above three, except
    #   that they use a cell-space partition to find the neighbors



    def _separation_plus(self, neighbors):
        '''SteeringBehaviors._separation_plus(self, neighbors) -> Vector2D

        [USES SPACIAL PARTITIONING]
        This calculates a force repelling from the other neighbors.

        Keyword arguments:
        neighbors -- collections.abc.Sequence<Wall2D>
        '''
        # iterate through the neighbors and sum up all the position vectors
        steering_force = Vector2D()
        for neighbor in self._vehicle.world.cell_space:

            # make sure this agent isn't included in the calculations and that
            # the agent being examined is close enough
            if neighbor != self._vehicle:
                to_agent = (Vector2D(*self._vehicle.position)
                        .sub(neighbor.position))

                # scale the force inversely proportional to the agents
                # distance from its neighbor.
                steering_force.add(Vector2D(*to_agent).normalize()
                        .div(to_agent.length))
        return steering_force



    def _alignment_plus(self, neighbors):
        '''SteeringBehaviors._alignment_plus(self, neighbors) -> Vector2D

        [USES SPACIAL PARTITIONING]
        Returns a force that attempts to align this agents heading with that
            of its neighbors.

        Keyword arguments:
        neighbors -- collections.abc.Sequence<Wall2D>
        '''
        # this will record the average heading of the neighbors
        average_heading = Vector2D()

        # This count the number of vehicles in the neighborhood
        neighbor_count = 0

        # iterate through the neighbors and sum up all the position vectors
        for neighbor in self._vehicle.world.cell_space:

            # make sure *this* agent isn't included in the calculations and
            # that the agent being examined  is close enough
            if neighbor != self._vehicle:
                average_heading.add(neighbor.heading)
                neighbor_count += 1

        # if the neighborhood contained one or more vehicles, average their
        # heading vectors
        if neighbor_count > 0:
            average_heading.div(neighbor_count).sub(self._vehicle.heading)

        return average_heading



    def _cohesion_plus(self, neighbors):
        '''SteeringBehaviors._cohesion_plus(self, neighbors) -> Vector2D

        [USES SPACIAL PARTITIONING]
        Returns a steering force that attempts to move the agent towards the
            center of mass of the agents in its immediate area.

        Keyword arguments:
        neighbors -- collections.abc.Sequence<Wall2D>
        '''
        # first find the center of mass of all the agents
        center_of_mass = Vector2D()
        steering_force = Vector2D()
        neighbor_count = 0

        # iterate through the neighbors and sum up all the position vectors
        for neighbor in self._vehicle.world.cell_space:

            # make sure *this* agent isn't included in the calculations and
            # that the agent being examined is close enough
            if neighbor != self._vehicle:
                center_of_mass.add(neighbor.position)
                neighbor_count += 1

        if neighbor_count > 0:

            # the center of mass is the average of the sum of positions
            center_of_mass.div(neighbor_count)

            # now seek towards that position
            steering_force.set(self._seek(center_of_mass))

        # the magnitude of cohesion is usually much larger than separation or
        # allignment so it usually helps to normalize it.
        return steering_force.normalize()



    def _interpose(self, agent_a, agent_b):
        '''SteeringBehaviors._interpose(self, agent_a, agent_b) -> Vector2D

        Given two agents, this method returns a force that attempts to
            position the vehicle between them

        Keyword arguments:
        agent_a -- Vehicle
        agent_b -- Vehicle
        '''
        # first we need to figure out where the two agents are going to be at
        # time T in the future. This is approximated by determining the time
        # taken to reach the mid way point at the current time at at max speed.
        mid_point = (Vector2D(*agent_a.position).add(agent_b.position)
                .div(2.0))
        time_to_reach_midpoint = (self._vehicle.position.distance(mid_point) /
                self._vehicle.max_speed)

        # now we have T, we assume that agent A and agent B will continue on a
        # straight trajectory and extrapolate to get their future positions
        a_pos = (Vector2D(*agent_a.velocity).mul(time_to_reach_midpoint)
                .add(agent_a.position))
        b_pos = (Vector2D(*agent_b.velocity).mul(time_to_reach_midpoint)
                .add(agent_b.position))

        # calculate the mid point of these predicted positions
        mid_point = a_pos.add(b_pos).div(2.0)

        # then steer to Arrive at it
        return self._arrive(mid_point, Deceleration.FAST)



    def _hide(self, hunter, obstacles):
        '''SteeringBehaviors._hide(self, hunter, obstacles) -> Vector2D

        Find the best hiding spot for this agent. Falls back to evading if
            none is found.

        Keyword arguments:
        hunter -- Vehicle
        obstacles -- collections.abc.Sequence<BaseGameEntity>
        '''
        dist_to_closest = float_info.max
        best_hiding_spot = Vector2D()
        # closest = None

        for current_obstacle in obstacles:

            # calculate the position of the hiding spot for this obstacle
            hiding_spot = self._get_hiding_position(
                    current_obstacle.position,
                    current_obstacle.bounding_radius,
                    hunter.position
                    )

            # work in distance-squared space to find the closest hiding
            # spot to the age
            dist_sq = hiding_spot.distance_sq(self._vehicle.position)
            if dist_sq < dist_to_closest:
                dist_to_closest = dist_sq
                best_hiding_spot = hiding_spot
                # closest = current_obstacle

        # if no suitable obstacles found then Evade the hunter
        if dist_to_closest == float_info.max:
            return self._evade(hunter)

        # else use Arrive on the hiding spot
        return self._arrive(best_hiding_spot, Deceleration.FAST)




    def _get_hiding_position(self, pos_ob, radius_ob, pos_hunter):
        '''SteeringBehaviors._get_hiding_position(self, pos_ob, radius_ob,
                pos_hunter) -> Vector2D

        Given the position of a hunter, and the position and radius of
            an obstacle, this method calculates a position
            DistanceFromBoundary away from its bounding radius and directly
            opposite the hunter.

        Keyword arguments:
        pos_ob -- Vector2D
        radius -- numers.Real
        pos_hunter -- Vector2D
        '''
        # calculate how far away the agent is to be from the chosen obstacle's
        # bounding radius
        distance_from_boundary = 30.0
        dist_away = radius_ob + distance_from_boundary

        # calculate the heading toward the object from the hunter
        to_ob = Vector2D(*pos_ob).sub(pos_hunter).normalize()

        # scale it to size and add to the obstacles position to get
        # the hiding spot.
        return to_ob.mul(dist_away).add(pos_ob)



    def _follow_path(self):
        '''SteeringBehaviors._follow_path(self) -> Vector2D

        Given a series of Vector2Ds, this method produces a force that will
            move the agent along the waypoints in order. The agent uses the
            'Seek' behavior to move to the next waypoint - unless it is the
            last waypoint, in which case it 'Arrives'.
        '''
        # move to next target if close enough to current target (working in
        # distance squared space)
        if (self._path.current_waypoint.distance_sq(self._vehicle.position) <
                self._waypoint_seek_distance_sq):
            self._path.set_next_waypoint()

        if self._path.is_finished():
            return self._seek(self._path.current_waypoint)

        return self._arrive(self._path.current_waypoint, Deceleration.NORMAL)



    def _offset_pursuit(self, leader, offset):
        '''SteeringBehaviors._offset_pursuit(leader, offset) -> Vector2D

        Produces a steering force that keeps a vehicle at a specified offset
            form a leader vehicle.

        Keyword arguments:
        leader -- Vehicle
        offset -- Vector2D
        '''
        # calculate the offset's position in world space
        world_offset_pos = point_to_world_space(
                point=offset,
                agent_heading=leader.heading,
                agent_side=leader.side,
                agent_position=leader.position)

        to_offset = Vector2D(*world_offset_pos).sub(self._vehicle.pos)

        # the lookahead time is propotional to the distance between the leader
        # and the pursuer; and is inversely proportional to the sum of both
        # agent's velocities
        look_ahead_time = (to_offset.length() /
                (self._vehicle.max_speed + leader.speed))

        # now Arrive at the predicted future position of the offset
        arrive_at = (Vector2D(*leader.velocity).mul(look_ahead_time)
                .add(world_offset_pos))
        return self._arrive(arrive_at, Deceleration.FAST)



    def calculate(self):
        '''SteeringBehaviors.calculate(self) -> Vector2D

        Calculates the accumulated steering force according to the method set
            in self._summing_method.
        '''
        self._steering_force.zero()

        if not self.is_space_partitioning_on():
            if (self._is_on(BehaviorType.SEPARATION) or
                    self._is_on(BehaviorType.ALIGNMENT) or
                    self._is_on(BehaviorType.COHESION)):
                self._vehicle.world.tag_vehicles_within_view_range(
                        self._vehicle,
                        self._view_distance
                        )
        else:
            if (self._is_on(BehaviorType.SEPARATION) or
                    self._is_on(BehaviorType.ALIGNMENT) or
                    self._is_on(BehaviorType.COHESION)):
                self._vehicle.world.cell_space.calculate_neighbors(
                        self._vehicle.position,
                        self._view_distance
                        )

        if self._summing_method == SummingMethod.WEIGHTED_AVERAGE:
            self._steering_force = self._calculate_weighted_sum()
        elif self._summing_method == SummingMethod.PRIORITIZED:
            self._steering_force = self._calculate_prioritized()
        elif self._summing_method == SummingMethod.DITHERED:
            self._steering_force = self._calculate_dithered()
        else:
            self._steering_force = Vector2D()

        return self._steering_force



    def forward_component(self):
        '''SteeringBehaviors.forward_component(self) -> Vector2D

        Returns the forward component of the steering force.
        '''
        return self._vehicle.heading * self._steering_force



    def side_component(self):
        '''SteeringBehaviors.side_component(self) -> Vector2D

        Returns the side component of the steering force
        '''
        return self._vehicle.side * self._steering_force



    def _calculate_prioritized(self):
        '''SteeringBehaviors.calculate_prioritized(self) -> Vector2D

        This method calls each active steering behavior in order of priority
            and acumulates their forces until the max steering force magnitude
            is reached, at which time the function returns the steering force
            accumulated to that  point
        '''
        force = Vector2D()

        if self._is_on(BehaviorType.WALL_AVOIDANCE):
            force = (self._wall_avoidance(self._vehicle.world.walls) *
                    self._weight_wall_avoidance)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.OBSTACLE_AVOIDANCE):
            force = self._obstacle_avoidance(self._vehicle.world.obstacles,
                    self._weight_obstacle_avoidance)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.EVADE):
            if self._target_agent_1 is None:
                raise ValueError('Evade target not assigned')

            force = self._evade(self._target_agent_1) * self._weight_evade
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.FLEE):
            force = (self._flee(self._vehicle.world.crosshair) *
                    self._weight_flee)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if not self.is_space_partitioning_on():
            if self._is_on(BehaviorType.SEPARATION):
                force = (self._separation(self._vehicle.world.agents) *
                        self._weight_separation)
                has_force_left, self._steering_force = self._accumulate_force(
                        self._steering_force, force).values()
                if not has_force_left:
                    return self._steering_force

            if self._is_on(BehaviorType.ALIGNMENT):
                force = (self._alignment(self._vehicle.world.agents) *
                        self._weight_alignment)
                has_force_left, self._steering_force = self._accumulate_force(
                        self._steering_force, force).values()
                if not has_force_left:
                    return self._steering_force

            if self._is_on(BehaviorType.COHESION):
                force = (self._cohesion(self._vehicle.world.agents) *
                        self._weight_cohesion)
                has_force_left, self._steering_force = self._accumulate_force(
                        self._steering_force, force).values()
                if not has_force_left:
                    return self._steering_force
        else:
            if self._is_on(BehaviorType.SEPARATION):
                force = (self._separation_plus(self._vehicle.world.agents) *
                        self._weight_separation)
                has_force_left, self._steering_force = self._accumulate_force(
                        self._steering_force, force).values()
                if not has_force_left:
                    return self._steering_force

            if self._is_on(BehaviorType.ALIGNMENT):
                force = (self._alignment_plus(self._vehicle.world.agents) *
                        self._weight_alignment)
                has_force_left, self._steering_force = self._accumulate_force(
                        self._steering_force, force).values()
                if not has_force_left:
                    return self._steering_force

            if self._is_on(BehaviorType.COHESION):
                force = (self._cohesion_plus(self._vehicle.world.agents) *
                        self._weight_cohesion)
                has_force_left, self._steering_force = self._accumulate_force(
                        self._steering_force, force).values()
                if not has_force_left:
                    return self._steering_force

        if self._is_on(BehaviorType.SEEK):
            force = (self._seek(self._vehicle.world.crosshair) *
                    self._weight_seek)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.ARRIVE):
            force = (self._arrive(self._vehicle.world.crosshair,
                        self._deceleration) *
                    self._weight_arrive)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.WANDER):
            force = self._wander() * self._weight_wander
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.PURSUIT):
            assert self._target_agent_1 is not None, ('Pursuit target not ' +
                    'assigned')
            force = (self._pursuit(self._target_agent_1) *
                    self._weight_pursuit)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.OFFSET_PURSUIT):
            assert self._target_agent_1 is not None, ('Pursuit target not ' +
                    'assigned')
            assert not self._offset.is_zero(), 'No offset assigned'

            force = (self._offset_pursuit(self._target_agent_1, self._offset) *
                    self._weight_offset_pursuit)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.INTERPOSE):
            assert (self._target_agent_1 is not None and
                    self._target_agent_2 is not None), ('Interpose agents ' +
                        'not assigned')
            force = (self._interpose(self._target_agent_1,
                        self._target_agent_2) *
                    self._weight_interpose)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.HIDE):
            assert self._target_agent_1 is not None, ('Hide target not ' +
                    'asssigned')
            force = (self._hide(self._target_agent_1,
                        self._vehicle.world.obstacles) *
                    self._weight_hide)
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        if self._is_on(BehaviorType.FOLLOW_PATH):
            force = self._follow_path() * self._weight_follow_path
            has_force_left, self._steering_force = self._accumulate_force(
                    self._steering_force, force).values()
            if not has_force_left:
                return self._steering_force

        return self._steering_force



    def _calculate_weighted_sum(self):
        '''SteeringBehaviors.calculate_weighted_sum(self) -> Vector2D

        This simply sums up all the active behaviors X their weights and
            truncates the result to the max available steering force before
            returning.
        '''
        if self._is_on(BehaviorType.WALL_AVOIDANCE):
            self._steering_force += (self._wall_avoidance(
                        self._vehicle.world.walls) *
                    self._weight_wall_avoidance)

        if self._is_on(BehaviorType.OBSTACLE_AVOIDANCE):
            self._steering_force += (self._obstacle_avoidance(
                        self._vehicle.world.obstacles) *
                    self._weight_obstacle_avoidance)

        if self._is_on(BehaviorType.EVADE):
            if self._target_agent_1 is None:
                raise ValueError('Evade target not assigned')

            self._steering_force += (self._evade(
                        self._target_agent_1) *
                    self._weight_evade)

        if self._is_on(BehaviorType.FLEE):
            self._steering_force += (self._flee(
                        self._vehicle.world.crosshair) *
                    self._weight_flee)

        if not self.is_space_partitioning_on():
            if self._is_on(BehaviorType.SEPARATION):
                self._steering_force += (self._separation(
                        self._vehicle.world.agents) *
                    self._weight_separation)

            if self._is_on(BehaviorType.ALIGNMENT):
                self._steering_force += (self._alignment(
                        self._vehicle.world.agents) *
                    self._weight_alignment)

            if self._is_on(BehaviorType.COHESION):
                self._steering_force += (self._cohesion(
                        self._vehicle.world.agents) *
                    self._weight_cohesion)
        else:
            if self._is_on(BehaviorType.SEPARATION):
                self._steering_force += (self._separation_plus(
                        self._vehicle.world.agents) *
                    self._weight_separation)

            if self._is_on(BehaviorType.ALIGNMENT):
                self._steering_force += (self._alignment_plus(
                        self._vehicle.world.agents) *
                    self._weight_alignment)

            if self._is_on(BehaviorType.COHESION):
                self._steering_force += (self._cohesion_plus(
                        self._vehicle.world.agents) *
                    self._weight_cohesion)

        if self._is_on(BehaviorType.SEEK):
            self._steering_force += (self._seek(
                        self._vehicle.world.crosshair) *
                    self._weight_seek)

        if self._is_on(BehaviorType.ARRIVE):
            self._steering_force += (self._arrive(
                        self._vehicle.world.crosshair, self._deceleration) *
                    self._weight_arrive)

        if self._is_on(BehaviorType.WANDER):
            self._steering_force += self._wander() * self._weight_wander

        if self._is_on(BehaviorType.PURSUIT):
            assert self._target_agent_1 is not None, ('Pursuit target not ' +
                    'assigned')
            self._steering_force += (self._pursuit(self._target_agent_1) *
                    self._weight_pursuit)

        if self._is_on(BehaviorType.OFFSET_PURSUIT):
            assert self._target_agent_1 is not None, ('Pursuit target not ' +
                    'assigned')
            assert not self._offset.is_zero(), 'No offset assigned'

            self._steering_force += (self._offset_pursuit(
                        self._target_agent_1, self._offset) *
                    self._weight_offset_pursuit)

        if self._is_on(BehaviorType.INTERPOSE):
            assert (self._target_agent_1 is not None and
                    self._target_agent_2 is not None), ('Interpose agents ' +
                        'not assigned')
            self._steering_force += (self._interpose(
                        self._target_agent_1, self._target_agent_2) *
                    self._weight_interpose)

        if self._is_on(BehaviorType.HIDE):
            assert self._target_agent_1 is not None, ('Hide target not ' +
                    'asssigned')
            self._steering_force += (self._hide(self._target_agent_1,
                        self._vehicle.world.obstacles) *
                    self._weight_hide)

        if self._is_on(BehaviorType.FOLLOW_PATH):
            self._steering_force += (self._follow_path() *
                    self._weight_follow_path)

        return self._steering_force



    def _calculate_dithered(self):
        '''SteeringBehaviors._calculate_dithered(self) -> Vector2D

        This method sums up the active behaviors by assigning a probability of
            being calculated to each behavior. It then tests the first
            priority to see if it should be calcukated this simulation-step.
            If so, it calculates the steering force resulting from this
            behavior. If it is more than zero it returns the force. If zero,
            or if the behavior is skipped it continues onto the next priority,
            and so on.

        NOTE: Not all of the behaviors have been implemented in this method,
            just a few, so you get the general idea.
        '''
        self._steering_force.zero = Vector2D()

        if (self._is_on(BehaviorType.WALL_AVOIDANCE) and
                random() < Cfg.probabilities.WALL_AVOIDANCE):
            self._steering_force += (
                    self._wall_avoidance(self._vehicle.world.walls) *
                    (self._weight_wall_avoidance /
                        Cfg.probabilities.WALL_AVOIDANCE)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        if (self._is_on(BehaviorType.OBSTACLE_AVOIDANCE) and
                random() < Cfg.probabilities.OBSTACLE_AVOIDANCE):
            self._steering_force += (
                    self._obstacle_avoidance(self._vehicle.world.obstacles) *
                    (self._weight_obstacle_avoidance /
                        Cfg.probabilities.OBSTACLE_AVOIDANCE)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        if not self.is_space_partitioning_on():
            if (self._is_on(BehaviorType.SEPARATION) and
                    random() < Cfg.probabilities.SEPARATION):
                self._steering_force += (
                        self._separation(self._vehicle.world.agents) *
                        (self._weight_separation /
                            Cfg.probabilities.SEPARATION)
                        )
                if not self._steering_force.is_zero():
                    self._steering_force.truncate_ip(self._vehicle.max_force)
                    return self._steering_force
        else:
            if (self._is_on(BehaviorType.SEPARATION) and
                    random() < Cfg.probabilities.SEPARATION):
                self._steering_force += (
                        self._separation_plus(self._vehicle.world.agents) *
                        (self._weight_separation /
                            Cfg.probabilities.SEPARATION)
                        )
                if not self._steering_force.is_zero():
                    self._steering_force.truncate_ip(self._vehicle.max_force)
                    return self._steering_force

        if (self._is_on(BehaviorType.FLEE) and
                random() < Cfg.probabilities.FLEE):
            self._steering_force += (
                    self._flee(self._vehicle.world.crosshair) *
                    (self._weight_flee /
                        Cfg.probabilities.FLEE)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        if (self._is_on(BehaviorType.EVADE) and
                random() < Cfg.probabilities.EVADE):
            assert self._target_agent_1 is not None, ('Evade target not ' +
                    'assigned')
            self._steering_force += (
                    self._evade(self._target_agent_1) *
                    (self._weight_evade /
                        Cfg.probabilities.EVADE)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        if not self.is_space_partitioning_on():
            if (self._is_on(BehaviorType.ALIGNMENT) and
                    random() < Cfg.probabilities.ALIGNMENT):
                self._steering_force += (
                        self._alignment(self._vehicle.world.agents) *
                        (self._weight_alignment /
                            Cfg.probabilities.ALIGNMENT)
                        )
                if not self._steering_force.is_zero():
                    self._steering_force.truncate_ip(self._vehicle.max_force)
                    return self._steering_force

            if (self._is_on(BehaviorType.COHESION) and
                    random() < Cfg.probabilities.COHESION):
                self._steering_force += (
                        self._cohesion(self._vehicle.world.agents) *
                        (self._weight_cohesion /
                            Cfg.probabilities.COHESION)
                        )
                if not self._steering_force.is_zero():
                    self._steering_force.truncate_ip(self._vehicle.max_force)
                    return self._steering_force
        else:
            if (self._is_on(BehaviorType.ALIGNMENT) and
                    random() < Cfg.probabilities.ALIGNMENT):
                self._steering_force += (
                        self._alignment_plus(self._vehicle.world.agents) *
                        (self._weight_alignment /
                            Cfg.probabilities.ALIGNMENT)
                        )
                if not self._steering_force.is_zero():
                    self._steering_force.truncate_ip(self._vehicle.max_force)
                    return self._steering_force

            if (self._is_on(BehaviorType.COHESION) and
                    random() < Cfg.probabilities.COHESION):
                self._steering_force += (
                        self._cohesion_plus(self._vehicle.world.agents) *
                        (self._weight_cohesion /
                            Cfg.probabilities.COHESION)
                        )
                if not self._steering_force.is_zero():
                    self._steering_force.truncate_ip(self._vehicle.max_force)
                    return self._steering_force

        if (self._is_on(BehaviorType.WANDER) and
                random() < Cfg.probabilities.WANDER):
            self._steering_force += (
                    self._wander() *
                    (self._weight_wander /
                        Cfg.probabilities.WANDER)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        if (self._is_on(BehaviorType.SEEK) and
                random() < Cfg.probabilities.SEEK):
            self._steering_force += (
                    self._seek(self._vehicle.world.crosshair) *
                    (self._weight_seek /
                        Cfg.probabilities.SEEK)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        if (self._is_on(BehaviorType.ARRIVE) and
                random() < Cfg.probabilities.ARRIVE):
            self._steering_force += (
                    self._arrive(self._vehicle.world.crosshair,
                        self._deceleration) *
                    (self._weight_arrive /
                        Cfg.probabilities.ARRIVE)
                    )
            if not self._steering_force.is_zero():
                self._steering_force.truncate_ip(self._vehicle.max_force)
                return self._steering_force

        return self._steering_force
