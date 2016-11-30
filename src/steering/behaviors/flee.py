from . import SteeringBehaviors
from src.common.math2d.vector2d import Vector2D


#pylint: disable=W0212
class Flee(SteeringBehaviors.Type):
    '''Flee(weight, container) -> Flee

    Does the opposite of Seek. Steers away from the target.

    Keyword arguments:
    weight -- Real
    container -- SteeringBehaviors
    '''

    def __call__(self, target_pos):
        ''' -> Vector2D
        target_pos -- Vector2D
        '''
        # Only flee if the target is within 'panic distance'.
        # Work in distance squared space.
        # if self._vehicle.position.distance_sq(target_pos) > 0:
        #     return Vector2D()
        vehicle = self._container._vehicle
        desired_velocity = (Vector2D(*vehicle.position)
                .sub(target_pos)
                .normalize()
                .mul(vehicle.max_speed))
        return desired_velocity - self._vehicle.velocity
#pylint: enable=W0212
