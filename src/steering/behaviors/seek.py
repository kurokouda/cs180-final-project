from . import SteeringBehaviors
from src.common.math2d.vector2d import Vector2D


#pylint: disable=W0212
class Seek(SteeringBehaviors.Type):
    '''Seek(weight, container) -> Seek

    Given a target, this behavior returns a steering force which will
        direct the agent towards the target.

    Keyword arguments:
    weight -- Real
    container -- SteeringBehaviors
    '''

    def __call__(self, target_pos):
        ''' -> Vector2D
        target_pos -- Vector2D
        '''
        vehicle = self._container._vehicle
        desired_velocity = (Vector2D(*target_pos)
                .sub(vehicle.position)
                .normalize()
                .mul(vehicle.max_speed))

        return desired_velocity - self._vehicle.velocity
#pylint: enable=W0212
