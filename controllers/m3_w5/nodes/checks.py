# DESCRIPTION
# nodes that always return 1 or -1 based on a condition.

import nodes

# like 'action.Generic', but only accepts a predicate and returns 1/-1 based it's evaluation.
class Generic(nodes.base.Node):
    def __init__(self, list_b_in, predicate, name=None):
        super().__init__(name)
        self.ins = list_b_in
        self.predicate = predicate
    
    def _update(self):
        from nodes.base import Node
        import my_robot
        input = [Node.read(b) for b in self.ins]
        o = bool(self.predicate(my_robot, input))
        
        match o:
            case True:
                return 1
            case False:
                return -1
            case _:
                raise ValueError(f"Invalid return value for predicate: {o}")

# checks if the robot is within a distance of a floor-point.
class NearPoint(nodes.base.Node):
    def __init__(self, b_point, b_distance=0.2):
        super().__init__(None, [(b_point, 'point'), (b_distance, 'distance')])

    def _update(self):
        import math, my_robot
        return 1 if math.dist(self.reads['point'], my_robot.position) <= self.reads['distance'] else -1

# checks if the robot is looking at a certain floor-point (within a given degree margin).
class LookingAt(nodes.base.Node):
    def __init__(self, b_point, b_error):
        super().__init__(None, [(b_point, 'point'), (b_error, 'error')])

    def _update(self):
        import my_robot, math
        return 1 if (abs(my_robot.rotationError(self.reads['point']))/math.pi)*180 <= self.reads['error'] else -1

# checks if a joint's value is within a given margin of <x>.
class JointNear(nodes.base.Node):
    def __init__(self, b_joint, b_value, b_error):
        super().__init__(None, [(b_joint, 'joint'), (b_value, 'value'), (b_error, 'error')])
    
    def _update(self):
        import my_robot
        return 1 if abs(my_robot.encoder[self.reads['joint']].getValue() - self.reads['value']) <= self.reads['error'] else -1

# checks if the grippers' force-feedback is within a given range.
class GripperForceWithin(nodes.base.Node):
    def __init__(self, b_gripper_side, b_min=-float('INF'), b_max=float('INF')):
        super().__init__(None, [(b_gripper_side, 'side'), (b_min, 'min'), (b_max, 'max')])
    
    def _update(self):
        import my_robot
        v = my_robot.joint[f'gripper_{self.reads['side']}_finger'].getForceFeedback()
        return 1 if v >= self.reads['min'] and v <= self.reads['max'] else -1