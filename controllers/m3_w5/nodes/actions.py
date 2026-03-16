# DESCRIPTION:
# Nodes that directly relate to robot actions.
# Mostly just wrappers around functions in 'my_robot.py' that always return 1.

import nodes

# like 'base.Do' but provides 'my_robot' and inputs to the function.
class Generic(nodes.base.Node):
    def __init__(self,list_b_in, func, returner=None, name=None):
        super().__init__(name)
        self.func = func
        self.ins = list_b_in
        match returner:
            case 1 | 0 | -1:
                self.return_func = lambda _: returner
            case None:
                self.return_func = lambda x: x
            case _:
                self.return_func = returner

    def _setup(self):
        from nodes.base import Node
        self.input = [Node.read(b) for b in self.ins]

    def _update(self):
        import my_robot
        o = self.func(my_robot, self.input)
        return self.return_func(o)
    
# travels toward a floor-point using a general algorithm (wrapper for 'my_robot.headToward()').
class HeadToward(nodes.base.Node):
    def __init__(self, b_point, b_max_speed=10, b_correction_multiplier=1):
        super().__init__(None, [(b_point, 'point'), (b_max_speed, 'max_speed'), (b_correction_multiplier, 'cmult')])

    def _update(self):
        import my_robot
        my_robot.headToward(self.reads['point'], self.reads['max_speed'], self.reads['cmult'])
        return 1
    
# rotates in-place to face a floor-point (wrapper for 'my_robot.rotateToward()').
class RotateToward(nodes.base.Node):
    def __init__(self, b_point, b_anchor):
        super().__init__(None, [(b_point, 'point'), (b_anchor, 'anchor')])

    def _update(self):
        import my_robot
        my_robot.rotateToward(self.reads['point'], self.reads['anchor'])
        return 1

# directly sets speed of the wheels (wrapper for 'my_robot.setWheels()').
class SetWheels(nodes.base.Node):
    def __init__(self, b_speed):
        super().__init__(None, [(b_speed, 'speed')])

    def _update(self):
        import my_robot
        my_robot.setWheels(self.reads['speed'])
        return 1
    
# directly sets position of a joint (wrapper for 'my_robot.setJoint()').
class SetJoint(nodes.base.Node):
    def __init__(self, b_joint, b_value):
        super().__init__(None, [(b_joint, 'joint'), (b_value, 'value')])

    def _update(self):
        import my_robot
        my_robot.setJoint(self.reads['joint'], self.reads['value'])
        return 1

# directly sets speed of a joint (wrapper for 'my_robot.setJointSpeed()').
class SetJointSpeed(nodes.base.Node):
    def __init__(self, b_joint, b_value):
        super().__init__(None, [(b_joint, 'joint'), (b_value, 'value')])

    def _update(self):
        import my_robot
        my_robot.setJointSpeed(self.reads['joint'], self.reads['value'])
        return 1

# draws a pixel on the display (wrapper for 'my_robot.drawPixel()').
# [unused]
class DrawPixel(nodes.base.Node):
    def __init__(self, b_pixel, b_color):
        super().__init__(None, [(b_pixel, 'pixel'), (b_color, 'color')])

    def _update(self):
        import my_robot
        print(self.reads['pixel'])
        my_robot.drawPixel(self.reads['pixel'], self.reads['color'])
        return 1

# does nothing and returns 0 for <x> seconds; then returns 1.
# timer resets if not ticked consecutively (or after returning 1).
class Wait(nodes.base.Node):
    def __init__(self, b_seconds):
        super().__init__(None, [(b_seconds, 'seconds')])
    
    def _setup(self):
        self.time = -1
        self.end_time = -1

    def _update(self):
        import my_robot
        self.time += 1
        if self.time != my_robot.total_ticks:
            self.time = my_robot.total_ticks
            self.end_time = self.time + my_robot.getInTicks(self.reads['seconds'])
        
        if self.time <= self.end_time:
            return 0
        return 1
