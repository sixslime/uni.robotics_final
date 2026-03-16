# DESCRIPTION:
# generators generate/retrieve/calculate some value and store it in a blackboard variable.

import my_util as my
import numpy as np
import heapq
import nodes
import my_robot

# base class for generators (abstract).
class _Generator(nodes.base.Node):
    def __init__(self, b_out, reads=[], name=None):
        super().__init__(name, reads)
        self.variable = b_out
        self.value = None
    
    def _setup(self):
        self.value = None
        self._start()

    def _terminate(self):
        self._finish()
        nodes.base.Node.write(self.variable, self.value)

    def _finish(self):
        pass

    def _start(self):
        pass

# like 'base.Transform' but provides 'my_robot' as an input to the arbitrary function.
class Generic(_Generator):
    def __init__(self, b_out, list_b_in, generator):
        super().__init__(b_out)
        self.b_ins = list_b_in
        self.gen = generator
    
    def _start(self):
        from nodes.base import Node
        import my_robot
        input = [Node.read(b) for b in self.b_ins]
        self.value = self.gen(my_robot, input)

# retrieves the encoder value of a joint.
class EncoderValue(_Generator):
    def __init__(self, b_out, b_joint):
        super().__init__(b_out, [(b_joint, 'joint')])
    
    def _start(self):
        self.value = my_robot.encoder[self.reads['joint']].getValue()

# retrieves the positions of the robot's currently recognized objects.
class RecognizedObjects(_Generator):
    def __init__(self, b_out):
        super().__init__(b_out)

    def _start(self):
        self.value = my_robot.recognized_objects

# retrieves the rotation error between a floor-point and the robot's current rotation.
class RotationError(_Generator):
    def __init__(self, b_out, b_target):
        super().__init__(b_out, [(b_target, 'target')])

    def _start(self):
        self.value = my_robot.rotationError(self.reads['target'])

# performs the c-space generating sequence.
# (travels in a fixed path around the room, lidar scanning and mapping the space)
class CSpace(_Generator):
    path_points = [(0.78, -1.29), (0.41, -2.8), (-1.43, -3.01), (-1.72, -1.99), (-1.73, -0.78), (-1.38, 0.25), (0.05, 0.27)]
    path = [path_points]

    def __init__(self, b_out, draw=False):
        super().__init__(b_out)
        self.map = None
        self.path_index = [0, 0]
        self.draw = draw

    def _start(self):
        self.map = my_robot.newMap(self.draw)

    def _finish(self):
        self.value = self.map.getConfigurationSpace(my_robot.ROBOT_RADIUS + 0.05)
        my_robot.setWheels([0, 0])

    def _update(self):

        pos = my_robot.position
        rot = my_robot.rotation
        ranges = np.array(my_robot.lidar.getRangeImage())
        ranges[ranges == np.inf] == 888
        
        # lidar data processing:
        X_i = np.array([ranges*np.cos(my_robot.lidar_angles) + my_robot.LIDAR_OFFSET[0], ranges*np.sin(my_robot.lidar_angles), np.ones((my_robot.LIDAR_RES, ))]) 
        w_T_r = np.array([
            [np.cos(rot), -np.sin(rot), pos[0]],
            [np.sin(rot), np.cos(rot), pos[1]],
            [0, 0, 1],
        ])
        D = w_T_r @ X_i
        
        # add every 3rd lidar point to map:
        points = [(x, y) for x, y in zip(D[0, ::3], D[1, ::3])]
        for x, y in points:
            self.map.addPoint((x, y))
        path_index = self.path_index
        dest = self.path[path_index[0]][path_index[1]]

        # detect path destination:
        if not my_robot.headToward(dest, 0.2):
            self.path_index[1] = path_index[1]+1
            # start next path if finished with current path:
            if (path_index[1] >= len(self.path[path_index[0]])):
                self.path_index = [path_index[0]+1, 0]
                # end if all paths finished:
                if self.path_index[0] == len(self.path):
                    return 1
        return 0


# calculates a pixel-by-pixel path to a floor-point given a c-space.
# (uses a*)
class GridPathTo(_Generator):
    def __init__(self, b_point, b_cspace, b_out, draw=False, steps_per_tick=10, name=None):
        super().__init__(b_out, name)
        self.b_goal = b_point
        self.b_cspace = b_cspace
        self.draw = draw
        self.per_tick = steps_per_tick
        self.map = None

    def _heuristic(self, a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def _start(self):
        from nodes.base import Node
        map = my_robot.newMap(False)
        self.start = map.getPixelValue(my_robot.position)
        self.goal = map.getPixelValue(Node.read(self.b_goal))
        self.cspace = Node.read(self.b_cspace)
        self.heap = [(self._heuristic(self.start, self.goal), 0, self.start)]
        self.distances = {self.start: 0}
        self.visited = set()
        self.came_from = {}
        self.counter = 0
        if self.draw:
            my_robot.overlayDisplay(self.cspace, 0xFF4444, 0x000000)
            my_robot.drawPixel(self.start, 0x2299FF)
            my_robot.drawPixel(self.goal, 0xFFFFFF)

    def _update(self):
        pass
    
    def _update(self):
        # setup:
        moves = [(0,1), (0,-1), (1,0), (-1,0)]
        rows, cols = self.cspace.shape

        # search loop:
        steps = 0
        while self.heap:
            steps += 1
            if self.per_tick != None and steps > self.per_tick:
                return 0
            
            _, _, current = heapq.heappop(self.heap)
            
            # check goal:
            if current == self.goal:
                path = [current]
                while current in self.came_from:
                    current = self.came_from[current]
                    path.append(current)
                self.value = path[::-1]
                if self.draw:
                    for p in self.value:
                        my_robot.drawPixel(p, 0x33FF55)
                return 1

            # mark visited:
            if current in self.visited:
                continue
            self.visited.add(current)
            if self.draw:
                my_robot.drawPixel(current, 0x555555)

            # explore neighbors:
            for dx, dy in moves:
                neighbor = (current[0] + dx, current[1] + dy)
                r, c = neighbor

                # check in bounds:
                if not (0 <= r < rows and 0 <= c < cols) or self.cspace[r, c]:
                    continue

                # update scores:
                tentative = self.distances[current] + 1
                if tentative < self.distances.get(neighbor, float('inf')):
                    self.came_from[neighbor] = current
                    self.distances[neighbor] = tentative
                    self.counter += 1
                    d = tentative + self._heuristic(neighbor, self.goal)
                    heapq.heappush(self.heap, (d, self.counter, neighbor))
                    if self.draw:
                        my_robot.drawPixel(neighbor, 0x8888DD)

        return -1

# reduce a path to the minimum representation that can be safely traversed with straight lines, given a c-space.
class OptimizedPath(_Generator):
    def __init__(self, b_path, b_cspace, b_out, draw=False, steps_per_tick=10, name=None):
        super().__init__(b_out, name)
        self.b_path = b_path
        self.b_cspace = b_cspace
        self.draw = draw
        self.per_tick = steps_per_tick
    
    def _start(self):
        from nodes.base import Node
        self.cspace = Node.read(self.b_cspace)
        self.path = Node.read(self.b_path)
        self.value = []
        self.start_i = 0
        self.end_i = len(self.path)-1
        if self.draw:
            my_robot.overlayDisplay(self.cspace, 0xFF2222)
            for p in self.path:
                my_robot.drawPixel(p, 0x006600)

    def _update(self):
        steps = 0
        while self.start_i < len(self.path)-1:
            steps += 1
            if (self.per_tick != None and steps > self.per_tick):
                return 0
            
            start = self.path[self.start_i]
            end = self.path[self.end_i]
            line = my.get_full_line(start, end)

            # check intersect:
            blocked = False
            for lx, ly in line:
                if self.cspace[lx, ly] == True:
                    self.end_i -= 1
                    if self.draw:
                        my_robot.drawPixel(end, 0x00FF00)
                    blocked = True
                    break
            if blocked == True:
                continue

            # add to value:
            self.value.append(end)
            self.start_i = self.end_i
            self.end_i = len(self.path)-1
            if self.draw:
                for p in self.path[self.start_i:]:
                    my_robot.drawPixel(p, 0x444444)
                for p in self.path[:self.end_i]:
                    my_robot.drawPixel(p, 0x006600)
                for p in line:
                    my_robot.drawPixel(p, 0x0088CC)

        self.value.insert(0, self.path[0])
        if self.draw:
            my_robot.overlayDisplay(self.cspace, 0xFF2222)
            for a, b in my.windows(self.value, 2):
                for p in my.get_line(a, b):
                    my_robot.drawPixel(p, 0xFF00FF)
            for p in self.value:
                my_robot.drawPixel(p, 0xFFFFFF)
        return 1

