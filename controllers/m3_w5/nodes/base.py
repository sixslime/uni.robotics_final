# DESCRIPTION:
# nodes make up the foundation of the behavior-tree system.

# NODE (_update()) RETURN CODES:
# 1 - success.
# -1 - failure.
# 0 - running.

# base node class (abstract):
class Node:
    import os
    resource_dir = os.path.join(os.path.split(os.path.split(os.getcwd())[0])[0], 'my_resources', 'm3_w5')
    blackboard = {}

    def get_save_path(self, variable):
        import os
        return os.path.join(self.resource_dir, f"{variable}.npy")

    # 'blackboard_reads' simplifies getting values from the blackboard at setup time.
    def __init__(self, name=None, blackboard_reads=None):
        self._running = False
        self.name = name or self.__class__.__name__
        self.reads = None
        self._blackboard_reads = blackboard_reads or []

    # populates 'self.reads' at setup time:
    def __setup(self):
        self.reads = {}
        for b, name in self._blackboard_reads:
            self.reads[name] = Node.read(b)
        self._setup()

    # abstract methods:
    def _update(self):
        return 1

    def _setup(self):
        pass

    def _terminate(self):
        pass
    
    # for interacting with blackboard:
    def read(b_var):
        if isinstance(b_var, Ref):
            return Node.blackboard[b_var.variable_name]
        return b_var
    
    def write(b_var, value):
        print(f"var: '{b_var}' <- {value}")
        Node.blackboard[b_var] = value

    # main tick function:
    def tick(self):
        if self._running == False:
            self.__setup()
            self._running = True
        o = self._update()
        match o:
            case 1 | -1:
                self._terminate()
                self._running = False
            case 0:
                pass
            case _:
                raise ValueError(f"Invalid return code: {o}")
        return o

# allows for arbitrarily defined node actions.
class Do(Node):
    def __init__(self, func, returner=None, name=None):
        super().__init__(name)
        self.func = func
        match returner:
            case 1 | 0 | -1:
                self.return_func = lambda _: returner
            case None:
                self.return_func = lambda x: x
            case _:
                self.return_func = returner

    def _update(self):
        o = self.func()
        return self.return_func(o)

# stores a value in a blackboard variable.
class Store(Node):
    def __init__(self, variable, value_generator, name=None):
        super().__init__(name)
        self.val_gen = value_generator
        self.v_name = variable

    def _setup(self):
        Node.write(self.v_name, self.val_gen())

# allows for dynamically generated nodes based off of blackboard inputs.
class Dynamic(Node):
    def __init__(self, list_b_in, node_generator, name=None):
        super().__init__(name)
        self.node_gen = node_generator
        self.ins = list_b_in
        self.node = None
    
    def _setup(self):
        input = [Node.read(b) for b in self.ins]
        self.node = self.node_gen(input)
        self.node.name = f'D_{self.name}'
        return self.node._setup()

    def _update(self):
        return self.node._update()

    def _terminate(self):
        return self.node._terminate()

# base class for composite nodes (abstract).
class CompositeNode(Node):
    def __init__(self, children=[], name=None):
        super().__init__(name)
        self.children = children
    
    def _composite():
        return 1
    
    def _update(self):
        if len(self.children) == 0:
            return 1
        return self._composite()

# base sequence node.
# if 'memory' == True (default), keeps track of where it is in the sequence.
# otherwise will always start from the beginning of it's child nodes when ticked.
class Sequence(CompositeNode):
    def __init__(self, children=[], memory=True, name=None):
        super().__init__(children, name)
        self.index = 0
        self.memory = memory

    def _setup(self):
        self.index = 0

    def _composite(self):
        for child in self.children[self.index:]:
            o = child.tick()
            match o:
                case 1:
                    if self.memory:
                        self.index += 1
                    continue
                case -1:
                    return -1
                case 0:
                    return 0
        return 1
    
# base selector node.
# (same 'memory' concept as Sequence, except defaults to False)
class Selector(CompositeNode):
    def __init__(self, children=[], memory=False, name=None):
        super().__init__(children, name)
        self.index = 0
        self.memory = memory

    def _setup(self):
        self.index = 0
    
    def _composite(self):
        for child in self.children[self.index:]:
            o = child.tick()
            match o:
                case -1:
                    if self.memory:
                        self.index += 1
                    continue
                case 1:
                    return 1
                case 0:
                    return 0
        return -1


# parallel node; runs until ALL children return 1 (or one returns -1).
# if 'memory' == True, will not tick child nodes that have already finished.
class ParallelAll(CompositeNode):
    def __init__(self, children, memory=False):
        super().__init__(children, None)
        self.memory = memory
        self.completed = None

    def _setup(self):
        self.completed = set()

    def _composite(self):
        for i, result in [
                (i, None) if i in self.completed else (i, child.tick())
                for i, child in enumerate(self.children)
            ]:
            match result:
                case None:
                    continue
                case 0 | -1:
                    return result
                case 1:
                    if self.memory:
                        self.completed.add(i)
                    continue
        return 1

# parallel node; runs until ONE returns 1 or -1.
class ParallelOne(CompositeNode):
    def _composite(self):
        for result in [child.tick() for child in self.children]:
            match result:
                case 1 | -1:
                    return result
                case 0:
                    continue
        return 0

# returns 0 until <x> is returned from it's child.
class Until(Node):
    def __init__(self, b_code, node):
        super().__init__(None, [(b_code, 'code')])
        self.node = node
    
    def _update(self):
        return 1 if self.node.tick() == self.reads['code'] else 0
    
# inverts the return code of it's child (multiplies by -1).
class Invert(CompositeNode):
    def __init__(self, node):
        super().__init__([node])
    
    def _composite(self):
        return -1 * self.children[0].tick()
    
# checks if blackboard variable exists.
class Exists(Node):
    def __init__(self, variable):
        super().__init__()
        self.variable = variable
    def _update(self):
        return 1 if self.variable in self.blackboard else -1

# like Store, but allows blackboard inputs.
# ('x = func(<inputs>)')
# (kinda a bad name tbh)
class Transform(Node):
    def __init__(self, list_b_in, b_out, func, name=None):
        super().__init__(name)
        self.func = func
        self.ins = list_b_in
        self.b_out = b_out
    
    def _update(self):
        Node.write(self.b_out, self.func([Node.read(b) for b in self.ins]))
        return 1
    
# identifier class for referencing a blackboard variable.
# (see Node.read())
class Ref():
    def __init__(self, variable_name):
        self.variable_name = variable_name

# reads a numpy file to a variable.
class FromFile(Node):
    def __init__(self, variable):
        super().__init__()
        self.variable = variable

    def _update(self):
        import os, numpy
        file_loc = self.get_save_path(self.variable)
        if not os.path.exists(file_loc):
            return -1
        self.blackboard[self.variable] = numpy.load(file_loc)
        return 1
    
# saves a numpy value to a file.
class ToFile(Node):
    def __init__(self, variable):
        super().__init__()
        self.variable = variable
    
    def _update(self):
        import numpy
        numpy.save(self.get_save_path(self.variable), self.blackboard[self.variable])
        return 1