# DESCRIPTION:
# includes all node constructs (abstractions of common node patterns).
# most are processes that return 0 until done (returning 1).
# also serves the purpose of making 'from nodes.all import *' import all nodes in an organized way.

from nodes.base import *
import nodes.actions as action
import nodes.checks as check
import nodes.generators as generate

# prints a value when ticked, returning 1.
# [used for debugging]
def Print(getter):
    return action.Generic(lambda r, _: print(getter(r)), [], 1)

# provides a convenient syntax for calling 'JointTo' on multiple arm nodes.
def SetArmJoints(arm_sets, b_error=0.01, b_max_speed=None):
    return ParallelAll([
        JointTo(f'arm_{arm}', value, b_error, b_max_speed)
        for arm, value in arm_sets
    ], True)

# sets arm to a "safe" position that's out of the way.
def ArmPositionSafe(b_max_speed=None):
    import math
    return ParallelAll([
        SetArmJoints([
            (2, 1.02), 
            (5, math.pi/2), 
            (6, 0), (7, 0)
            ], b_max_speed=b_max_speed),
        Sequence([
            JointTo('arm_3', -math.pi, b_max_speed=b_max_speed),
            JointTo('arm_4', 1, b_max_speed=b_max_speed),
        ])
    ])

# sets arm to a position thats good for travelling across the room.
def ArmPositionTravel(b_max_speed=None):
    import math
    return ParallelAll([
        ArmPositionSafe(b_max_speed),
        JointTo('arm_1', math.pi/2, b_max_speed=b_max_speed)
    ])

# sets arm to the grabbing position.
def ArmPositionGrab(b_max_speed=None):
    import math
    return SetArmJoints([
            (1, (math.pi/2)+0.04),
            (2, 0),
            (3, -math.pi/2),
            (4, 0),
            (5, -math.pi/2),
            (6, 0),
            (7, math.pi/2),
        ],
        b_max_speed=b_max_speed)

# sets arm to a position thats good for holding objects.
def ArmPositionHoldingObject(b_max_speed=None):
    import math
    return Sequence([
        SetArmJoints([
            (1, math.pi/2),
            (3, -math.pi),
            (5, -math.pi/2)
        ] + [(i, 0) for i in range(6, 8)], b_max_speed=1),
        SetArmJoints([(4, math.pi/5)], b_max_speed=b_max_speed),
        SetArmJoints([
            (2, -math.pi/4),
            (4, (3*math.pi)/5),
        ], b_max_speed=b_max_speed)
    ])

# checks if this robot is *actually* moving.
# (used for checking if the robot is stuck.)
def IsMoving(b_pos_epsilon=0.001, b_rot_epsilon=0.001):
    import math
    return Sequence([
        check.Generic([b_pos_epsilon, b_rot_epsilon],
        lambda r, epsilons: math.dist([0, 0], r.position_delta) > epsilons[0] or abs(r.rotation_delta) > epsilons[1], "IsMoving")
    ])

# checks if gripping an object with a force.
def IsGrabbingObject(b_grip_force=10):
    return Sequence([
        Transform([b_grip_force], 'grip_force_check', lambda ins: -ins[0]),
        check.GripperForceWithin('left', b_max=Ref('grip_force_check')),
        check.GripperForceWithin('right', b_max=Ref('grip_force_check')),
    ], False)

# opens the gripper.
def OpenGripper():
    return ParallelAll([
        JointTo('gripper_left_finger', 0.045, 0.001),
        JointTo('gripper_right_finger', 0.045, 0.001),
    ])

# closes the gripper; returns 1 if fully closes, -1 if cannot fully close (likely due to an object being grabbed).
def CloseGripper(b_max_force=10, b_timeout=5):
    return ParallelOne([
        Invert(action.Wait(b_timeout)),
        ParallelAll([
            Invert(IsGrabbingObject(b_max_force)),
            JointTo('gripper_left_finger', 0, 0.01),
            JointTo('gripper_right_finger', 0, 0.01),
        ]),
    ])

# shorthand for SetWheels([0, 0])
def StopWheels():
    return action.SetWheels([0, 0])

# stops all joint movement.
# [unused]
def StopAllJoints():
    import my_robot
    return Sequence([
        action.SetJointSpeed(joint, 0)
        for joint in my_robot.joint.keys()
    ])

# attemps to move a joint to a position, with an optional max speed.
# returns 0 until the joint reaches target position (within a given error margin).
def JointTo(b_joint, b_value, b_error=0.01, b_max_speed=None):
    import my_util as my
    def gen(inputs):
        max_speed = inputs[0]
        if max_speed == None:
            return ParallelAll([
                Until(1, check.JointNear(b_joint, b_value, b_error)),
                action.SetJoint(b_joint, b_value),
            ])
        if max_speed <= 0:
            return Do(lambda: print(f"Invalid joint speed requested: {max_speed}"), -1)
        return Sequence([
            generate.EncoderValue('joint_pos', b_joint),
            Transform([b_value, Ref('joint_pos')], 'joint_speed',
                lambda ins: my.sign(ins[0]-ins[1]) * max_speed),
            action.SetJointSpeed(b_joint, Ref('joint_speed')),
            Until(1, check.JointNear(b_joint, b_value, b_error)),
            action.SetJointSpeed(b_joint, 0),
        ], False)
    return Dynamic([b_max_speed], gen, "JointTo")

# like 'action.HeadToward()', but tries to un-stuck the robot if it's stuck.
# returns 0 until destination is reached (within a given margin)
def SmartHeadToward(b_point, b_error, b_stuck_timer=1):
    import random
    return Sequence([
        Store('backing_up', lambda: False),
        ParallelAll([
            Until(1, check.NearPoint(b_point, b_error)),
            ParallelAll([
                Selector([
                    Selector([
                        IsMoving(),
                        Invert(action.Wait(b_stuck_timer)),
                    ]),
                    Sequence([
                        Store('backing_up', lambda: True),
                        action.SetWheels([random.uniform(-0.2, -0.5), random.uniform(-0.2, -0.5)]),
                        action.Wait(3),
                        StopWheels(),
                        action.Wait(0.5),
                        Store('backing_up', lambda: False),
                    ]),
                ], True),
                Selector([
                    check.Generic([Ref('backing_up')], lambda _, ins: ins[0] == True),
                    action.HeadToward(b_point),
                ])
            ])
        ])
    ])

# follows a path (sequence of floor-points) using 'SmartHeadToward()'.
def FollowPath(b_path, b_pos_margin):
    def gen(inputs):
        return Sequence([
            Sequence([
                SmartHeadToward(b_point, b_pos_margin),
                StopWheels(),
                action.Wait(0.2)
                ])
                for b_point in inputs[0]
            ], True)
    return Dynamic([b_path], gen, 'FollowPathPrecise')

# plans a path to a point (given a c-space) and then follows that path.
def PlanAndGoTo(b_point, b_pos_margin, b_cspace = Ref('cspace')):
    return Sequence([
        generate.GridPathTo(b_point, b_cspace, 'grid_path', True, 20),
        generate.OptimizedPath(Ref('grid_path'), b_cspace, 'pixel_path', True, 10),
        generate.Generic('path', [Ref('pixel_path')], lambda r, ins: [r.MAP.getWorldValue(p) for p in ins[0]]),
        ParallelAll([
            action.Generic([], lambda r, _: r.drawPixel(r.MAP.getPixelValue(r.position), 0x00AACC), 1),
            FollowPath(Ref('path'), b_pos_margin)
        ])
    ], True, "PlanAndGoTo")

# rotates in-place until it is looking at a target floor-point (within a given margin).
def RotateToLookAt(b_point, b_error=0.2):
    import my_robot
    return Sequence([
        Store('look_anchor', lambda: my_robot.position),
        Until(-1, IsMoving()),
        ParallelAll([
            Until(1, check.LookingAt(b_point, b_error)),
            action.RotateToward(b_point, Ref('look_anchor'))
        ]),
        StopWheels()
    ])

# puts robot in grab-ready position.
def PrepareToGrab(b_obj_pos):
    return Sequence([
        generate.Generic('lift', [b_obj_pos], lambda r, ins: max(0, ins[0][2]-r.BASE_ARM_HEIGHT+0.03)),
        ParallelAll([
            JointTo('torso_lift', Ref('lift'), 0.05),
            ArmPositionGrab(0.8),
            OpenGripper(),
        ]),
        action.SetJoint('arm_2', 0)
    ])

# fully prepares, moves toward, and attemps to grab an object at a target position.
# returns 1 if an object was grabbed, -1 if not.
def AttemptGrab(b_obj_pos, b_force=10):
    import math
    import my_util as my
    return Sequence([
        PrepareToGrab(b_obj_pos),
        action.Wait(0.2),
        Transform([b_obj_pos], 'grab_target', lambda ins: ins[0][:2]),
        ParallelAll([
            action.HeadToward(Ref('grab_target'), 0.25, 0.7),
            Sequence([
                generate.RotationError('grab_rot_err', Ref('grab_target')),
                Transform([Ref('grab_rot_err')], 'arm_angle', lambda ins: ins[0] + math.pi/2),
                generate.Generic('wrist_angle', [Ref('grab_target')],
                    lambda r, ins: my.rotationError(r.toWorldPos([0.8, 0]), r.rotation, ins[0])/2),
                action.SetJoint('arm_6', Ref('wrist_angle')),
            ], False),
            Until(1, check.Generic([Ref('grab_target')],
                lambda r, ins: math.dist(ins[0], r.position) <= r.BASE_ARM_LENGTH-0.06)),
        ]),
        StopWheels(),
        Invert(CloseGripper(b_force)),
    ])