# DESCRIPTION:
# See 'nodes/' folder for implementation of behavior tree nodes.
#- 'nodes/base.py' contains basis/core nodes.
#- 'nodes/all.py' contains node constructs (most of what you see aside from core nodes).
# 'my_robot.py' contains implementation for actual robot/world interaction.
# I apologize for such sparse comments, theres just so much and I am forking done working on this project.

# tldr; most important details are in 'nodes/base.py' and 'nodes/all.py'

# if you want to observe c-space scanning behavior, delete the file '<project>/my_resources/m3_w5/cspace.npy')

import my_robot
from nodes.all import *
import math

# --[define locations]--
counter_basepoint = (0, 0)
counter_lookpoint = (1.7, 0)
table_basepoint = (0.65, -1.45)
table_lookpoint = (-0.61, -1.45)
table_height = 0.74
place_locations = [(-0.43, -0.78), (-0.46, -1.48), (-0.52, -2.04)]

# --[ digestable behavior-step abstractions ]--
def GetCSpace():
    return Selector([
        Exists('cspace'),
        FromFile('cspace'),
        Sequence([
            ArmPositionTravel(),
            generate.CSpace('cspace', draw=True),
            ToFile('cspace'),
        ]),
    ])

def GoToCounter():
    return Sequence([
        PlanAndGoTo(counter_basepoint, 0.1),
        RotateToLookAt(counter_lookpoint)
    ])

def GoToTable():
    return Sequence([
        PlanAndGoTo(table_basepoint, 0.1),
        RotateToLookAt(table_lookpoint)
    ])

def ObtainFirstRecognizedObject():
    return Sequence([
        action.Wait(0.5),
        generate.RecognizedObjects('objects'),
        Transform([Ref('objects')], 'object', lambda ins: ins[0][0]),
        Selector([
            AttemptGrab(Ref('object'), 5),
            Sequence([
                action.SetWheels([-0.4, -0.4]),
                action.Wait(1.5),
                StopWheels(),
                Do(lambda: print("Pick-up attempt failed, trying again."), -1),
            ]),
        ], True),
        action.SetWheels([-0.4, -0.4]),
        ParallelAll([
            ArmPositionHoldingObject(0.5),
            Sequence([
                action.Wait(1.5),
                StopWheels(),
            ]),
        ], True),
        StopWheels(),
    ])

def PlaceAtNextLocation():
    return Sequence([
        ArmPositionGrab(0.6),
        Transform([Ref('placed_count')], 'place_location', lambda ins: place_locations[ins[0]]),
        RotateToLookAt(Ref('place_location')),
        JointTo('torso_lift', table_height-my_robot.BASE_ARM_HEIGHT+0.1),
        OpenGripper(),
        action.Wait(0.5)
    ])

# main behavior node:
# (this fully expands into hundreds of nodes probably)
behavior = Sequence([
    GetCSpace(),
    Store('placed_count', lambda: 0),
    # 'Until(1, <sequence node>)' will keep restarting the sequence until it returns 1.
    Until(1, Sequence([
        ArmPositionTravel(),
        GoToCounter(),
        ObtainFirstRecognizedObject(),
        GoToTable(),
        PlaceAtNextLocation(),
        # increments 'placed_count'
        Transform([Ref('placed_count')], 'placed_count', lambda ins: ins[0]+1),
        # will always return -1 if 'placed_count' is less than 3
        check.Generic([Ref('placed_count')], lambda _, ins: ins[0] >= len(place_locations)),
    ]))
])

# main loop:
print("-- START --")
while my_robot.step():
    o = behavior.tick()
    match o:
        case 1:
            print("SUCCESS.")
            break
        case -1:
            print("FAILURE.")
            break
        case 0:
            continue

