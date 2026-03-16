# DESCRIPTION:
# used for debugging/development.

import my_robot
from nodes.all import *
import math

arms_test = Sequence([
    Sequence(mover, True)
    for mover in [
        [Sequence([
            action.SetJoint(f'arm_{i}', math.pi/2),
            action.Wait(1.2),
        ], True)
        for i in range(1, 8)]
    ]
], True)
fingers_test = Sequence([
    action.SetJoint('gripper_left_finger', 0.045),
    action.Wait(1),
    action.SetJoint('gripper_right_finger', 0.045),
    action.Wait(1),
], True)
lift_test = Sequence([
    action.SetJoint('torso_lift', 0.35),
    action.Wait(6),
    action.SetJoint('torso_lift', 0),
    action.Wait(6),
], True)
head_test = Sequence([
    action.SetJoint('head_1', 1.24),
    action.Wait(1),
    action.SetJoint('head_1', -1.24),
    action.Wait(1),
    action.SetJoint('head_2', 0.79),
    action.Wait(1),
    action.SetJoint('head_2', -0.98),
    action.Wait(1),
], True)

motion_test = Sequence([
    fingers_test,
    head_test,
    arms_test,
    lift_test,
], True)

grab_test = Sequence([
    action.SetJoint('gripper_left_finger', 0.045),
    action.SetJoint('gripper_right_finger', 0.045),
    action.Wait(2),
    action.SetJoint('gripper_left_finger', 0),
    action.SetJoint('gripper_right_finger', 0),
    action.Wait(2),
    action.SetJoint('arm_7', math.pi/2),
    action.Wait(2),

])

wait_test = action.Wait(10)
print_test = Sequence([
    Print(lambda r: [round(enc.getValue(), 2) for enc in r.encoder.values()]),
])

test_behavior = arms_test
test_behavior = Sequence([
    ArmPositionGrab(),
])
def doTest():
    print("-- TESTING --")
    while my_robot.step() and test_behavior.tick() == 0:
        pass
    print ("-- TEST FINISHED --")