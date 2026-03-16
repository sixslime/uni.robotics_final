# DESCRIPTION:
# acts as an api for accessing/controlling the robot that behavior nodes use.
# contains all direct robot control implementation.
# not documented much because most of it is either just webots functions or has already been covered in previous class projects/lectures.

# This should absolutely be a class; too bad.

from controller import Robot, Motor, Display, Supervisor # type: ignore
import numpy as np
from numpy import sin, cos
import math
import my_util as my
from my_util import INF
from point_map import PointMap

TIME_STEP = 16
ROBOT_RADIUS = 0.265
BASE_ARM_HEIGHT = 0.64
BASE_ARM_LENGTH = 1.08
DISPLAY_RESOLUTION = 100
BASE_ARM_POSITION = (0.1, 0)

# -[ setup ]-
total_ticks = 0
instance = Supervisor()
marker = instance.getFromDef("marker").getField("translation")

# wheels:
motor = {
    'left': instance.getDevice('wheel_left_joint'),
    'right': instance.getDevice('wheel_right_joint'),
}
for m in motor.values():
    m.setPosition(INF)
    m.setVelocity(0)

# joints:
joint = {}
for name in [
    name
    for group in
    [
        ['torso_lift'],
        [f'arm_{i}' for i in range(1, 8)],
        [f'gripper_{d}_finger' for d in ['left', 'right']],
        [f'head_{i}' for i in range(1, 3)]
    ]
    for name in group
]:
    joint[name] = instance.getDevice(f'{name}_joint')
for device in joint.values():
    device.enableForceFeedback(TIME_STEP)

safe_joint_positions = {
    'torso_lift': 0.35,
    'arm_1': 0.71,
    'arm_2': 1.02,
    'arm_3': -2.815,
    'arm_4': 1.011,
    'arm_5': 0,
    'arm_6': 0,
    'arm_7': 0,
    'gripper_left_finger' : 0,
    'gripper_right_finger': 0,
    'head_1':0,
    'head_2':0
}
# for name, value in safe_joint_positions.items():
#     joint[name].setPosition(value)

# encoders:
encoder = {}
for name in [
    name
    for group in
    [
        ['torso_lift'],
        [f'arm_{i}' for i in range(1, 8)],
        [f'head_{i}' for i in range(1, 3)]
    ]
    for name in group
]:
    encoder[name] = instance.getDevice(f'{name}_joint_sensor')
encoder.update({
    'gripper_left_finger': instance.getDevice('gripper_left_sensor_finger_joint'),
    'gripper_right_finger': instance.getDevice('gripper_right_sensor_finger_joint')
})
for device in encoder.values():
    device.enable(TIME_STEP)

# lidar:
LIDAR_RES = 360
LIDAR_OFFSET = (0.202, 0)
lidar = instance.getDevice("Hokuyo URG-04LX-UG01")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
lidar_angles = np.linspace(np.pi/2, -np.pi/2, LIDAR_RES)

# gps / compass / display / camera:
gps = instance.getDevice("gps")
gps.enable(TIME_STEP)
compass = instance.getDevice("compass")
compass.enable(TIME_STEP)
display = instance.getDevice('display')
camera = instance.getDevice('camera')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

# declare info fields:
position = [0, 0, 0]
position_delta = [0, 0, 0]
rotation_delta = 0
rotation = 0
lidar_points = []
lidar_ranges = []
recognized_objects = []
w_T_r = []

# -[ define functions ]-
def newMap(draw = False):
    return PointMap(
    center = (-0.6, -1.2),
    width = 6,
    resolution = DISPLAY_RESOLUTION,
    threshold_sec = 2,
    display = display,
    draw = draw
)

MAP = newMap()

def getInTicks(seconds):
    return math.ceil((seconds*1000)/TIME_STEP)
    
def setWheels(speeds):
    print(f"SPEED: {speeds}")
    speeds = [my.clamp(v, -1, 1) for v in speeds]
    # 2pi is arbitrary tbh.
    motor['left'].setVelocity(speeds[0]*my.PI*2)
    motor['right'].setVelocity(speeds[1]*my.PI*2)

def setJoint(name, value):
    print(f"JOINT: {name} -> {value}")
    joint[name].setVelocity(joint[name].getMaxVelocity())
    joint[name].setPosition(value)

def setJointSpeed(name, value):
    print(f"JOINT SPEED: {name} -> {value}")
    joint[name].setPosition(INF)
    joint[name].setVelocity(value)

def headToward(destination, max_speed=1, correction_multiplier=1):
    marker.setSFVec3f([*destination, 0])
    speed = (0, 0)
    # calc position error:
    pos_err = np.sqrt(((position[0]-destination[0])**2) + ((position[1]-destination[1])**2))

    # calc rotation error:
    dest_rot = np.arctan2(destination[1]-position[1], destination[0]-position[0])
    rob_rot = np.arctan2(np.sin(rotation), np.cos(rotation))
    rot_err_raw = dest_rot - rob_rot
    
    # normalize rotation error:
    # (bullshit)
    rot_err = np.arctan2(np.sin(rot_err_raw), np.cos(rot_err_raw))
    if (rot_err > np.pi):
        rot_err -= 2*np.pi
    
    # wheelspeed algorithm:
    rot_factor = (rot_err / np.pi)
    pos_factor = my.clamp((my.clamp(pos_err*2, 0, 0.5) - abs(rot_factor*4)), 0.2, 0.6)
    if (abs(rot_factor*180) > 20):
        pos_factor = 0
        rot_factor = my.clamp(rot_factor*1.5, -1, 1)
    if pos_err < 0.4:
        pos_factor = pos_factor/1.5
    elif (abs(rot_factor*180) < 10):
        pos_factor = pos_factor*2
    pos_factor = min(pos_factor, max_speed)
    rot_factor = rot_factor * correction_multiplier
    speed = ((pos_factor - rot_factor), (pos_factor + rot_factor))
    setWheels(speed)
    return True

def rotationError(destination):
    return my.rotationError(position, rotation, destination)

def toWorldPos(pos):
    pos = np.array([*[x for x in pos], 1])
    return [x for x in w_T_r @ pos]

def rotateToward(destination, anchor_pos):
    marker.setSFVec3f([*destination, 0])
    speed = (0, 0)

    anchor_direction = 1 if abs(rotationError(anchor_pos)) < math.pi/4 else -1
    pos_err = math.dist(position, anchor_pos)
    anchor_factor = 0
    if pos_err > 0.2:
        v = anchor_direction * pos_err * 2
        setWheels([v, v])
        return True
    rot_err = rotationError(destination)
    deg_rot_err = abs(rot_err/np.pi) * 180
    # wheelspeed algorithm:
    rot_factor = my.clamp(my.sign(rot_err)*(0.05+(abs(rot_err / np.pi))), -0.3, 0.3)

    speed = (anchor_factor-rot_factor, anchor_factor + rot_factor)
    setWheels(speed)
    return True

def overlayDisplay(pixels, true_color=0xFFFFFF, false_color=None):
    for x, y in my.range2d(0, DISPLAY_RESOLUTION, 0, DISPLAY_RESOLUTION):
        display.setColor(true_color if pixels[x, y] else false_color)
        display.drawPixel(x, y)

def fillDisplay(color=0x000000):
    display.setColor(color)
    display.fillRectangle(0, 0, DISPLAY_RESOLUTION, DISPLAY_RESOLUTION)

def drawPixel(pixel, color):
    display.setColor(color)
    display.drawPixel(pixel[0], pixel[1])

def step():
    global position
    global position_delta
    global rotation
    global rotation_delta
    global total_ticks
    global lidar_points
    global lidar_ranges
    global recognized_objects
    global w_T_r

    total_ticks += 1
    pos = gps.getValues()[:2]
    rot = np.arctan2(compass.getValues()[0],compass.getValues()[1])
    position_delta = [pos[i]-position[i] for i in range(len(pos))]
    rotation_delta = rot-rotation
    position = pos
    rotation = rot
    w_T_r = np.array([
            [cos(rotation), -sin(rotation), position[0]],
            [sin(rotation), cos(rotation), position[1]],
            [0, 0, 1],
        ])
    # lidar data:
    if total_ticks > 1:
        ranges = np.array(lidar.getRangeImage())
        lidar_ranges = ranges
        ranges[ranges == np.inf] == 888

        X_i = np.array([ranges*np.cos(lidar_angles) + LIDAR_OFFSET[0], ranges*sin(lidar_angles), np.ones((LIDAR_RES, ))]) 
        
        lidar_points = w_T_r @ X_i
    
    # recognition objects:
    if total_ticks > 1:
        objects = [list(o.getPosition()) for o in camera.getRecognitionObjects()]
        # torso lift:
        pos = encoder['torso_lift'].getValue()
        T_0_1 = np.array([
            [1, 0, 0, -0.054],
            [0, 1, 0, 0], 
            [0, 0, 1, 0.6+0.193+pos],
            [0, 0, 0, 1],
        ])
        # head 1:
        a = encoder['head_1'].getValue()
        T_1_2 = np.array([
            [cos(a), -sin(a), 0, 0.182],
            [sin(a), cos(a), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]),
        # head 2:
        a = -encoder['head_2'].getValue()
        T_2_3 = np.array([
            [cos(a), 0, sin(a), 0.005],
            [0, 1, 0, 0],
            [-sin(a), 0, cos(a), 0.098],
            [0, 0, 0, 1],
        ]),
        # camera:
        T_3_4 = np.array([
            [1, 0, 0, 0.107],
            [0, 1, 0, 0],
            [0, 0, 1, 0.0802],
            [0, 0, 0, 1],
        ])
        T_FINAL = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4
        recognized_objects = [list((T_FINAL @ np.array([*o, 1]))[0][:3]) for o in objects]

    return instance.step(TIME_STEP) != -1


    