#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
motor_left = Motor(Port.A)
motor_right = Motor(Port.B)
ultra_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

ultra_sensor = UltrasonicSensor(Port.S1)
cs_right = ColorSensor(Port.S2)
cs_left = ColorSensor(Port.S3)

robot = DriveBase(motor_left, motor_right, 56, 87)
robot.settings(300, 1000)

watch = StopWatch()

# Write your program here.
ev3.speaker.beep()
ultra_motor.reset_angle(0)


RAND_LINKS = 11
RAND_RECHTS = 12
FAHREN = 2
SCANNEN_RECHTS = 31
SCANNEN_LINKS = 32
NICHTS = 4


not_angle = 120
reflection_limit = 30

maxa_target = 90
mina_target = -90
us_speed = 90
us_distances = {}

drive_speed = 300
turn_speed = 0


zustand = SCANNEN_RECHTS
watch.reset()


def reflection_detection():
    if cs_left.reflection() > reflection_limit:
        zustand = RAND_LINKS
    elif cs_right.reflection() > reflection_limit:
        zustand = RAND_RECHTS
    else:
        zustand = None
    return zustand

def check_end():
    if watch.time() > 90000 or Button.LEFT in ev3.buttons.pressed():
        ultra_motor.hold()
        robot.stop()
        return NICHTS
    
    return None


while True:
    zustand = check_end() or zustand

    if zustand == FAHREN:
        ultra_motor.run_target(us_speed, 0, wait=False)
        robot.drive(drive_speed, turn_speed)

        zustand = reflection_detection() or zustand

        

    elif zustand == RAND_LINKS:
        robot.turn(not_angle)
        zustand = SCANNEN_RECHTS
        us_distances.clear()

    elif zustand == RAND_RECHTS:
        robot.turn(-not_angle)
        zustand = SCANNEN_RECHTS
        us_distances.clear()

    elif zustand == SCANNEN_RECHTS:
        ultra_motor.run(us_speed)
        us_distances = {}
        us_distances[ultra_motor.angle()] = ultra_sensor.distance()

        if ultra_motor.angle() >= 90:
            zustand = SCANNEN_LINKS
        
        zustand = reflection_detection() or zustand


    elif zustand == SCANNEN_LINKS:
        ultra_motor.run(-us_speed)
        us_distances[ultra_motor.angle()] = ultra_sensor.distance()
 
        if ultra_motor.angle() <= -90:
            angle = min(us_distances, key=us_distances.get)
            ultra_motor.stop()
            turn_speed = angle * drive_speed / (math.pi * ((us_distances[angle] / 2) / math.sin(angle)) * 2 * angle / 180)
            while not watch.time() > 5000:
                pass    
            zustand = FAHREN

        zustand = reflection_detection() or zustand
    
    elif zustand == NICHTS:
        ultra_motor.run_target(us_speed, 0)
        break
