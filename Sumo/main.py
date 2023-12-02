#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

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

# Write your program here.
ev3.speaker.beep()
ultra_motor.reset_angle(0)


RAND_LINKS = 11
RAND_RECHTS = 12
FAHREN_RECHTS = 21
FAHREN_LINKS = 22


not_angle = 120
reflection_limit = 30

maxa_target = 90
mina_target = -90
us_speed = 90

drive_speed = 200
turn_speed = 0


zustand = FAHREN_RECHTS
ultra_motor.run(us_speed)


def rechts_fahren():
    global zustand
    zustand = FAHREN_RECHTS
    ultra_motor.run(us_speed)

def links_fahren():
    global zustand
    zustand = FAHREN_LINKS
    ultra_motor.run(-us_speed)


# while True:
#     print(cs_left.reflection(), cs_right.reflection())

while True:
    if zustand == FAHREN_RECHTS:
        robot.drive(drive_speed, turn_speed)
        if cs_left.reflection() > reflection_limit:
            zustand = RAND_LINKS
        elif cs_right.reflection() > reflection_limit:
            zustand = RAND_RECHTS
        
        if ultra_motor.angle() >= maxa_target:
            links_fahren()

        if ultra_sensor.distance() < 1000:
            foundAngle = ultra_motor.angle()
            turn_speed = foundAngle * drive_speed / ultra_sensor.distance()
            if foundAngle > -90 and foundAngle < 90:
                maxa_target = foundAngle + 20
                mina_target = foundAngle - 20
            else:
                maxa_target = 90
                mina_target = -90

    elif zustand == FAHREN_LINKS:
        robot.drive(drive_speed, turn_speed)
        if cs_left.reflection() > reflection_limit:
            zustand = RAND_LINKS
        elif cs_right.reflection() > reflection_limit:
            zustand = RAND_RECHTS
        
        if ultra_motor.angle() <= mina_target:
            rechts_fahren()

        if ultra_sensor.distance() < 1000:
            foundAngle = ultra_motor.angle()
            turn_speed = foundAngle * drive_speed / ultra_sensor.distance()
            if foundAngle > -90 and foundAngle < 90:
                maxa_target = foundAngle + 20
                mina_target = foundAngle - 20
            else:
                maxa_target = 90
                mina_target = -90

    elif zustand == RAND_LINKS:
        ultra_motor.hold()
        robot.turn(not_angle)
        if ultra_motor.angle() > 0:
            links_fahren()
        else:
            rechts_fahren()
        maxa_target = 90
        mina_target = -90
        turn_speed = 0

    elif zustand == RAND_RECHTS:
        ultra_motor.hold()
        robot.turn(-not_angle)
        if ultra_motor.angle() > 0:
            links_fahren()
        else:
            rechts_fahren()
        maxa_target = 90
        mina_target = -90
        turn_speed = 0