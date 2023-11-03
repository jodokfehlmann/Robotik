#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
-----------------------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase



# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
gyro_sensor = GyroSensor(Port.S1)
touch_sensor = TouchSensor(Port.S3)
color_sensor = ColorSensor(Port.S4)
tur_sensor = TouchSensor(Port.S2)



# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
tur_motor = Motor(Port.C)

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Play a sound to tell us when we are ready to start moving
ev3.speaker.beep()

def lied():
    erika_melody = [
    "E4/4", "F4/4", "G4/4", "G4/4", "G4/4", "C5/4", "C5/4", "E5/4", "E5/4", "D5/4", "C5/4", "B4/4", "C5/4", "D5/4", "E5/4", "D5/4", "C5/4"
    ]

    ev3.speaker.play_notes(erika_melody, 200)

# The following loop makes the robot drive forward until it detects an
# obstacle. Then it backs up and turns around. It keeps on doing this
# until you stop the program.

WARTEN_TURE_ZU = 0
WARTEN_TURE_OFFEN = 1
TURE_OFFNEN = 11
TURE_SCHLIESSEN = 12
FARBE_FINDEN = 3
GRUEN_FINDEN = 31
ROT_FINDEN = 32

turningSpeed = 100
movingSpeed = 3000
turningAngle = 45

zustand = WARTEN_TURE_ZU
naechste_farbe_finden = FARBE_FINDEN

while True:
    if zustand == WARTEN_TURE_ZU:
        while not (touch_sensor.pressed() or tur_sensor.pressed()):
            pass
        if touch_sensor.pressed():
            zustand = naechste_farbe_finden
        elif tur_sensor.pressed():
            zustand = TURE_OFFNEN
    elif zustand == WARTEN_TURE_OFFEN:
        while not tur_sensor.pressed():
            pass
        zustand = TURE_SCHLIESSEN
    elif zustand == TURE_OFFNEN:
        lied()
        angle = tur_motor.angle()
        tur_motor.run_target(turningSpeed, turningAngle + angle)
        zustand = WARTEN_TURE_OFFEN
    elif zustand == TURE_SCHLIESSEN:
        angle = tur_motor.angle()
        tur_motor.run_target(turningSpeed, angle - turningAngle)
        zustand = WARTEN_TURE_ZU
    elif zustand == FARBE_FINDEN:
        robot.drive(movingSpeed, 0)

        while not (color_sensor.color() == Color.GREEN or color_sensor.color() == Color.RED):
            pass
        if color_sensor.color() == Color.GREEN:
            naechste_farbe_finden = ROT_FINDEN
        elif color_sensor.color() == Color.RED:
            naechste_farbe_finden = GRUEN_FINDEN

        robot.stop()
        zustand = WARTEN_TURE_ZU
    elif zustand == ROT_FINDEN:
        robot.drive(movingSpeed, 0)

        while not color_sensor.color() == Color.RED:
            pass
        naechste_farbe_finden = GRUEN_FINDEN
        robot.stop()
        zustand = WARTEN_TURE_ZU
    elif zustand == GRUEN_FINDEN:
        robot.drive(movingSpeed, 0)

        while not color_sensor.color() == Color.GREEN:
            pass
        naechste_farbe_finden = ROT_FINDEN
        robot.stop()
        zustand = WARTEN_TURE_ZU



WARTEN = 0
TURE_OFFNEN = 1
TURE_SCHLIESSEN = 2
FARBE_FINDEN = 3

turningSpeed = 100

zustand = WARTEN
ture_offen = False
letzte_farbe = None

while True:
    if zustand == WARTEN:
        while not (touch_sensor.pressed() or tur_sensor.pressed()):
            pass
        if touch_sensor.pressed() and not ture_offen:
            zustand = FARBE_FINDEN
        elif tur_sensor.pressed():
            if ture_offen:
                zustand = TURE_SCHLIESSEN
            else:
                zustand = TURE_OFFNEN
    elif zustand == TURE_OFFNEN:
        angle = tur_motor.angle()
        tur_motor.run_target(turningSpeed, 45 + angle)
        zustand = WARTEN
        ture_offen = True
    elif zustand == TURE_SCHLIESSEN:
        angle = tur_motor.angle()
        tur_motor.run_target(turningSpeed, angle - 45)
        zustand = WARTEN
        ture_offen = False
    elif zustand == FARBE_FINDEN:
        robot.drive(500, 0)

        if letzte_farbe == "Grün":
            while not color_sensor.color() == Color.RED:
                pass
            letzte_farbe = "Rot"
        elif letzte_farbe == "Rot":
            while not color_sensor.color() == Color.GREEN:
                pass
            letzte_farbe = "Grün"
        else:
            while not (color_sensor.color() == Color.GREEN or color_sensor.color() == Color.RED):
                pass
            if color_sensor.color() == Color.GREEN:
                letzte_farbe = "Grün"
            elif color_sensor.color() == Color.RED:
                letzte_farbe = "Rot"

        robot.stop()
        zustand = WARTEN

    






while True:
    if zustand == WARTEN:
        while not touch_sensor.pressed():
            pass
    elif zustand == ROT_FINDEN:
        robot.drive(500, 0)
        if color_sensor.color() == Color.RED:
            zustand = ARM_WARTEN
    elif zustand == ARM_WARTEN:
        while not tur_sensor.pressed():
            pass
        zustand = DREHEN
        gyro_sensor.reset_angle(0)
    elif zustand == DREHEN:
        pass
    elif zustand == GRUEN_FINDEN:
        robot.drive(500, 0)
        if color_sensor.color() == Color.GREEN:
            zustand = WARTEN

