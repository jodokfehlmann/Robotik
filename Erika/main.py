#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
erika_melody = [
    "E4/4", "F4/4", "G4/4", "G4/4", "G4/4", "C5/4", "C5/4", "E5/4", "E5/4", "D5/4", "C5/4", "B4/4", "C5/4", "D5/4", "E5/4", "D5/4", "C5/4"
]

ev3.speaker.play_notes(erika_melody, 200)
