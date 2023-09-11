import keyboard
import RPi.GPIO as GPIO
import time
from RobotCar import *


DISTANCE = 30
P = 2.5


GPIO.setmode(GPIO.BOARD)

Motor1 = Motor(1000, 13, 15, 11)

Servo1 = Servo(7, 50)

Ultraschallsensor = SuperSonicSensor(16, 12)
Ultraschallsensor.start_measurement()

time.sleep(0.1)
print(Ultraschallsensor.distance)

"""
while True:
    time.sleep(0.02)
    distance = Ultraschall()
    Error = distance - DISTANCE
    Correction = P * Error
    if Correction > 90:
        Correction = 90
    elif Correction < -90:
        Correction = -90
    print(Error)

    steering(-Correction)

"""