import keyboard
import RPi.GPIO as GPIO
import time
from RobotCarClasses import *


DISTANCE = 30
P = 2.5


GPIO.setmode(GPIO.BOARD)

Motor1 = Motor(1000, 13, 15, 11)

Servo1 = Servo(7, 50)

Ultraschallsensor = SuperSonicSensor(16, 12)
Ultraschallsensor.start_measurement()

time.sleep(0.1)
print(Ultraschallsensor.distance)
