import keyboard
import RPi.GPIO as GPIO
import time
from RobotCarClasses import *

GPIO.setmode(GPIO.BOARD)

Ultraschallsensor1 = SuperSonicSensor(11, 13)
Ultraschallsensor1.start_measurement()

while True:
    print(Ultraschallsensor1.distance)
    time.sleep(0.1)