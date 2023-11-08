import RPi.GPIO as GPIO
import time
from RobotCarClasses import *

Gyro = Gyroscope()

while True:
    StartTime = time.time()
    print(Gyro.get_angle())
    StopTime = time.time()
    #print(StopTime - StartTime)