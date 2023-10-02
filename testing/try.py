import time
import RPi.GPIO as GPIO
from RobotCarClasses import *

Farbsensor = ColorSensor()
Farbsensor.start_measurement()

while True:
   print(Farbsensor.color_temperature)
   time.sleep(0.01)