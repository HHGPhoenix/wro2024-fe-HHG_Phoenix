import RPi.GPIO as GPIO
import time
from RobotCarClasses import *

Motor1 = Motor(1000, 27, 17, 22)
Motor1.start()
Servo1 = Servo(18, 50)

Motor1.drive("f", 100)

while True:
    Servo1.steer(-100)
    time.sleep(0.5)
    Servo1.steer(100)
    time.sleep(0.5)