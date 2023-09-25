from RobotCarClasses import *
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

Motor1 = Motor(1000, 17, 27, 22) 
Motor1.start()

Utils = Utility(None, None, None, Motor1)

while True:
    try:
        Motor1.drive("r", 80)
        time.sleep(10)
    except:
        Utils.cleanup()