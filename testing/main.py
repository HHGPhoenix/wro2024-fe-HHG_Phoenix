import keyboard
import RPi.GPIO as GPIO
import time
from RobotCarClasses import *


DISTANCE = 30
P = 5
GPIO.setmode(GPIO.BOARD)

Ultraschall1 = SuperSonicSensor(11, 13)

Servo1 = Servo(7, 50)

Motor1 = Motor(1000, 16, 18, 15)
Motor1.start()

Ultraschall1.start_measurement()

time.sleep(1)

Motor1.drive("r", 75)

running = True
while running:
    try:
        Error = Ultraschall1.distance - DISTANCE
        Correction = P * Error
        if Correction > 100:
            Correction = 100
        elif Correction < -100:
            Correction = -100
        print(Ultraschall1.distance)
        Servo1.steer(-Correction)
    except KeyboardInterrupt:
        Ultraschall1.stop_measurement()
        GPIO.cleanup()
        running = False


