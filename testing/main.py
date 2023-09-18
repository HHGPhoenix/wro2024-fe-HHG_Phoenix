import keyboard
import RPi.GPIO as GPIO
import time
from RobotCarClasses import *


DISTANCE = 30
P = 5

GPIO.setmode(GPIO.BOARD)

StartButton = Button(22)
StopButton = Button(32)

Ultraschall1 = SuperSonicSensor(11, 13)

Servo1 = Servo(7, 50)

Motor1 = Motor(1000, 16, 18, 15)
Motor1.start()
Motor1.drive("r", 85)

Ultraschall1.start_measurement()

running = True
Start = 0

while running:
    time.sleep(0.01)
    try:
        Error = Ultraschall1.distance - DISTANCE
        Correction = P * Error
        if Correction > 100:
            Correction = 100
        elif Correction < -100:
            Correction = -100
        Servo1.steer(-Correction, Motor1, 85)
    except KeyboardInterrupt:
        Ultraschall1.stop_measurement()
        GPIO.cleanup()
        running = False