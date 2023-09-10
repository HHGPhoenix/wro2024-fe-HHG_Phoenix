import RPi.GPIO as GPIO
import time


Signal = 12

GPIO.setmode(GPIO.BOARD)

GPIO.setup(Signal, GPIO.OUT)

servo1 = GPIO.PWM(Signal, 50)

servo1.start(6.6)
time.sleep(2)
servo1.ChangeDutyCycle(5.1)
time.sleep(2)
servo1.ChangeDutyCycle(8.1)
time.sleep(2)
GPIO.cleanup()