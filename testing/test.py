import RPi.GPIO as GPIO
import time
from random import *

ledR = 11
ledG = 13
ledB = 15
buttonR = 7
buttonC = 12


GPIO.setmode(GPIO.BOARD)


GPIO.setup(ledR, GPIO.OUT)
GPIO.setup(ledG, GPIO.OUT)
GPIO.setup(ledB, GPIO.OUT)

GPIO.setup(buttonR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buttonC, GPIO.IN, pull_up_down=GPIO.PUD_UP)

R = GPIO.PWM(ledR, 100)
G = GPIO.PWM(ledG, 100)
B = GPIO.PWM(ledB, 100)

R.start(0)
G.start(0)
B.start(0)

def RandomRGB(R, G, B):
    R.ChangeDutyCycle(randrange(0, 10))
    G.ChangeDutyCycle(randrange(0, 10))
    B.ChangeDutyCycle(randrange(0, 10))

def Reset(R, G, B):
    R.ChangeDutyCycle(0)
    G.ChangeDutyCycle(0)
    B.ChangeDutyCycle(0)

while True:
    if GPIO.input(buttonR) == 0:
        Reset(R, G, B)
    elif GPIO.input(buttonC) == 0:
        RandomRGB(R, G, B)
        time.sleep(0.2)