import RPi.GPIO as GPIO
import time


R = 16
F = 18
Speed = 37

GPIO.setmode(GPIO.BOARD)

GPIO.setup(F, GPIO.OUT)
GPIO.setup(R, GPIO.OUT)
GPIO.setup(Speed, GPIO.OUT)

Motor = GPIO.PWM(Speed, 1000)

Motor.start(0)

GPIO.output(F, 1)
time.sleep(2)
Motor.ChangeDutyCycle(95)
time.sleep(100)
GPIO.cleanup()


