import RPi.GPIO as GPIO
import time


TrigPin = 38
EchoPin = 40

GPIO.setmode(GPIO.BOARD)
GPIO.cleanup()

GPIO.setup(TrigPin, GPIO.OUT)
GPIO.setup(EchoPin, GPIO.IN)


def read(TrigPin, EchoPin):
    GPIO.output(TrigPin, 0)
    time.sleep(0.000005)
    GPIO.output(TrigPin, 1)
    time.sleep(0.000010)
    GPIO.output(TrigPin, 0)
    while GPIO.input(EchoPin) == 0:
        StartTime = time.time() 

    while GPIO.input(EchoPin) == 1:
        StopTime = time.time()

    delay = StopTime - StartTime 
    distance = (delay * 34300) / 2
    #print(delay)
    
    return distance

while True:
    time.sleep(0.03)
    distance = read(TrigPin, EchoPin)
    print(str(distance) + "cm")