import keyboard
import RPi.GPIO as GPIO
import time

F = 16
R = 18
Signal = 12
TrigPin = 38
EchoPin = 40

GPIO.setmode(GPIO.BOARD)


GPIO.setup(F, GPIO.OUT)
GPIO.setup(R, GPIO.OUT)

GPIO.setup(Signal, GPIO.OUT)

GPIO.setup(TrigPin, GPIO.OUT)
GPIO.setup(EchoPin, GPIO.IN)


servo1 = GPIO.PWM(Signal, 50)
servo1.start(6.7)


class drive:
    def straight(self, speed):
        
       

def Ultraschall():
    #Trigger
    GPIO.output(TrigPin, 0)
    time.sleep(0.000005)
    GPIO.output(TrigPin, 1)
    time.sleep(0.000010)
    GPIO.output(TrigPin, 0)

    #get times
    while GPIO.input(EchoPin) == 0:
        StartTime = time.time() 

    while GPIO.input(EchoPin) == 1:
        StopTime = time.time()

    #calculate distance
    delay = StopTime - StartTime 
    distance = (delay * 34300) / 2
    
    return distance


def steering(angle):
    if angle > 100:
        print("specified angle too big: -100 to 100")
    elif angle < -100:
        print("specified angle too small: -100 to 100")
    else:
        DutyCycle = 3e-5 * angle**2 + 0.018 * angle + 6.57
        servo1.ChangeDutyCycle(DutyCycle)
        print(DutyCycle)


while True:
    steering(100)
    time.sleep(2)
    steering(0)
    time.sleep(2)
    steering(-100)
    time.sleep(2)


"""
while True:
    try:
        time.sleep(0.02)
        distance = Ultraschall()
        print(distance)
        if distance > 55:
            GPIO.output(F, 1)
            GPIO.output(R, 0)
        elif distance < 45:
            GPIO.output(R, 1)
            GPIO.output(F, 0)
        else:
            GPIO.output(R, 0)
            GPIO.output(F, 0)

    except KeyboardInterrupt:
        GPIO.cleanup()
"""
