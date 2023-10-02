import time
import RPi.GPIO as GPIO

TrigPin = 23
EchoPin = 24
MAXTIME = 0.04
StartTime = 0
StopTime = 0

while True:
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(EchoPin, GPIO.IN)
    # Trigger
    GPIO.output(TrigPin, 0)
    time.sleep(0.03)

    GPIO.output(TrigPin, 1)
    time.sleep(0.00001)
    GPIO.output(TrigPin, 0)

    # Get times
    TIMEOUT = time.time() + MAXTIME      
    while GPIO.input(EchoPin) == 0:
        StartTime = time.time()
        
    TIMEOUT = time.time() + MAXTIME
    while GPIO.input(EchoPin) == 1:
        StopTime = time.time()

    # Calculate distance
    delay = StopTime - StartTime
    distance = (delay * 34030) / 2
    print(delay)
    print(distance)
    time.sleep(0.5)
    