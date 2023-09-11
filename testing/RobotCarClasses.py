import RPi.GPIO as GPIO
import time
import threading


class Motor:
    def __init__(self, frequency, fpin, rpin, spin):
        self.frequency = frequency
        self.fpin = fpin
        self.rpin = rpin
        self.spin = spin
        self.speed = 0
        
        #GPIO setup
        GPIO.setup(fpin, GPIO.OUT)
        GPIO.setup(rpin, GPIO.OUT)
        GPIO.setup(spin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(spin, frequency)

    def drive(self, direction, speed):
        self.speed = speed
        if direction == 'f':
            GPIO.output(self.fpin, 1)
            GPIO.output(self.rpin, 0)
        elif direction == 'r':
            GPIO.output(self.fpin, 0)
            GPIO.output(self.rpin, 1)
        else:
            print("no valid direction specified:" + direction)

        try:
            self.pwm.ChangeDutyCycle(speed)
        except:
            print(f"no valid speed value specified: {speed}")
            
    def start(self):
        self.pwm.start(self.speed)    
            
    def stop(self):
        GPIO.output(self.fpin, 0) 
        GPIO.output(self.rpin, 0)
        self.pwm.stop()
        
        
class SuperSonicSensor:
    def __init__(self, TrigPin, EchoPin):
        self.EchoPin = EchoPin
        self.TrigPin = TrigPin
        self.distance = 0
        
        #GPIO setup
        GPIO.setup(TrigPin, GPIO.OUT)
        GPIO.setup(EchoPin, GPIO.IN)
        
    def start_measurement(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.measure_distance, daemon=True)
        self.thread.start()
        
    def measure_distance(self):
        GPIO.setmode(GPIO.BOARD)
        while self.threadStop == 0:
            # Trigger
            GPIO.output(self.TrigPin, 0)
            time.sleep(0.000005)
            GPIO.output(self.TrigPin, 1)
            time.sleep(0.000010)
            GPIO.output(self.TrigPin, 0)

            # Get times
            while GPIO.input(self.EchoPin) == 0:
                StartTime = time.time()

            while GPIO.input(self.EchoPin) == 1:
                StopTime = time.time()

            # Calculate distance
            delay = StopTime - StartTime
            distance = (delay * 34300) / 2

            # Update self.dist
            self.distance = distance

            time.sleep(0.025)

    def stop_measurement(self):
        self.threadStop = 1
        
        
class Servo:
    def __init__(self, SignalPin, frequency):
        self.SignalPin = SignalPin
        self.frequency = frequency
        
        #GPIO setup
        GPIO.setup(self.SignalPin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(SignalPin, frequency)
        self.pwm.start(6.6)
        
    def steer(self, percentage):
        if percentage > 100:
            print("specified percentage too big: -100 to 100")
        elif percentage < -100:
            print("specified percentage too small: -100 to 100")
        else:
            DutyCycle = 3e-5 * percentage**2 + 0.018 * percentage + 6.57
            self.pwm.ChangeDutyCycle(DutyCycle)