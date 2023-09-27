import RPi.GPIO as GPIO
import time
import threading
from ctypes import *
from pixy import *
import pixy
import board
import adafruit_tcs34725
import os
import signal


##########################################################
##                                                      ##
##                     Classes                          ##
##                                                      ##
##########################################################

class CustomException(Exception):
    #raise and error and print the message at the same time
    
    def __init__(self, message):
        print("CustomExeption: "+message)
        super().__init__(message)
        

class Motor:
    def __init__(self, frequency, fpin, rpin, spin):
        self.frequency = frequency
        self.fpin = fpin
        self.rpin = rpin
        self.spin = spin
        self.speed = 0
        
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(fpin, GPIO.OUT)
        GPIO.setup(rpin, GPIO.OUT)
        GPIO.setup(spin, GPIO.OUT)
        
        try:
            self.pwm = GPIO.PWM(spin, frequency)
        except:
            print(f"no valid frequency specified {frequency}")
            raise CustomException(f"no valid frequency specified {frequency}")

    def drive(self, direction, speed=0):
        self.speed = speed
        if direction == 'f':
            GPIO.output(self.fpin, 1)
            GPIO.output(self.rpin, 0)
        elif direction == 'r':
            GPIO.output(self.fpin, 0)
            GPIO.output(self.rpin, 1)
        else:
            print(f"no valid direction specified: {direction}")
            raise CustomException(f"no valid direction specified: {direction}")

        try:
            self.pwm.ChangeDutyCycle(speed)
        except:
            print(f"no valid direction specified: {direction}")
            raise CustomException(f"no valid speed value specified: {speed}")
            
    def start(self):
        try:
            self.pwm.start(self.speed)  
        except:
            print(f"no valid speed value specified: {self.speed}")
            raise CustomException(f"no valid speed value specified: {self.speed}")  
            
    def stop(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.output(self.fpin, 0) 
        GPIO.output(self.rpin, 0)
        self.pwm.stop()
        
        
class SuperSonicSensor:
    def __init__(self, TrigPin, EchoPin, smoothing_window_size=40):
        self.EchoPin = EchoPin
        self.TrigPin = TrigPin
        self.distance = 0
        self.smoothing_window_size = smoothing_window_size
        self.sDistance = 0
        
        #Moving Average
        self.values = [0] * self.smoothing_window_size
        self.index = 0
        
        #GPIO setup
        GPIO.setup(TrigPin, GPIO.OUT)
        GPIO.setup(EchoPin, GPIO.IN)
        
    def start_measurement(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.measure_distance, daemon=True)
        self.thread.start()
        
    def measure_distance(self):
        MAXTIME = 0.01
        StartTime = 0
        StopTime = 0
        #GPIO.setmode(GPIO.BOARD)
        while self.threadStop == 0:
            # Trigger
            GPIO.output(self.TrigPin, 0)
            time.sleep(0.03)
            
            GPIO.output(self.TrigPin, 1)
            time.sleep(0.00001)
            GPIO.output(self.TrigPin, 0)

            # Get times
            TIMEOUT = time.time() + MAXTIME
            while GPIO.input(self.EchoPin) == 0 and StartTime < TIMEOUT:
                StartTime = time.time()
            
            if StartTime > TIMEOUT:
                print("Timeout1: ", StartTime > TIMEOUT)

            TIMEOUT = time.time() + MAXTIME
            while GPIO.input(self.EchoPin) == 1 and StopTime < TIMEOUT:
                StopTime = time.time()
            
            if StopTime > TIMEOUT:
                print("Timeout2: ", StopTime > TIMEOUT)
            

            # Calculate distance
            delay = StopTime - StartTime
            distance = (delay * 34300) / 2

            # Update self.dist
            self.distance = distance
            
            self.values[self.index] = self.distance
            self.index = (self.index + 1) % self.smoothing_window_size
            self.sDistance = sum(self.values) / self.smoothing_window_size

    def stop_measurement(self):
        self.threadStop = 1
        
        
class Servo:
    def __init__(self, SignalPin, frequency):
        self.SignalPin = SignalPin
        self.frequency = frequency
        
        #GPIO setup
        GPIO.setup(self.SignalPin, GPIO.OUT)
        
        try:
            self.pwm = GPIO.PWM(SignalPin, frequency)
            self.pwm.start(6.6)
        except:
            print(f"no valid frequency specified: {frequency}")
            raise CustomException(f"no valid frequency specified: {frequency}")
        
    def steer(self, percentage):
        try:
            DutyCycle = 3e-5 * percentage**2 + 0.018 * percentage + 6.57
            self.pwm.ChangeDutyCycle(DutyCycle)
        
        except:
            print(f"no valid steering percentage specified: {percentage}")
            raise CustomException(f"no valid steering percentage specified: {percentage}")
            
            
class PixyCam: #KOMM WIR WATCHEN NEN MUHFIEEEEEEEEEEEE
    def __init__(self):
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0
        self.threadStop = 0
        self.count = 0
        self.age = 0
        self.index = 0
        self.angle = 0
        self.YCutOffTop = YCutOffTop
        self.YCutOffBottom = YCutOffBottom
        
        pixy.init()
        pixy.change_prog ("color_connected_components");        
        
    def start_reading(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
        
    def read(self):
        class Blocks (Structure):
            _fields_ = [ ("m_signature", c_uint),
                ("m_x", c_uint),
                ("m_y", c_uint),
                ("m_width", c_uint),
                ("m_height", c_uint),
                ("m_angle", c_uint),
                ("m_index", c_uint),
                ("m_age", c_uint) ]
            
        self.output = BlockArray(100)
            
        while self.threadStop == 0:
            self.count = pixy.ccc_get_blocks(100, self.output)
            
    def stop_reading(self):
        self.threadStop = 1
        
    def LED(self, state):
        if state == 1:
            set_lamp(1, 1)
        elif state == 0:
            set_lamp(0, 0)
        else:
            print(f"no valid state specified: {state}")
            raise CustomException(f"no valid state specified: {state}")
            
            
class Button:
    def __init__(self, SignalPin):
        self.SignalPin = SignalPin
        #GPIO
        GPIO.setup(SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def state(self):
        if GPIO.input(self.SignalPin) == 0:
            return 1
        elif GPIO.input(self.SignalPin) == 1:
            return 0
        
    def start_StopButton(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read_StopButton, daemon=True)
        self.thread.start()
        
    def read_StopButton(self):
        while self.threadStop == 0:
            time.sleep(0.01)
            if self.state() == 1:
                os.kill(os.getpid(), signal.SIGINT)
                
    def stop_StopButton(self):
        self.threadStop = 1
        
         
class ColorSensor: #KOMM WIR WATCHEN NEN MUHFIEEEEEEEEEEEE
    def __init__(self):
        self.color_rgb = 0
        self.color_temperature = 0
        self.lux = 0
        
        
        #Colorsensor
        i2c = board.I2C()
        self.sensor = adafruit_tcs34725.TCS34725(i2c)
            
    def start_measurement(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
        
    def read(self):
        while self.threadStop == 0:
            time.sleep(0.01)
            self.color_rgb = self.sensor.color_rgb_bytes
            self.color_temperature = self.sensor.color_temperature
            self.lux = self.sensor.lux
        
    def stop_measurement(self):
        self.threadStop = 1


class Utility: #KOMM WIR WATCHEN 
    def __init__(self, Ultraschall1=None, Ultraschall2=None, Farbsensor=None, Motor1=None, Servo1=None, StartButton=None, StopButton=None, Pixy=None):
        self.Ultraschall1 = Ultraschall1
        self.Ultraschall2 = Ultraschall2
        self.Farbsensor = Farbsensor
        self.Motor1 = Motor1
        self.Servo1 = Servo1
        self.StartButton = StartButton
        self.StopButton = StopButton
        self.Pixy = Pixy
        
    def cleanup(self):
        GPIO.setmode(GPIO.BCM)
        if self.Ultraschall1:
            self.Ultraschall1.stop_measurement()
        if self.Ultraschall2:
            self.Ultraschall2.stop_measurement()
        if self.Farbsensor:
            self.Farbsensor.stop_measurement()
        if self.Motor1:
            self.Motor1.stop()
        
        self.StopButton.stop_StopButton()
        
        GPIO.cleanup()
        self.running = False
    
    def StartRun(self, MotorSpeed, steer=0, direction="f"):
        self.waiting = True
        while self.running and self.waiting:
            try:
                time.sleep(0.01)
                if self.StartButton.state() == 1:
                    self.StartTime = time.time()
                    print(f"Run started: {time.time()}")
                    
                    self.Motor1.drive(direction, MotorSpeed)
                    self.Servo1.steer(steer)
                    
                    self.waiting = False
                    
            except:
                self.cleanup()
                
    def StopRun(self):
        self.cleanup()
        self.StopTime = time.time()
        print(f"Run ended: {self.StopTime}")
        print(f"Time needed: {self.StopTime - self.StartTime}")


class Functions:
    def __init__(self, Utils):
        self.Utils = Utils
        self.Ultraschall1 = Utils.Ultraschall1
        self.Ultraschall2 = Utils.Ultraschall2
        self.Farbsensor = Utils.Farbsensor
        self.Motor1 = Utils.Motor1
        self.Servo1 = Utils.Servo1
        self.StartButton = Utils.StartButton
        self.StopButton = Utils.StopButton
        self.Pixy = Utils.Pixy
        self.rounds = 0
        self.corners = 0

    def HoldDistance(self, DISTANCE=50, HoldAtLine=False, P=5, speed=0, direction="f", colorTemperature=1, LineWaitTime=1):
        self.P = P
        self.Motor1.drive(direction, speed)
        driving = True
        TIMEOUT = 0
        
        
        
        while self.Utils.running and self.rounds < 3 and driving:
            time.sleep(0.01)
            #print(f"Distance1: {self.Ultraschall1.distance}, Distance2: {self.Ultraschall2.distance}")
            #print(self.Farbsensor.color_temperature)
            try:
                Error = self.Ultraschall1.distance - DISTANCE
                Correction = P * Error
                if Correction > 95:
                    Correction = 95
                elif Correction < -95:
                    Correction = -95
                    
                if direction == "f":
                    self.Servo1.steer(-Correction)
                elif direction == "r":
                    self.Servo1.steer(Correction)
                else:
                    print(f"no valid direction specified: {direction}")
                    raise CustomException(f"no valid direction specified: {direction}")
                
                if self.Farbsensor.color_temperature >= colorTemperature - 100 and self.Farbsensor.color_temperature <= colorTemperature + 100 and time.time() > TIMEOUT:
                    corners = corners + 1
                    if corners == 4:
                        corners = 0
                        self.rounds = self.rounds + 1
                        
                    if HoldAtLine == True:
                        driving = False
                        
                    TIMEOUT = time.time() + LineWaitTime
                    
                with open("sensor_data.txt", "a") as data_file:
                    data_file.write(f"rounds; {self.rounds}; corners; {corners}; Farbtemperatur; {self.Farbsensor.color_temperature}\n")
            except:
                self.Utils.cleanup()
            
    def HoldLane(self, YCutOffTop=0, YCutOffBottom=0, BlockWaitTime=1, LaneCoords=[1, 1], Lane=1, SIZE=1, HoldAtLine=False, P=5, speed=0, direction="f", colorTemperature=1, LineWaitTime=1):
        self.P = P
        self.Motor1.drive(direction, speed)
        driving = True
        TIMEOUT = 0
        TIMEOUTPixy = 0
        
        #clear the sensor data txt file at the start of the run
        with open("HoldLane.txt", "w") as data_file:
            data_file.write("")
     
        while self.Utils.running and self.rounds < 3 and driving:
            time.sleep(0.01)
            try:
                if Lane == 0:
                    DISTANCE = 25
                elif Lane == 1:
                    DISTANCE = 50
                elif Lane == 2:
                    DISTANCE = 75
                else:
                    print(f"No valid Lane specified: {Lane}")
                    raise CustomException(f"No valid Lane specified: {Lane}")
                
                #HoldLine
                Error = self.Ultraschall1.distance - DISTANCE
                Correction = P * Error
                if Correction > 95:
                    Correction = 95
                elif Correction < -95:
                    Correction = -95
                    
                if direction == "f":
                    self.Servo1.steer(-Correction)
                elif direction == "r":
                    self.Servo1.steer(Correction)
                else:
                    print(f"no valid direction specified: {direction}")
                    raise CustomException(f"no valid direction specified: {direction}")
                
                #Count rounds with ColorSensor
                if self.Farbsensor.color_temperature >= colorTemperature - 100 and self.Farbsensor.color_temperature <= colorTemperature + 100 and time.time() > TIMEOUT:
                    corners = corners + 1
                    if corners == 4:
                        corners = 0
                        self.rounds = self.rounds + 1
                        
                    if HoldAtLine == True:
                        driving = False
                        
                    TIMEOUT = time.time() + LineWaitTime
                    
                #get Pixy objects and calculate new lanes
                count = self.Pixy.count
                if count >= 1 and time.time() > TIMEOUTPixy:
                    NextObject = -1
                    for x in count:
                        yCoord = self.Pixy.output[x].m_y
                        
                        if yCoord > YCutOffTop or yCoord < YCutOffBottom:
                            
                            size = self.Pixy.output[x].m_width * self.Pixy.output[x].m_height
                            
                            if size >= SIZE:
                                NextObject = x
                                break
                            
                        else:
                            NextObject = -1
                    
                    if NextObject >= 0:
                        xCoord = self.Pixy.output[NextObject].m_x
                        if xCoord < LaneCoords[0]:
                            Lane = 0
                        elif xCoord > LaneCoords[1]:
                            Lane = 2
                        elif xCoord >= LaneCoords[0] and xCoord <= LaneCoords[1]:
                            Lane = 1
                        else:
                            print(f"Could not calculate new lane from xCoord: {xCoord}")
                            raise CustomException(f"Could not calculate new lane from xCoord: {xCoord}")
                    
                    TIMEOUTPixy = time.time() + BlockWaitTime
                    
                with open("HoldLine.txt", "a") as data_file:
                    data_file.write(f"rounds; {self.rounds}; corners; {corners}; Farbtemperatur; {self.Farbsensor.color_temperature}\n")
            except:
                self.Utils.cleanup()
                
    def DriveCorner(self, direction="f", speed=0, steer=0, wait=0, stop=True):
        self.Motor1.drive(direction, speed)
        self.Servo1.steer(steer)
        time.sleep(wait)
        if stop:
            self.Motor1.drive(direction, 0)
            self.Servo1.steer(0)
            
    def CalMiddle(self, CarWidth=17):
        self.middledistance = (self.Ultraschall1.distance + self.Ultraschall2.distance + CarWidth) / 2