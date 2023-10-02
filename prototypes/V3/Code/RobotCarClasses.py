import RPi.GPIO as GPIO
import time
import threading
from ctypes import *
from pixy import *
import pixy
import board, adafruit_tcs34725, adafruit_mpu6050
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import os, signal, socket
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from gpiozero import CPUTemperature
import psutil
import busio

os.system('cls' if os.name == 'nt' else 'clear')

global printlist

printlist = []

##########################################################
##                                                      ##
##                     Classes                          ##
##                                                      ##
##########################################################

class CustomException(Exception):
    def __init__(self, message):
        print(message)  # Print the custom message
        super().__init__(message)


class UtilityFunctions:
    
    
    def convert_to_two_decimal_point(self, number):
        # Format CPU temperature with 2 decimal places
        number = float(number)
        formated_string = f"{number:.2f}"

        # Check if there is a decimal point
        if '.' in formated_string:
            integer_part, decimal_part = formated_string.split('.')
            
            # Add a zero if there's only one decimal place
            if len(decimal_part) == 1:
                formated_string += '0'
        else:
            # Add ".00" if there are no decimal places
            formated_string += '.00'

        return formated_string
    
    def convert_to_one_decimal_point(self, number):
        return round(float(number), 1)
    
 
        

class Motor:
    def __init__(self, frequency, fpin, rpin, spin):
        #setup Variables
        self.frequency, self.fpin, self.rpin, self.spin, self.speed = frequency, fpin, rpin, spin, 0
        
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
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.fpin, GPIO.OUT)
        GPIO.setup(self.rpin, GPIO.OUT)
        GPIO.setup(self.spin, GPIO.OUT)

        self.speed = speed
        
        #set direction
        if direction == 'f':
            GPIO.output(self.fpin, 1)
            GPIO.output(self.rpin, 0)
        elif direction == 'r':
            GPIO.output(self.fpin, 0)
            GPIO.output(self.rpin, 1)
        else:
            print(f"no valid direction specified: {direction}")
            raise CustomException(f"no valid direction specified: {direction}")

        #set speed
        try:
            self.pwm.ChangeDutyCycle(speed)
        except:
            print(f"no valid direction specified: {direction}")
            raise CustomException(f"no valid speed value specified: {speed}")
            
    def start(self):
        #Start Motor
        try:
            self.pwm.start(self.speed)  
        except:
            print(f"no valid speed value specified: {self.speed}")
            raise CustomException(f"no valid speed value specified: {self.speed}")  
            
    def stop(self):
        #Stop Motor
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.output(self.fpin, 0) 
        GPIO.output(self.rpin, 0)
        self.pwm.stop()
        
        
class SuperSonicSensor:
    def __init__(self, TrigPin, EchoPin, ID=0, smoothing_window_size=3):
        #Variables
        self.EchoPin, self.TrigPin, self.distance, self.smoothing_window_size, self.sDistance = EchoPin, TrigPin, 0, smoothing_window_size, 0
        self.ID = ID
        
        #Moving Average
        self.values = [0] * self.smoothing_window_size
        self.index = 0
        
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TrigPin, GPIO.OUT)
        GPIO.setup(EchoPin, GPIO.IN)
        
    def start_measurement(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.measure_distance, daemon=True)
        self.thread.start()
        
    def measure_distance(self):
        #Variables
        MAXTIME = 0.04
        StartTime = 0
        StopTime = 0
        
        while self.threadStop == 0:
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.TrigPin, GPIO.OUT)
            GPIO.setup(self.EchoPin, GPIO.IN)
            # Trigger
            GPIO.output(self.TrigPin, 0)
            time.sleep(0.003)
            
            GPIO.output(self.TrigPin, 1)
            time.sleep(0.00001)
            GPIO.output(self.TrigPin, 0)

            # Get times
            TIMEOUT = time.time() + MAXTIME      
            while GPIO.input(self.EchoPin) == 0 and StopTime < TIMEOUT:
                StartTime = time.time()
                
            TIMEOUT = time.time() + MAXTIME
            while GPIO.input(self.EchoPin) == 1 and StopTime < TIMEOUT:
                StopTime = time.time()

            # Calculate distance
            delay = StopTime - StartTime
            distance = (delay * 34030) / 2

            # Update self.dist
            self.distance = distance
            #print(f"Measured distance: {self.distance} my ID is {self.ID}")
            """
            self.values[self.index] = self.distance
            self.index = (self.index + 1) % self.smoothing_window_size
            self.sDistance = sum(self.values) / self.smoothing_window_size
            """

    def stop_measurement(self):
        self.threadStop = 1
        
        
class Servo:
    def __init__(self, SignalPin, frequency):
        self.SignalPin, self.frequency = SignalPin, frequency
        
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SignalPin, GPIO.OUT)
        
        #Start steering at 0
        try:
            self.pwm = GPIO.PWM(SignalPin, frequency)
            self.pwm.start(6.6)
        except:
            print(f"no valid frequency specified: {frequency}")
            raise CustomException(f"no valid frequency specified: {frequency}")
        
    def steer(self, percentage):
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SignalPin, GPIO.OUT)
        
        #calculate DutyCycle and set it
        try:
            DutyCycle = 3e-5 * percentage**2 + 0.018 * percentage + 6.57
            self.pwm.ChangeDutyCycle(DutyCycle)
        except:
            print(f"no valid steering percentage specified: {percentage}")
            raise CustomException(f"no valid steering percentage specified: {percentage}")
            
            
class PixyCam:
    def __init__(self):
        self.count, self.output = 0, ()
        
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
        #Variables
        self.SignalPin = SignalPin
        
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def state(self):
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        #read button state
        if GPIO.input(self.SignalPin) == 0:
            return 1
        elif GPIO.input(self.SignalPin) == 1:
            return 0
        
    def start_StopButton(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read_StopButton, daemon=True)
        self.thread.start()
        
    def read_StopButton(self):
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        #stop program if stopbutton is pressed
        while self.threadStop == 0:
            time.sleep(0.1)
            if self.state() == 1:
                print("StopButton pressed")
                os.kill(os.getpid(), signal.SIGINT)
                
    def stop_StopButton(self):
        self.threadStop = 1
        
         
class ColorSensor:
    def __init__(self):
        self.color_rgb, self.color_temperature, self.lux = 0, 0, 0
        
        #Colorsensor init
        i2c = board.I2C()
        self.sensor = adafruit_tcs34725.TCS34725(i2c)
            
    def start_measurement(self):
        self.sensor.active = True
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
        
    def read(self):
        #Write sensor data to variables
        while self.threadStop == 0:
            time.sleep(0.003)
            self.color_temperature = self.sensor.color_temperature
            #self.color_rgb = self.sensor.color_rgb_bytes
            #self.lux = self.sensor.lux
        
    def stop_measurement(self):
        self.sensor.active = False
        self.threadStop = 1
        
        
class Gyroscope:
    def __init__(self):
        i2c = board.I2C()
        self.sensor = adafruit_mpu6050.MPU6050(i2c)
        self.sensor._gyro_range = 0

        # Initialize variables for storing the angle and time
        self.angle = 0.0  # Initial angle
        self.last_time = time.time()

    def get_angle(self):
        # Read gyroscope data
        gyro_data = self.sensor.gyro

        # Get the current time
        current_time = time.time()

        # Calculate the time elapsed since the last measurement
        delta_time = current_time - self.last_time

        # Integrate the gyroscope readings to get the change in angle
        if gyro_data[0] < 0.02 and gyro_data[0] > -0.02:
            gyro_data = 0
        else:
            gyro_data = gyro_data[0]
            
        delta_angle = gyro_data * delta_time

        # Update the angle
        self.angle += delta_angle

        # Update the last time for the next iteration
        self.last_time = current_time

        return self.angle * 210
        
        
class AnalogDigitalConverter:
    def __init__(self):
        i2c = busio.I2C(board.D1, board.D0)
        self.ads = ADS.ADS1015(i2c)
        self.ads.active = True
        self.chan = AnalogIn(self.ads, ADS.P0)
        
    def read(self):
        return self.chan.voltage * 4.35


class DisplayOled(UtilityFunctions):
    def __init__(self, ADC=None):
        serial = i2c(port=0, address=0x3C)
        self.device = sh1106(serial)
        
        self.first_line = ""
        self.second_line = ""
        self.ADC = ADC
        
        with canvas(self.device) as draw:
            draw.rectangle(self.device.bounding_box, outline="white", fill="black")
        
    def clear(self):
        with canvas(self.device) as draw:
            draw.rectangle(self.device.bounding_box, outline="white", fill="black")
        
    def write(self, first_line="", second_line="",reset=False, xCoord=0, yCoord=17):
        if reset:
            self.first_line = ""
            self.second_line = ""
        
        if first_line != "":
            self.first_line = first_line
        if second_line != "":
            self.second_line = second_line
                
    def start_update(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self): #Guys wir watchen nen Movie
        while self.threadStop == 0:
            cpuTemp = CPUTemperature()

            self.cpu_usage = psutil.cpu_percent(interval=0)
            self.ram = psutil.virtual_memory()
            self.disk = psutil.disk_usage('/')
            
            cpu_temp_formatted = self.convert_to_one_decimal_point(cpuTemp.temperature)
            cpu_usage_formatted = self.convert_to_one_decimal_point(self.cpu_usage)
            ram_usage_formatted = self.convert_to_one_decimal_point(self.ram.percent)
            disk_usage_formatted = self.convert_to_one_decimal_point(self.disk.percent)
            voltage_value_formatted = self.convert_to_two_decimal_point(self.ADC.read())
            
            
            with canvas(self.device) as draw:
                #top
                draw.text((0, 0), f"{cpu_temp_formatted}°C", fill="white", align="left")
                draw.text((40, 0), f"DISK:{int(disk_usage_formatted)}%", fill="white")
                draw.text((92, 0), f"{voltage_value_formatted}V", fill="white")
                
                #bottom
                draw.text((0, 50), f"CPU:{cpu_usage_formatted}%", fill="white")
                draw.text((75, 50), f"RAM:{ram_usage_formatted}%", fill="white")
                
                #custom
                draw.multiline_text((0, 15), f"{self.first_line}\n{self.second_line}", fill="white", align="center", anchor="ma")
            
            #time.sleep(0.3)
            
    def stop_update(self):
        self.threadStop = 1


class Utility:
    def __init__(self, Funcs, Ultraschall1=None, Ultraschall2=None, Farbsensor=None, Motor1=None, Servo1=None, StartButton=None, StopButton=None, Pixy=None):
        self.Ultraschall1, self.Ultraschall2, self.Farbsensor, self.Motor1, self.Servo1, self.StartButton, self.StopButton, self.Pixy, self.Funcs = Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy, Funcs

    def cleanup(self):
        print("Started cleanup")
        print(self.Funcs.rounds, self.running)
        #Stop all Threads and cleanup GPIO
        self.Ultraschall1.stop_measurement()
        self.Ultraschall2.stop_measurement()
        
        self.Farbsensor.stop_measurement()

        self.Motor1.drive("f", 0)
        self.Motor1.stop()
        
        self.StopButton.stop_StopButton()
        
        time.sleep(0.1)
        GPIO.cleanup()
        self.running = False
        print("Finished cleanup")
    
    def StartRun(self, MotorSpeed, steer=0, direction="f"):
        #Wait until StartButton is pressed
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
        #Stop Motor and cleanup
        self.Motor1.drive("f", 0)
        self.cleanup()
        self.StopTime = time.time()
        print(f"Run ended: {self.StopTime}")
        seconds = round(self.StopTime - self.StartTime, 2)
        
        #Print time needed
        minutes = seconds // 60
        hours = minutes // 60
        if hours > 0:
            print(f"{hours} hour(s), {minutes % 60} minute(s), {seconds % 60} second(s) needed")
        elif minutes > 0:
            print(f"{minutes} minute(s), {seconds % 60} second(s) needed")
        else:
            print(f"{seconds} second(s) needed")
            
    def log(self):
        try:
            data = f"Distance1; {self.Ultraschall1.distance}; sDistance1; {self.Ultraschall1.sDistance}; Distance2; {self.Ultraschall2.distance}; sDistance2; {self.Ultraschall2.sDistance}; Farbtemperatur; {self.Farbsensor.color_temperature}; rounds; {self.Funcs.rounds}"
            with open("Sensor_Output.txt", "a") as data_file:
                data_file.write(data + "\n")
                
        except Exception as e:
            print(f"Could not write data to file: {e}")


class Functions:
    def __init__(self, Ultraschall1=None, Ultraschall2=None, Farbsensor=None, Motor1=None, Servo1=None, StartButton=None, StopButton=None, Pixy=None):
        self.Ultraschall1, self.Ultraschall2, self.Farbsensor, self.Motor1, self.Servo1, self.StartButton, self.StopButton, self.Pixy, self.rounds, self.corners = Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy, 0, 0
        
    def information(self, Utils):
        #Get Utils, because it is initialized after Functions
        self.Utils = Utils

    def HoldDistance(self, DISTANCE=50, HoldAtLine=False, P=5, speed=0, direction="f", colorTemperature=1, LineWaitTime=1):
        #Variables
        self.P = P
        self.Motor1.drive(direction, speed)
        driving = True
        TIMEOUT = 0
        corners = 0

        #Clear data file
        with open("Sensor_Output.txt", "w") as data_file:
            data_file.write("")
        
        #Hold Distance to wall
        while self.Utils.running and self.rounds < 3 and driving:
            try:
                time.sleep(0.05)
                print(self.Ultraschall2.distance)
                Error = self.Ultraschall2.distance - DISTANCE
                Correction = P * Error
                if Correction > 95:
                    Correction = 95
                elif Correction < -95:
                    Correction = -95
                
                if direction == "f":
                    self.Servo1.steer(Correction)
                elif direction == "r":
                    self.Servo1.steer(-Correction)
                else:
                    print(f"no valid direction specified: {direction}")
                    raise CustomException(f"no valid direction specified: {direction}")
                
                #Count rounds with ColorSensor
                if self.Farbsensor.color_temperature >= colorTemperature - 100 and self.Farbsensor.color_temperature <= colorTemperature + 100 and time.time() > TIMEOUT:
                    corners = corners + 1
                    if corners == 4:
                        corners = 0
                        self.rounds = self.rounds + 1
                        print(f"Round: {self.rounds}")
                        
                    if HoldAtLine == True:
                        driving = False
                        
                    TIMEOUT = time.time() + LineWaitTime
                    
                #print(-Correction)
                #self.Utils.log()
            except Exception as e:
                print(e)
                self.Utils.cleanup()
            
    def HoldLane(self, YCutOffTop=200, YCutOffBottom=0, BlockWaitTime=1, Lane=1, SIZE=1, HoldAtLine=False, P=7, speed=65, direction="f", colorTemperature=1, LineWaitTime=1, Sensor=2):
        #Variables
        self.P = P
        self.Motor1.drive(direction, speed)
        driving = True
        TIMEOUT = 0
        TIMEOUTPixy = 0

        #Hold Lane
        while self.Utils.running and self.rounds < 3 and driving:
            try:
                time.sleep(0.05)
                if Lane == 0:
                    DISTANCE = 25
                    Sensor = 2
                elif Lane == 1:
                    DISTANCE = 50
                    Sensor = 2
                elif Lane == 2:
                    DISTANCE = 25
                    Sensor = 1
                else:
                    print(f"No valid Lane specified: {Lane}")
                    raise CustomException(f"No valid Lane specified: {Lane}")

                #HoldLine
                if Sensor == 1:
                    print(self.Ultraschall1.distance)
                    Error = self.Ultraschall1.sDistance - DISTANCE
                    Correction = P * Error
                    if Correction > 95:
                        Correction = 95
                    elif Correction < -95:
                        Correction = -95
                        
                    print(Correction)
                    if direction == "f":
                        self.Servo1.steer(-Correction)
                    elif direction == "r":
                        self.Servo1.steer(Correction)
                    else:
                        print(f"no valid direction specified: {direction}")
                        raise CustomException(f"no valid direction specified: {direction}")  
                elif Sensor == 2:
                    print(self.Ultraschall2.distance)
                    Error = self.Ultraschall2.sDistance - DISTANCE
                    Correction = P * Error
                    if Correction > 95:
                        Correction = 95
                    elif Correction < -95:
                        Correction = -95
                        
                    print(Correction)
                    if direction == "f":
                        self.Servo1.steer(Correction)
                    elif direction == "r":
                        self.Servo1.steer(-Correction)
                    else:
                        print(f"no valid direction specified: {direction}")
                        raise CustomException(f"no valid direction specified: {direction}")
                else:
                    print(f"No valid Sensor specified: {Sensor}")
                    raise CustomException(f"No valid Sensor specified: {Sensor}")
                
                #Count rounds with ColorSensor
                if self.Farbsensor.color_temperature >= colorTemperature - 100 and self.Farbsensor.color_temperature <= colorTemperature + 100 and time.time() > TIMEOUT:
                    corners = corners + 1
                    if corners == 4:
                        corners = 0
                        self.rounds = self.rounds + 1
                        
                    if HoldAtLine == True:
                        driving = False
                        
                    TIMEOUT = time.time() + LineWaitTime
                    
                #get Pixy objects and calculate new lanes#
                if time.time() > TIMEOUTPixy:
                    count = self.Pixy.count
                    if count >= 1:
                        NextObject = -1
                        for x in range(count):
                            yCoord = self.Pixy.output[x].m_y
                            
                            if yCoord < YCutOffTop and yCoord > YCutOffBottom:
                                
                                size = self.Pixy.output[x].m_width * self.Pixy.output[x].m_height
                                
                                if size >= SIZE:
                                    NextObject = x
                                    break
                                
                            else:
                                NextObject = -1
                        
                        if NextObject >= 0:
                            signature = self.Pixy.output[NextObject].m_signature
                            print(signature)
                            if signature == 1:
                                Lane = 0
                            elif signature == 2:
                                Lane = 2
                            else:
                                Lane = 1

                            TIMEOUTPixy = time.time() + BlockWaitTime

                    else:
                        Lane = 1
                        
                    
            except Exception as e:
                print(e)
                self.Utils.cleanup()
      
"""          
    def DriveCorner(self, direction="f", speed=0, steer=0, wait=0, stop=True):
        self.Motor1.drive(direction, speed)
        self.Servo1.steer(steer)
        time.sleep(wait)
        if stop:
            self.Motor1.drive(direction, 0)
            self.Servo1.steer(0)
            
    def CalMiddle(self, CarWidth=17):
        self.middledistance = (self.Ultraschall1.distance + self.Ultraschall2.distance + CarWidth) / 2
"""
