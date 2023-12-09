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
import multiprocessing as mp
import logging
import serial
import subprocess



##########################################################
##                                                      ##
##                     Classes                          ##
##                                                      ##
##########################################################

#A class that can be called to raise a custom exception and self.Utils.LogError a custom message
class CustomException(Exception):
    def __init__(self, message):
        self.Utils.LogError(message)  # self.Utils.LogError the custom message



#A class that has some necessary tools for calculating, usw.
class Utility:
    #Transfer data so it can be used in other classes
    def transferSensorData(self, Farbsensor=None, StartButton=None, StopButton=None, Display=None, ADC=None, Buzzer1=None, Pixy=None):
        #self.Ultraschall1, self.Ultraschall2, self.Farbsensor, self.Motor1, self.Servo1, self.StartButton, self.StopButton, self.Pixy, self.Funcs = Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy, Funcs
        self.Farbsensor = Farbsensor
        self.StartButton = StartButton
        self.StopButton = StopButton
        self.Pixy = Pixy
        self.Display = Display
        self.ADC = ADC
        self.Buzzer1 = Buzzer1
        self.StartTime = time.time()
        
        self.ActivSensor = 0
        self.file_path = "/tmp/StandbyScript.lock"
        self.Starttime = 0
        self.UltraschallThreadStop = 1
        
        
    #Cleanup after the run is finished or an error occured
    def cleanup(self):
        self.LogDebug("Started cleanup")
        
        
        #Stop all Threads if the class has been initialized
        if self.Farbsensor != None:
            self.Farbsensor.stop_measurement()
        if self.Pixy != None:
            self.Pixy.stop_reading()
        if self.Display != None:
            self.Display.stop_update()
        if self.StopButton != None:
            self.StopButton.stop_StopButton()
        
        #Stop Nodemcu's
        self.EspHoldDistance.write(f"STOP\n".encode())
        time.sleep(0.1)
        self.EspHoldSpeed.write(f"STOP\n".encode())
        
        #self.StopNodemcus()
        
        #Wait a short time to make sure all threads are stopped
        self.Buzzer1.buzz(1000, 80, 0.5)
        
        #GPIO cleanup and Program exit
        GPIO.cleanup()
        self.running = False
        
        time.sleep(0.1)
        self.EspHoldDistance.close()
        time.sleep(0.1)
        self.EspHoldSpeed.close()
        
        os.kill(os.getpid(), signal.SIGTERM)
    
    
    #Do some init and wait until StartButton is pressed
    def StartRun(self):
        #clear console
        os.system('cls' if os.name=='nt' else 'clear')

        self.InitNodemcus()

        #Start Processes
        if self.Display != None:
            p1 = mp.Process(target=self.Display.start_update())
            p1.start()
        if self.StopButton != None:
            p2 = mp.Process(target=self.StopButton.start_StopButton())
            p2.start()
        if self.Farbsensor != None:
            p3 = mp.Process(target=self.Farbsensor.start_measurement())
            p3.start()
        if self.Pixy != None:
            p4 = mp.Process(target=self.Pixy.start_reading())
            p4.start()

        #Wait for StartButton to be pressed
        self.running = True
        self.waiting = True
        
        self.LogDebug("Waiting for Button to be pressed...")
        self.Display.write("Waiting for Button", "to be pressed...")
        
        self.Buzzer1.buzz(1000, 80, 0.1)
        time.sleep(0.1)
        self.Buzzer1.buzz(1000, 80, 0.1)
        
        while self.running and self.waiting:
            try:
                time.sleep(0.1)
                if self.StartButton.state() == 1:
                    
                    self.StartNodemcus()
                    self.Gyro.angle = 0
                    
                    self.Starttime = time.time()
                    self.LogDebug(f"Run started: {time.time()}")
                    self.Display.write("Run started:", f"{time.time()}")  
                    self.Buzzer1.buzz(1000, 80, 0.1) 

                    self.waiting = False
                    
            except:
                self.Utils.StopRun()
    
    
    #Stop the run and calculate the time needed            
    def StopRun(self):
        self.StopTime = time.time()
        self.LogDebug(f"Run ended: {self.StopTime}")
        
        if self.Starttime != None:
            seconds = round(self.StopTime - self.StartTime, 2)
        
            #self.Utils.LogError time needed
            minutes = seconds // 60
            hours = minutes // 60
            if hours > 0:
                self.LogDebug(f"{hours} hour(s), {minutes % 60} minute(s), {seconds % 60} second(s) needed")
                self.Display.write("Time needed:", f"{hours}h {minutes % 60}m {seconds % 60}s")
            elif minutes > 0:
                self.LogDebug(f"{minutes} minute(s), {seconds % 60} second(s) needed")
                self.Display.write("Time needed:", f"{minutes}m {seconds % 60}s")
            else:
                self.LogDebug(f"{seconds} second(s) needed")
                self.Display.write("Time needed:", f"{seconds}s")
            
        self.cleanup()
          
    
    #Setup datalogging
    def setupDataLog(self):
        try:
            #Clear DataLog
            #os.remove("DataLog.log")
                
            #Create datalogger
            self.datalogger = logging.getLogger("DataLogger")
            self.datalogger.setLevel(logging.DEBUG)

            
            #Create file handler and set level to debug
            fh = logging.FileHandler("DataLog.log", 'w')
            fh.setLevel(logging.DEBUG)
            self.datalogger.addHandler(fh)

            #Start Process to log data while the program is running
            self.DataLoggerStop = 0
            PDataLogger = mp.Process(target=self.LogData)
            PDataLogger.start()
            
        except Exception as e:
            self.LogError(f"An Error occured in Utility.setupDataLog: {e}")
            self.Utils.StopRun()
            

    #Log Sensor values
    def LogData(self):
        try:
            self.datalogger.debug(f"Farbsensor; {self.Farbsensor.color_temperature};  CPU; {psutil.cpu_percent()}; RAM; {psutil.virtual_memory().percent}; CPUTemp; {CPUTemperature().temperature}; Voltage; {self.ADC.voltage}")
        except Exception as e:
            self.LogError(f"An Error occured in Utility.LogData: {e}")
            self.Utils.StopRun()
        
    
    #Stop the DataLogger Process
    def StopDataLog(self):
        self.DataLoggerStop = 1
        
        
    #Setup logging
    def setupLog(self, name='Log', filename='Debug.log'):
        try:
            #Create logger
            self.logger = logging.getLogger(name)
            self.logger.setLevel(logging.DEBUG)
            
            #Create console handler and set level to debug
            ch = logging.StreamHandler()
            ch.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            
            #Create file handler and set level to debug
            fh = logging.FileHandler(filename, 'w')
            fh.setLevel(logging.DEBUG)
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            
            #Add handlers to logger
            self.logger.addHandler(ch)
            self.logger.addHandler(fh)
        except Exception as e:
            self.LogError(f"An Error occured in Utility.setupLog: {e}")
            self.Utils.StopRun()
        
        
    #Log debug messages
    def LogDebug(self, message):
        self.logger.debug(message)
        
    def LogInfo(self, message):
        self.logger.info(message)
        
    def LogWarning(self, message):
        self.logger.warning(message)
        
    def LogError(self, message):
        self.logger.error(message)
        
    def LogCritical(self, message):
        self.logger.critical(message)

    
    #Convert a number to a specified number of decimal points
    def convert_to_decimal_points(self, number, decimal_points):
        try:
            number = float(number)
            formatted_string = f"{number:.{decimal_points}f}"
            return formatted_string
        except Exception as e:
            self.LogError(f"Could not convert number to {decimal_points} decimal points: {e}")
            # Handle the error or return an appropriate value if needed


    #Convert a number to a specified number of digits
    def convert_to_specified_digits(self, number, num_digits):
        # Check if the input number is a valid float or int
        if not isinstance(number, (float, int)):
            raise CustomException(f"Invalid input: Please provide a valid number: {number}")
        
        # Convert the number to a string
        num_str = str(number)
        
        # Split the number into its integer and decimal parts
        if '.' in num_str:
            integer_part, decimal_part = num_str.split('.')
        else:
            integer_part, decimal_part = num_str, '0'
        
        # If the integer part has more digits than the specified number, return it as is
        if len(integer_part) >= num_digits:
            return integer_part
        
        # Otherwise, pad the integer part with leading zeros to match the specified number of digits
        padded_integer_part = integer_part.zfill(num_digits)
        
        # Reconstruct the final number with the decimal part
        final_number = f"{padded_integer_part}.{decimal_part}"
        
        return final_number


    #Start a new thread for reading the sensor    
    def toggle_supersonic_sensor(self, ID):
        try: 
            #Turn off both sensors
            if ID == 0:
                self.UltraschallThreadStop = 1
                
            #Toggle to sensor left
            if ID == 1:
                self.UltraschallThreadStop = 1
                   
                time.sleep(0.1)     
                p_Ultra1 = mp.Process(target=self.Ultraschall1.start_measurement())
                p_Ultra1.start()
                
                return
            
            #Toggle to sensor right
            if ID == 2:
                self.UltraschallThreadStop = 1
                   
                time.sleep(0.1)
                p_Ultra2 = mp.Process(target=self.Ultraschall2.start_measurement())
                p_Ultra2.start()

                return
            
        except Exception as e:
            self.LogError(f"An Error occured in Utility.toggle_supersonic_sensor: {e}")
            self.Utils.StopRun()
    
    
    #Init both NodeMCUs
    def InitNodemcus(self):
        usb_devices = []
        try:
            # Run the 'ls /dev/tty*' command using a shell and capture the output
            result = subprocess.run('ls /dev/tty*', shell=True, stdout=subprocess.PIPE, text=True, check=True)
            
            # Split the output into lines and print each line
            devices = result.stdout.split('\n')
            for device in devices:
                if "/dev/ttyUSB" in device:
                    usb_devices.append(device)
            
        except Exception as e:
            self.LogError(f"An Error occured in Utility.StartNodemcus: {e}")
            self.Utils.StopRun()

        if len(usb_devices) != 2:
            self.LogError(f"Could not find both NodeMCUs: {usb_devices}")
            self.Utils.StopRun()
            
        #Identify both NodeMCUs
        for device in usb_devices:
            try:
                ESP = serial.Serial(device,baudrate=115200,timeout=1)
                ESP.write(f"IDENT\n".encode())
                time.sleep(0.1)
                
                #wait for response
                Timeout = time.time() + 5
                self.LogDebug(f"Waiting for response from {device} ...")
                while not ESP.in_waiting and time.time() < Timeout:
                    time.sleep(0.01)
                response = ESP.read(ESP.inWaiting())
                print(response)
                self.LogDebug(f"Received response from {device}")
                
                if "HoldDistance" in response.decode("utf-8"):
                    ESP.close()
                    self.EspHoldDistance = serial.Serial(device,baudrate=115200,timeout=1)
                elif "HoldSpeed" in response.decode("utf-8"):
                    ESP.close()
                    self.EspHoldSpeed = serial.Serial(device,baudrate=115200,timeout=1)  
                else:
                    self.LogError(f"Could not identify NodeMCU on: {device}")
                    self.Utils.StopRun()
                
                time.sleep(0.1)
                    
            except Exception as e:
                self.LogError(f"An Error occured in Utility.StartNodemcus: {e}")
                self.Utils.StopRun()
    
    
    #Start both NodeMCUs and wait for responses
    def StartNodemcus(self):
        #Start both NodeMCUs
        for esp in [self.EspHoldDistance, self.EspHoldSpeed]:
            esp.write(f"START\n".encode())
            time.sleep(0.1)
            waitingForResponse = True
            responseTimeout = time.time() + 5

            while waitingForResponse:
                try:
                    response = esp.read(esp.inWaiting())
                    if "Received START command. Performing action..." in response.decode("utf-8"):
                        waitingForResponse = False
                    elif time.time() > responseTimeout:
                        self.LogError("No response from NodeMCU")
                        self.StopRun()
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    self.LogError(f"An exception occurred in Utility.StartRun: {e}")
                    self.StopRun()
                    
            time.sleep(0.1)
                    
                    
    #Stop both NodeMCUs and wait for responses
    def StopNodemcus(self):
        for esp in [self.EspHoldDistance, self.EspHoldSpeed]:
            esp.flush()
            esp.write(f"STOP\n".encode())
            waitingForResponse = True
            responseTimeout = time.time() + 5

            while waitingForResponse:
                try:
                    response = esp.read(esp.inWaiting())
                    if "Received STOP command. Performing action..." in response.decode("utf-8"):
                        waitingForResponse = False
                    elif time.time() > responseTimeout:
                        self.LogError("No response from NodeMCU")
                        self.StopRun()
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    self.LogError(f"An exception occurred in Utility.StartRun: {e}")
                    self.StopRun()
        
        time.sleep(0.1)
        
    
#Class for the drive Motor
class Motor(Utility):
    def __init__(self, frequency, fpin, rpin, spin, Utils):
        try:
            #Setup Variables
            self.frequency, self.fpin, self.rpin, self.spin, self.speed = frequency, fpin, rpin, spin, 0
            self.Utils = Utils
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(fpin, GPIO.OUT)
            GPIO.setup(rpin, GPIO.OUT)
            GPIO.setup(spin, GPIO.OUT)
            
            self.pwm = GPIO.PWM(self.spin, self.frequency)
            self.MotorSpeed = 0
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Motor initialization: {e}")
            self.Utils.StopRun()


    #Set the direction and speed of the Motor
    def drive(self, direction="f", speed=0):
        try:
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.fpin, GPIO.OUT)
            GPIO.setup(self.rpin, GPIO.OUT)

            self.speed = speed
            
            #Set direction
            if direction == 'f':
                GPIO.output(self.fpin, 1)
                GPIO.output(self.rpin, 0)
            elif direction == 'r':
                GPIO.output(self.fpin, 0)
                GPIO.output(self.rpin, 1)
            else:
                self.Utils.LogError(f"No valid direction specified: {direction}")
                raise CustomException(f"No valid direction specified: {direction}")

            #Set speed
            self.pwm.ChangeDutyCycle(speed)
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Motor.drive: {e}")
            self.Utils.StopRun()

       
    #Start the Motor with the last known speed        
    def start(self):
        try:
            self.pwm.start(self.speed)  
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Motor.start: {e}")
            self.Utils.StopRun()
       
       
    #Stop the Motor        
    def stop(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.output(self.fpin, 0) 
            GPIO.output(self.rpin, 0)
            self.pwm.stop()
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Motor.stop: {e}")
            self.Utils.StopRun()
        
    
    #Set a motor speed that gets controlled by the Speed Sensor, if it was started   
    def setMotorSpeed(self, Speed):
        self.drive("f", 100)
        time.sleep(0.01)
        self.MotorSpeed = Speed
        


#Class for reading a SuperSonicSensor
class SuperSonicSensor(Utility):
    def __init__(self, TrigPin, EchoPin, ID, Utils, smoothing_window_size=7):
        try:
            #Variable init
            self.EchoPin, self.TrigPin, self.distance, self.smoothing_window_size, self.sDistance = EchoPin, TrigPin, 0, smoothing_window_size, 0
            self.ID = ID
            self.ThreadStop = 0
            self.Utils = Utils
            #Moving Average init
            self.values = [0] * self.smoothing_window_size
            self.index = 0
            
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(TrigPin, GPIO.OUT)
            GPIO.setup(EchoPin, GPIO.IN)
            
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SuperSonicSensor initialization: {e}")
            self.Utils.StopRun()
        
    
    #Start a new thread for measuring the sensor   
    def start_measurement(self):
        try:
            #GPIO.remove_event_detect(self.EchoPin)
            self.Utils.UltraschallThreadStop = 0
            self.thread = threading.Thread(target=self.measure_distance, daemon=True, name=f"Ultraschall{self.ID}", args=(self.Utils,))
            self.thread.start()
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SuperSonicSensor.start_measurement: {e}")
            self.Utils.StopRun()
    
    
    #measure the distance with the sensor
    def measure_distance(self, Utils):
        GPIO.remove_event_detect(self.EchoPin)
        GPIO.add_event_detect(self.EchoPin, GPIO.RISING)
        #Variables
        MAXTIME = 40
        StartTime = 0
        StopTime = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TrigPin, GPIO.OUT)
        GPIO.setup(self.EchoPin, GPIO.IN)
        
        while Utils.UltraschallThreadStop == 0:
            try:
                StartTime2 = time.time()
                #GPIO setup
                #Trigger
                GPIO.output(self.TrigPin, 0)
                time.sleep(0.03)
                
                GPIO.output(self.TrigPin, 1)
                time.sleep(0.00001)
                GPIO.output(self.TrigPin, 0)

                #Get times
                time.sleep(0.010)
                TIMEOUT = time.time() + MAXTIME
                while not GPIO.event_detected(self.EchoPin) and StopTime < TIMEOUT:
                    StartTime = time.time()

                TIMEOUT = time.time() + MAXTIME
                while GPIO.input(self.EchoPin) == 1 and StopTime < TIMEOUT:
                    StopTime = time.time()
               

                #Calculate distance
                delay = StopTime - StartTime
                distance = (delay * 34030) / 2
                
                if distance < 0:
                    self.distance = 0
                elif distance > 500:
                    self.distance = 500
                else:
                    self.distance = distance
                
                self.values[self.index] = self.distance
                self.index = (self.index + 1) % self.smoothing_window_size
                self.sDistance = sum(self.values) / self.smoothing_window_size
                StopTime2 = time.time()
            
            except Exception as e:
                self.Utils.LogError(f"An Error occured in SuperSonicSensor.measure_distance: {e}")
                self.threadStop = 1
                self.Utils.StopRun()
       
    
    #Stop the thread for measuring the sensor
    def stop_measurement(self):
        try:
            self.Utils.UltraschallThreadStop = 1
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SuperSonicSensor.stop_measurement: {e}")
            self.Utils.StopRun()
        
        

#A class for controlling the Servo that is used for steering        
class Servo(Utility):
    def __init__(self, SignalPin, frequency, Utils):
        try:
            #Variable init
            self.SignalPin, self.frequency = SignalPin, frequency
            self.Utils = Utils
            self.percentage = 0
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.SignalPin, GPIO.OUT)
            
            #Start steering at 0

            self.pwm = GPIO.PWM(SignalPin, frequency)
            self.pwm.start(6.6)
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Servo initialization: {e}")
            self.Utils.StopRun()

        
    #Calculate the DutyCycle from a steering percentage and steer the Servo    
    def steer(self, percentage):
        try:    
            self.percentage = percentage
            
            #Calculate DutyCycle and set it
            try:
                DutyCycle = 3e-5 * self.percentage**2 + 0.018 * self.percentage + 6.57
                self.pwm.ChangeDutyCycle(DutyCycle)
            except:
                self.Utils.LogError(f"No valid steering percentage specified: {percentage}")
                raise CustomException(f"No valid steering percentage specified: {percentage}")
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Servo.steer: {e}")
            self.Utils.StopRun()
            
         

#A class for reading the PixyCam ############################ WIP #########################################            
class PixyCam(Utility):
    def __init__(self, Utils):
        self.count, self.output = 0, ()
        self.Utils = Utils
        pixy.init()
        pixy.change_prog ("color_connected_components");        
        
        
    def start_reading(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
       
        
    def read(self):
        self.output = BlockArray(100)
            
        while self.threadStop == 0:
            time.sleep(0.01)
            self.count = pixy.ccc_get_blocks(100, self.output)
       
            
    def stop_reading(self):
        self.threadStop = 1
      
        
    def LED(self, state):
        if state == 1:
            set_lamp(1, 1)
        elif state == 0:
            set_lamp(0, 0)
        else:
            self.Utils.LogError(f"no valid state specified: {state}")
            raise CustomException(f"no valid state specified: {state}")
            
       

#A class for reading a Button; A Button that instantly stops the program if pressed            
class Button(Utility):
    def __init__(self, SignalPin, Utils):
        try:
            #Variables
            self.SignalPin = SignalPin
            self.Utils = Utils
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Button initialization: {e}")
            self.Utils.StopRun()
        
    
    #Read the state of the Button -- 1 if pressed, 0 if not    
    def state(self):
        try:
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            #Read button state
            if GPIO.input(self.SignalPin) == 0:
                return 1
            elif GPIO.input(self.SignalPin) == 1:
                return 0
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Button.state: {e}")
            self.Utils.StopRun()
      
    
    #Start the Thread for reading the StopButton    
    def start_StopButton(self):
        try:
            self.threadStop = 0
            self.thread = threading.Thread(target=self.read_StopButton, daemon=True)
            self.thread.start()
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Button.start_StopButton: {e}")
            self.Utils.StopRun()
        
    
    #Function that kills the program if the StopButton is pressed    
    def read_StopButton(self):
        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        #Stop program if stopbutton is pressed
        while self.threadStop == 0:
            StartTime = time.time()
            time.sleep(0.1)
            if self.state() == 1:
                self.Utils.LogError("StopButton pressed")
                self.Utils.StopRun()
            StopTime = time.time()
          
    #Stop the Thread for reading the StopButton      
    def stop_StopButton(self):
        try:
            self.threadStop = 1
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Button.stop_StopButton: {e}")
            self.Utils.StopRun()
     
     
        
#A class for reading a TCS34725 ColorSensor         
class ColorSensor(Utility):
    def __init__(self, Utils):
        try:
            #Variable init
            self.color_rgb, self.color_temperature, self.lux = 0, 0, 0
            self.Utils = Utils
            
            #Colorsensor init
            i2c = board.I2C()
            self.sensor = adafruit_tcs34725.TCS34725(i2c)
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in ColorSensor initialization: {e}")
            self.Utils.StopRun()
            
    
    #Start a new thread for reading the sensor        
    def start_measurement(self):
        try:
            self.sensor.active = True
            self.threadStop = 0
            self.thread = threading.Thread(target=self.read, daemon=True)
            self.thread.start()
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in ColorSensor.start_measurement: {e}")
            self.Utils.StopRun()
        
        
    #Read the sensor data    
    def read(self):
        try:
            #Write sensor data to variables
            while self.threadStop == 0:
                self.color_temperature = self.sensor.color_temperature
                #self.color_rgb = self.sensor.color_rgb_bytes
                #self.lux = self.sensor.lux
                
        except Exception as e:
            self.Utils.LogError(f"An Error occured in ColorSensor.read: {e}")
            self.Utils.StopRun()
        
    
    #Stop the thread for reading the sensor   
    def stop_measurement(self):
        try:
            self.sensor.active = False
            self.threadStop = 1
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in ColorSensor.stop_measurement: {e}")
            self.Utils.StopRun()
        
        

#A class for reading a MPU6050 Gyroscope ############################ WIP #########################################  
class Gyroscope(Utility):
    def __init__(self, Utils):
        try:
            i2c = busio.I2C(board.D1, board.D0)
            self.sensor = adafruit_mpu6050.MPU6050(i2c)
            self.sensor._gyro_range = 0
            self.Utils = Utils

            #Initialize variables for storing the angle and time
            self.angle = 0.0  # Initial angle
            self.last_time = time.time()
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Gyroscope initialization: {e}")
            self.Utils.StopRun()


    #Read the gyroscope data and calculate the angle
    def get_angle(self):
        try:
            offset_x = 0.29
            offset_y = 0.0
            offset_z = 0.0
            #Read gyroscope data
            gyro_data = self.sensor.gyro
            gyro_data = [gyro_data[0] + offset_x, gyro_data[1] + offset_y, gyro_data[2] + offset_z]

            # Get the current time
            current_time = time.time()

            # Calculate the time elapsed since the last measurement
            delta_time = current_time - self.last_time
            
            #bugfix for time-jumps
            if delta_time >= 0.5:
                delta_time = 0.003

            # Integrate the gyroscope readings to get the change in angle
            if gyro_data[0] < 0.02 and gyro_data[0] > -0.02:
                gyro_data = 0
            else:
                gyro_data = gyro_data[0]
                
            delta_angle = gyro_data * delta_time * 60

            # Update the angle
            self.angle += delta_angle

            # Update the last time for the next iteration
            self.last_time = current_time
    
        except Exception as e:
                self.Utils.LogError(f"An Error occured in Gyroscope.get_angle: {e}")
                self.Utils.StopRun()
    
        

#A class for reading a ADS1015 ADC        
class AnalogDigitalConverter(Utility):
    def __init__(self, Utils, channel=2):
        try:
            #Variables
            self.channel = channel
            self.voltage = 12
            
            #ADC init
            i2c = busio.I2C(board.D1, board.D0)
            self.ads = ADS.ADS1015(i2c)
            self.ads.active = True
            self.Utils = Utils
            
            #Channel init
            if channel == 0:
                self.chan = AnalogIn(self.ads, ADS.P0)
            elif channel == 1:
                self.chan = AnalogIn(self.ads, ADS.P1)
            elif channel == 2:
                self.chan = AnalogIn(self.ads, ADS.P2)
            elif channel == 3:
                self.chan = AnalogIn(self.ads, ADS.P3)
            else:
                raise CustomException(f"No valid ADC channel specified: {channel}")
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in AnalogDigitalConverter initialization: {e}")
            self.Utils.StopRun()
       
    
    #Read the voltage from the ADC    
    def read(self):
        self.voltage = self.chan.voltage * 4.395
        return self.chan.voltage * 4.395



#A class for reading a LM393 speed sensor
class SpeedSensor(Utility):
    def __init__(self, SignalPin, NumberSlots, Utils, P=1):
        try:
            #Variables
            self.SignalPin = SignalPin
            self.speed = 0
            self.NumSlots = NumberSlots
            self.Utils = Utils
            self.P = P
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SignalPin, GPIO.IN)
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SpeedSensor initialization: {e}")
            self.Utils.StopRun()
            
    
    #Start a new thread for measuring the sensor
    def start_measurement(self):
        try:
            GPIO.setmode(GPIO.BCM)
            self.threadStop = 0
            self.thread = threading.Thread(target=self.hold_speed, daemon=True, args=(self.Utils, self.NumSlots,))
            self.thread.start()
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SpeedSensor.start_measurement: {e}")
            self.Utils.StopRun()
            
    
    #Measure the speed with the sensor
    def hold_speed(self, Utils, NumSlots):
        try:
            GPIO.setmode(GPIO.BCM)
            self.Utils = Utils
            PulseTime = time.time()
            lastPulseTime = 0
            GPIO.setup(self.SignalPin, GPIO.IN)
            
            while self.threadStop == 0:
                try:
                    if self.Utils.Motor1.MotorSpeed != 0:
                        GPIO.wait_for_edge(self.SignalPin, GPIO.FALLING)
                            
                        #GPIO setup
                        #GPIO.setmode(GPIO.BCM)
                        
                        #Measure speed
                        GPIO.wait_for_edge(self.SignalPin, GPIO.RISING)
                        PulseTime = time.time()
                            
                        self.speed = (60 * 1000) / (NumSlots * (PulseTime - lastPulseTime)) / 100000 * 1.5
                        lastPulseTime = PulseTime
                        
                        Error = self.speed - self.Utils.Motor1.MotorSpeed
                        Correction = self.P * Error * -1
                        
                        if Correction > 100:
                            Correction = 100
                        elif Correction < 0:
                            Correction = 0
                            
                        self.Utils.Motor1.drive('f', Correction)
                        
                except Exception as e:
                    self.Utils.LogError(f"An Error occured in SpeedSensor.hold_speed: {e}")
                    self.Utils.StopRun()
                
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SpeedSensor.hold_speed: {e}")
            self.Utils.StopRun()
        
      
    #Stop the thread for measuring the sensor  
    def stop_measurement(self):
        try:
            self.threadStop = 1
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in SpeedSensor.stop_measurement: {e}")
            self.Utils.StopRun()
        
        
        
#A class for writing to a OLED Display
class DisplayOled(Utility):
    def __init__(self, ADC, Gyro, Utils):
        try:
            serial = i2c(port=0, address=0x3C)
            self.device = sh1106(serial)
            
            #Variable init
            self.first_line = ""
            self.second_line = ""
            self.ADC = ADC
            self.Utils = Utils
            self.Gyro = Gyro
            
            #Wake the screen by drawing an outline
            with canvas(self.device) as draw:
                draw.rectangle(self.device.bounding_box, outline="white", fill="black")
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in DisplayOled initialization: {e}")
            self.Utils.StopRun()
        
       
    #Clear the Display 
    def clear(self):
        try:
            with canvas(self.device) as draw:
                draw.rectangle(self.device.bounding_box, outline="white", fill="black")
                
        except Exception as e:
            self.Utils.LogError(f"An Error occured in DisplayOled.clear: {e}")
            self.Utils.StopRun()
       
    
    #Write lines in variables so they get written by the update function   
    def write(self, first_line="", second_line="",reset=False, xCoord=0, yCoord=17):
        try:
            if reset:
                self.first_line = ""
                self.second_line = ""
            
            if first_line != "":
                self.first_line = first_line
            if second_line != "":
                self.second_line = second_line
        
        except Exception as e:
            self.Utils.LogError(f"An Error occured in DisplayOled.write: {e}")
            self.Utils.StopRun()
        

    #Start a new thread for updating the Display
    def start_update(self):
        try:
            self.threadStop = 0
            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in DisplayOled.start_update: {e}")
            self.Utils.StopRun()

    
    #Update the Display
    def update(self):
        try:
            counter = 0
            while self.threadStop == 0:
                if counter == 200:
                    counter = 0
                    
                    #Get CPU temperature, CPU usage, RAM usage and Disk usage
                    cpuTemp = CPUTemperature()
                    self.cpu_usage = psutil.cpu_percent(interval=0)
                    self.ram = psutil.virtual_memory()
                    self.disk = psutil.disk_usage('/')
                    
                    #Format them to always have the same number of decimal points
                    cpu_temp_formatted = self.convert_to_decimal_points(cpuTemp.temperature, 1)
                    cpu_usage_formatted = self.convert_to_decimal_points(self.cpu_usage, 1)
                    ram_usage_formatted = self.convert_to_decimal_points(self.ram.percent, 1)
                    disk_usage_formatted = self.convert_to_decimal_points(self.disk.percent, 1)
                    voltage_value_formatted = self.convert_to_decimal_points(self.ADC.read(), 2)
                    
                    #Draw all the data on the Display
                    with canvas(self.device) as draw:
                        #top
                        draw.text((0, 0), f"{cpu_temp_formatted}°C", fill="white", align="left")
                        draw.text((40, 0), f"DISK:{(int(float(disk_usage_formatted)))}%", fill="white")
                        draw.text((92, 0), f"{voltage_value_formatted}V", fill="white")
                        
                        #bottom
                        draw.text((0, 50), f"CPU:{cpu_usage_formatted}%", fill="white")
                        draw.text((75, 50), f"RAM:{ram_usage_formatted}%", fill="white")
                        
                        #custom
                        draw.multiline_text((0, 15), f"{self.first_line}\n{self.second_line}", fill="white", align="center", anchor="ma")
                        
                self.Gyro.get_angle()
                counter += 1
                time.sleep(0.003)
                    
        except Exception as e:
            self.Utils.LogError(f"An Error occured in DisplayOled.update: {e}")
            self.Utils.StopRun()
         
    
    #Stop the thread for updating the Display      
    def stop_update(self):
        try:
            self.threadStop = 1

        except Exception as e:
            self.Utils.LogError(f"An Error occured in DisplayOled.stop_update: {e}")
            self.Utils.StopRun()



#A class for making sounds with a Buzzer
class Buzzer(Utility):
    def __init__(self, SignalPin, Utils):
        try:
            self.SignalPin = SignalPin
            self.Utils = Utils
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Buzzer initialization: {e}")
            self.Utils.StopRun()
            
    def buzz(self, frequency, volume, duration):
        try:
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.SignalPin, GPIO.OUT)
            pwm = GPIO.PWM(self.SignalPin, frequency)
            pwm.start(volume)
            time.sleep(duration)
            pwm.stop()
            time.sleep(0.01)
            
        except Exception as e:
            self.Utils.LogError(f"An Error occured in Buzzer.buzz: {e}")
            self.Utils.StopRun()