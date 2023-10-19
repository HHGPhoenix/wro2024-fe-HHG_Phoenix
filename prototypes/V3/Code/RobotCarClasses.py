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



##########################################################
##                                                      ##
##                     Classes                          ##
##                                                      ##
##########################################################

#A class that can be called to raise a custom exception and print a custom message
class CustomException(Exception):
    def __init__(self, message):
        print(message)  # Print the custom message



#A class that has some necessary tools for calculating, usw.
class Utility:
    #Transfer data so it can be used in other classes
    def transferSensorData(self, Ultraschall1=None, Ultraschall2=None, Farbsensor=None, Motor1=None, Servo1=None, StartButton=None, StopButton=None, Funcs=None, Display=None, Pixy=None):
        #self.Ultraschall1, self.Ultraschall2, self.Farbsensor, self.Motor1, self.Servo1, self.StartButton, self.StopButton, self.Pixy, self.Funcs = Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy, Funcs
        self.Ultraschall1 = Ultraschall1
        self.Ultraschall2 = Ultraschall2
        self.Farbsensor = Farbsensor
        self.Motor1 = Motor1
        self.Servo1 = Servo1
        self.StartButton = StartButton
        self.StopButton = StopButton
        self.Funcs = Funcs
        self.Pixy = Pixy
        self.Display = Display
        
        self.ActivSensor = 0
        self.file_path = "/tmp/StandbyScript.lock"
        self.StartTime = 0
        
        
    #Cleanup after the run is finished or an error occured
    def cleanup(self):
        print("Started cleanup")
        
        #Stop all Threads if the class has been initialized
        if self.Ultraschall1 != None:
            self.Ultraschall1.stop_measurement()
        if self.Ultraschall2 != None:
            self.Ultraschall2.stop_measurement()
        if self.Farbsensor != None:
            self.Farbsensor.stop_measurement()
        if self.Pixy != None:
            self.Pixy.stop_reading()
        if self.Display != None:
            self.Display.stop_update()
        if self.StopButton != None:
            self.StopButton.stop_StopButton()

        #Stop the Motor if it has been initialized
        if self.Motor1 != None:
            self.Motor1.drive("f", 0)
            self.Motor1.stop()
        
        #Wait a short time to make sure all threads are stopped
        time.sleep(0.5)
        
        #GPIO cleanup and Program exit
        GPIO.cleanup()
        self.running = False
        print("Finished cleanup")
        
        #Start StandbyScript
        os.remove(self.file_path)
    
    
    #Do some init and wait until StartButton is pressed
    def StartRun(self, MotorSpeed=0, steer=0, direction="f"):
        #Stop StandbyScript.py
        with open(self.file_path, 'w'):
            pass  # Using 'pass' as a placeholder for no content
        time.sleep(1)

        #Start Processes
        p1 = mp.Process(target=self.Display.start_update())
        p1.start()
        p2 = mp.Process(target=self.StopButton.start_StopButton())
        p2.start()
        p3 = mp.Process(target=self.Farbsensor.start_measurement())
        p3.start()
        p4 = mp.Process(target=self.Ultraschall2.start_measurement())
        p4.start()
        
        #Wait for StartButton to be pressed
        self.running = True
        self.waiting = True
        while self.running and self.waiting:
            try:
                time.sleep(0.1)
                if self.StartButton.state() == 1:
                    self.StartTime = time.time()
                    print(f"Run started: {time.time()}")
                    
                    self.Motor1.drive(direction, MotorSpeed)
                    self.Servo1.steer(steer)
                    
                    self.waiting = False   
            except:
                self.StopRun()
          
    
    #Stop the run and calculate the time needed            
    def StopRun(self):
        self.StopTime = time.time()
        print(f"Run ended: {self.StopTime}")
        if self.Starttime != None:
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
            
        self.cleanup()
          
    
    #Setup datalogging
    def setupDataLog(self):
        try:
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
            print(f"An Error occured in Utility.setupDataLog: {e}")
            self.StopRun()
            

    #Log Sensor values
    def LogData(self):
        while self.DataLoggerStop == 0:
            try:
                data = f"Ultraschall1: "
                self.datalogger.debug(data)
            except Exception as e:
                print(f"An Error occured in Utility.LogData: {e}")
                self.StopRun()
        
    
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
            print(f"An Error occured in Utility.setupLog: {e}")
            self.StopRun()
        
        
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
            print(f"Could not convert number to {decimal_points} decimal points: {e}")
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
            if ID == 0:
                for thread in threading.enumerate():
                    if thread.name == "Ultraschall1":
                        self.Ultraschall1.threadStop = 1
                        thread.join()
                    if thread.name == "Ultraschall2":
                        self.Ultraschall2.threadStop = 1 
                        thread.join()    
                return
                
            if ID == 1:
                for thread in threading.enumerate():
                    if thread.name == "Ultraschall2Thread":
                        self.Ultraschall2.threadStop = 1
                        thread.join()
                    if thread.name == "Ultraschall1Thread":
                        return 
                        
                self.Ultraschall1.threadStop = 0
                Ultraschall1Thread = threading.Thread(target=self.Ultraschall1.measure_distance, name="Ultraschall1Thread", daemon=True)
                Ultraschall1Thread.start()
                return
            
            if ID == 2:
                for thread in threading.enumerate():
                    if thread.name == "Ultraschall1Thread":
                        self.Ultraschall1.threadStop = 1
                        thread.join()
                    if thread.name == "Ultraschall2Thread":
                        return 
                   
                self.Ultraschall2.threadStop = 0     
                Ultraschall2Thread = threading.Thread(target=self.Ultraschall2.measure_distance, name="Ultraschall2Thread", daemon=True)
                Ultraschall2Thread.start()
                return
            
        except Exception as e:
            print(f"An Error occured in Utility.toggle_supersonic_sensor: {e}")
            self.StopRun()
  
    
    
#Class for the drive Motor
class Motor(Utility):
    def __init__(self, frequency, fpin, rpin, spin):
        try:
            #Setup Variables
            self.frequency, self.fpin, self.rpin, self.spin, self.speed = frequency, fpin, rpin, spin, 0
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(fpin, GPIO.OUT)
            GPIO.setup(rpin, GPIO.OUT)
            GPIO.setup(spin, GPIO.OUT)
            
            self.pwm = GPIO.PWM(spin, frequency)
            
        except Exception as e:
            print(f"An Error occured in Motor initialization: {e}")
            self.StopRun()


    #Set the direction and speed of the Motor
    def drive(self, direction, speed=0):
        try:
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.fpin, GPIO.OUT)
            GPIO.setup(self.rpin, GPIO.OUT)
            GPIO.setup(self.spin, GPIO.OUT)

            self.speed = speed
            
            #Set direction
            if direction == 'f':
                GPIO.output(self.fpin, 1)
                GPIO.output(self.rpin, 0)
            elif direction == 'r':
                GPIO.output(self.fpin, 0)
                GPIO.output(self.rpin, 1)
            else:
                print(f"No valid direction specified: {direction}")
                raise CustomException(f"No valid direction specified: {direction}")

            #Set speed
            self.pwm.ChangeDutyCycle(speed)
            
        except Exception as e:
            print(f"An Error occured in Motor.drive: {e}")
            self.StopRun()

       
    #Start the Motor with the last known speed        
    def start(self):
        try:
            self.pwm.start(self.speed)  
        except Exception as e:
            print(f"An Error occured in Motor.start: {e}")
            self.StopRun()
       
       
    #Stop the Motor        
    def stop(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.output(self.fpin, 0) 
            GPIO.output(self.rpin, 0)
            self.pwm.stop()
        except Exception as e:
            print(f"An Error occured in Motor.stop: {e}")
            self.StopRun()
        
        

#Class for reading a SuperSonicSensor        
class SuperSonicSensor(Utility):
    def __init__(self, TrigPin, EchoPin, ID, smoothing_window_size=3):
        try:
            #Variable init
            self.EchoPin, self.TrigPin, self.distance, self.smoothing_window_size, self.sDistance = EchoPin, TrigPin, 0, smoothing_window_size, 0
            self.ID = ID
            self.ThreadStop = 0
            #Moving Average init
            self.values = [0] * self.smoothing_window_size
            self.index = 0
            
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(TrigPin, GPIO.OUT)
            GPIO.setup(EchoPin, GPIO.IN)
            
            
        except Exception as e:
            print(f"An Error occured in SuperSonicSensor initialization: {e}")
            self.StopRun()
        
    
    #Start a new thread for measuring the sensor   
    def start_measurement(self):
        try:
            self.threadStop = 0
            self.thread = threading.Thread(target=self.measure_distance, daemon=True)
            self.thread.start()
            
        except Exception as e:
            print(f"An Error occured in SuperSonicSensor.start_measurement: {e}")
            self.StopRun()
    
    
    #measure the distance with the sensor
    def measure_distance(self):
        #Variables
        MAXTIME = 0.04
        StartTime = 0
        StopTime = 0
        
        while self.threadStop == 0:
            try:
                StartTime2 = time.time()
                #GPIO setup
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.TrigPin, GPIO.OUT)
                GPIO.setup(self.EchoPin, GPIO.IN)
                #Trigger
                GPIO.output(self.TrigPin, 0)
                time.sleep(0.003)
                
                GPIO.output(self.TrigPin, 1)
                time.sleep(0.00001)
                GPIO.output(self.TrigPin, 0)

                #Get times
                TIMEOUT = time.time() + MAXTIME      
                while GPIO.input(self.EchoPin) == 0 and StopTime < TIMEOUT:
                    StartTime = time.time()
                    
                TIMEOUT = time.time() + MAXTIME
                while GPIO.input(self.EchoPin) == 1 and StopTime < TIMEOUT:
                    StopTime = time.time()

                #Calculate distance
                delay = StopTime - StartTime
                self.distance = (delay * 34030) / 2
                
                self.values[self.index] = self.distance
                self.index = (self.index + 1) % self.smoothing_window_size
                self.sDistance = sum(self.values) / self.smoothing_window_size
                StopTime2 = time.time()
            
            except Exception as e:
                print(f"An Error occured in SuperSonicSensor: {e}")
                self.threadStop = 1
                self.StopRun()
       
    
    #Stop the thread for measuring the sensor
    def stop_measurement(self):
        try:
            self.threadStop = 1
        
        except Exception as e:
            print(f"An Error occured in SuperSonicSensor.stop_measurement: {e}")
            self.StopRun()
        
        

#A class for controlling the Servo that is used for steering        
class Servo(Utility):
    def __init__(self, SignalPin, frequency):
        try:
            #Variable init
            self.SignalPin, self.frequency = SignalPin, frequency
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.SignalPin, GPIO.OUT)
            
            #Start steering at 0

            self.pwm = GPIO.PWM(SignalPin, frequency)
            self.pwm.start(6.6)
            
        except Exception as e:
            print(f"An Error occured in Servo initialization: {e}")
            self.StopRun()

        
    #Calculate the DutyCycle from a steering percentage and steer the Servo    
    def steer(self, percentage):
        try:
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.SignalPin, GPIO.OUT)
            
            #Calculate DutyCycle and set it
            try:
                DutyCycle = 3e-5 * percentage**2 + 0.018 * percentage + 6.57
                self.pwm.ChangeDutyCycle(DutyCycle)
            except:
                print(f"No valid steering percentage specified: {percentage}")
                raise CustomException(f"No valid steering percentage specified: {percentage}")
            
        except Exception as e:
            print(f"An Error occured in Servo.steer: {e}")
            self.StopRun()
            
         

#A class for reading the PixyCam ############################ WIP #########################################            
class PixyCam(Utility):
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
            
       

#A class for reading a Button; A Button that instantly stops the program if pressed            
class Button(Utility):
    def __init__(self, SignalPin):
        try:
            #Variables
            self.SignalPin = SignalPin
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SignalPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
        except Exception as e:
            print(f"An Error occured in Button initialization: {e}")
            self.StopRun()
        
    
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
            print(f"An Error occured in Button.state: {e}")
            self.StopRun()
      
    
    #Start the Thread for reading the StopButton    
    def start_StopButton(self):
        try:
            self.threadStop = 0
            self.thread = threading.Thread(target=self.read_StopButton, daemon=True)
            self.thread.start()
        except Exception as e:
            print(f"An Error occured in Button.start_StopButton: {e}")
            self.StopRun()
        
    
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
                print("StopButton pressed")
                self.StopRun()
            StopTime = time.time()
          
    #Stop the Thread for reading the StopButton      
    def stop_StopButton(self):
        try:
            self.threadStop = 1
            
        except Exception as e:
            print(f"An Error occured in Button.stop_StopButton: {e}")
            self.StopRun()
     
     
        
#A class for reading a TCS34725 ColorSensor         
class ColorSensor(Utility):
    def __init__(self):
        try:
            #Variable init
            self.color_rgb, self.color_temperature, self.lux = 0, 0, 0
            
            #Colorsensor init
            i2c = board.I2C()
            self.sensor = adafruit_tcs34725.TCS34725(i2c)
            
        except Exception as e:
            print(f"An Error occured in ColorSensor initialization: {e}")
            self.StopRun()
            
    
    #Start a new thread for reading the sensor        
    def start_measurement(self):
        try:
            self.sensor.active = True
            self.threadStop = 0
            self.thread = threading.Thread(target=self.read, daemon=True)
            self.thread.start()
            
        except Exception as e:
            print(f"An Error occured in ColorSensor.start_measurement: {e}")
            self.StopRun()
        
        
    #Read the sensor data    
    def read(self):
        try:
            #Write sensor data to variables
            while self.threadStop == 0:
                StartTime = time.time()
                time.sleep(0.003)
                self.color_temperature = self.sensor.color_temperature
                #self.color_rgb = self.sensor.color_rgb_bytes
                #self.lux = self.sensor.lux
                StopTime = time.time()
                
        except Exception as e:
            print(f"An Error occured in ColorSensor.read: {e}")
            self.StopRun()
        
    
    #Stop the thread for reading the sensor   
    def stop_measurement(self):
        try:
            self.sensor.active = False
            self.threadStop = 1
        
        except Exception as e:
            print(f"An Error occured in ColorSensor.stop_measurement: {e}")
            self.StopRun()
        
        

#A class for reading a MPU6050 Gyroscope ############################ WIP #########################################  
class Gyroscope(Utility):
    def __init__(self):
        try:
            i2c = board.I2C()
            self.sensor = adafruit_mpu6050.MPU6050(i2c)
            self.sensor._gyro_range = 0

            #Initialize variables for storing the angle and time
            self.angle = 0.0  # Initial angle
            self.last_time = time.time()
            
        except Exception as e:
            print(f"An Error occured in Gyroscope initialization: {e}")
            self.StopRun()


    #Read the gyroscope data and calculate the angle
    def get_angle(self):
        try:
            #Read gyroscope data
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
        
        except Exception as e:
            print(f"An Error occured in Gyroscope.get_angle: {e}")
            self.StopRun()
    
        

#A class for reading a ADS1015 ADC ############################ WIP #########################################         
class AnalogDigitalConverter(Utility):
    def __init__(self, channel=0):
        try:
            #Variables
            self.channel = channel
            
            #ADC init
            i2c = busio.I2C(board.D1, board.D0)
            self.ads = ADS.ADS1015(i2c)
            self.ads.active = True
            
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
            print(f"An Error occured in AnalogDigitalConverter initialization: {e}")
            self.StopRun()
       
    
    #Read the voltage from the ADC    
    def read(self):
        return self.chan.voltage * 4.35



#A class for reading a LM393 speed sensor ############################ testing needed #########################################  
class SpeedSensor(Utility):
    def __init__(self, SignalPin, NumberSlots):
        try:
            #Variables
            self.SignalPin = SignalPin
            self.speed = 0
            self.NumSlots = NumberSlots
            
            #GPIO setup
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SignalPin, GPIO.IN)
            
        except Exception as e:
            print(f"An Error occured in SpeedSensor initialization: {e}")
            self.StopRun()
            
    
    #Start a new thread for measuring the sensor
    def start_measurement(self):
        try:
            self.threadStop = 0
            self.thread = threading.Thread(target=self.measure_speed, daemon=True)
            self.thread.start()
            
        except Exception as e:
            print(f"An Error occured in SpeedSensor.start_measurement: {e}")
            self.StopRun()
            
    
    #Measure the speed with the sensor
    def measure_speed(self):
        try:
            lastPulseTime = time.time()
            while self.threadStop == 0:
                #GPIO setup
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.SignalPin, GPIO.IN)
                
                #Measure speed
                while GPIO.input(self.SignalPin) == 0:
                    PulseTime = time.time()
                    
                self.speed = (60 * 100 * 10) / (self.NumSlots * (PulseTime - lastPulseTime)) / 100000 * 1.5
                lastPulseTime = PulseTime
                
        except Exception as e:
            print(f"An Error occured in SpeedSensor.measure_speed: {e}")
            self.StopRun()
        
      
    #Stop the thread for measuring the sensor  
    def stop_measurement(self):
        try:
            self.threadStop = 1
        
        except Exception as e:
            print(f"An Error occured in SpeedSensor.stop_measurement: {e}")
            self.StopRun()
        
        
        
#A class for writing to a OLED Display
class DisplayOled(Utility):
    def __init__(self, ADC=None):
        try:
            serial = i2c(port=0, address=0x3C)
            self.device = sh1106(serial)
            
            #Variable init
            self.first_line = ""
            self.second_line = ""
            self.ADC = ADC
            
            #Wake the screen by drawing an outline
            with canvas(self.device) as draw:
                draw.rectangle(self.device.bounding_box, outline="white", fill="black")
        
        except Exception as e:
            print(f"An Error occured in DisplayOled initialization: {e}")
            self.StopRun()
        
       
    #Clear the Display 
    def clear(self):
        try:
            with canvas(self.device) as draw:
                draw.rectangle(self.device.bounding_box, outline="white", fill="black")
                
        except Exception as e:
            print(f"An Error occured in DisplayOled.clear: {e}")
            self.StopRun()
       
    
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
            print(f"An Error occured in DisplayOled.write: {e}")
            self.StopRun()
        

    #Start a new thread for updating the Display
    def start_update(self):
        try:
            self.threadStop = 0
            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()
            
        except Exception as e:
            print(f"An Error occured in DisplayOled.start_update: {e}")
            self.StopRun()

    
    #Update the Display
    def update(self):
        try:
            while self.threadStop == 0:
                StartTime = time.time()
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
                
                time.sleep(0.5)
                StopTime = time.time()
                
        except Exception as e:
            print(f"An Error occured in DisplayOled.update: {e}")
            self.StopRun()
         
    
    #Stop the thread for updating the Display      
    def stop_update(self):
        try:
            self.threadStop = 1

        except Exception as e:
            print(f"An Error occured in DisplayOled.stop_update: {e}")
            self.StopRun()



#A class for driving functions
class Functions(Utility):
    def __init__(self, Utils, Ultraschall1=None, Ultraschall2=None, Farbsensor=None, Motor1=None, Servo1=None, StartButton=None, StopButton=None, Display=None, Pixy=None):
        self.Ultraschall1 = Ultraschall1
        self.Ultraschall2 = Ultraschall2
        self.Farbsensor = Farbsensor
        self.Motor1 = Motor1
        self.Servo1 = Servo1
        self.StartButton = StartButton
        self.StopButton = StopButton
        self.Pixy = Pixy
        self.Display = Display
        
        self.rounds = 0
        self.Utils = Utils
     
            
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