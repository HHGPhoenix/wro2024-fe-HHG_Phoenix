import gpiod
import time
import threading
import os, signal, socket
from gpiozero import CPUTemperature
import psutil
import multiprocessing as mp
import logging
from flask import Flask, render_template, Response, jsonify
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
from USB_communication_handler import USBCommunication
from I2C_handler import I2Ccommunication



##########################################################
##                                                      ##
##                   GPIO init                          ##
##                                                      ##
##########################################################

global chip, all_lines

chip = gpiod.Chip('gpiochip4')

all_lines = []


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
    def transferSensorData(self, Farbsensor=None, StartButton=None, StopButton=None, Buzzer1=None):
        self.setupLog()
        
        self.usb_communication = USBCommunication(self)
        self.ESPHoldDistance, self.ESPHoldSpeed = self.usb_communication.initNodeMCUs()
        
        self.I2C_communication = I2Ccommunication(self)
        self.Display = self.I2C_communication.Display
        self.ADC = self.I2C_communication.ADC
        self.Gyro = self.I2C_communication.Gyro
        
        self.Farbsensor = Farbsensor
        self.StartButton = StartButton
        self.StopButton = StopButton
        self.Buzzer1 = Buzzer1
        self.StartTime = time.time()
        
        self.ActivSensor = 0
        self.file_path = "/tmp/StandbyScript.lock"
        
        self.stop_run_callable = True
        
        return self.ESPHoldDistance, self.ESPHoldSpeed
        
        
    #Cleanup after the run is finished or an error occured
    def cleanup(self):
        self.LogDebug("Started cleanup")

        self.I2C_communication.stop_threads()
        self.StopButton.stop_StopButton()
        
        #Stop Nodemcu's
        self.usb_communication.sendMessage("STOP", self.ESPHoldDistance)
        time.sleep(0.1)
        self.usb_communication.sendMessage("STOP", self.ESPHoldSpeed)
        
        #self.StopNodemcus()
        
        #Wait a short time to make sure all threads are stopped
        self.Buzzer1.buzz(1000, 80, 0.5)
        
        #Clear all used lines
        for line in all_lines:
            line.release()
        
        self.running = False
        self.usb_communication.closeNodeMCUs(self.ESPHoldDistance, self.ESPHoldSpeed)
        os.kill(os.getpid(), signal.SIGTERM)
    
    
    #Do some init and wait until StartButton is pressed
    def StartRun(self):
        #clear console
        os.system('cls' if os.name=='nt' else 'clear')
        
        pI2C = mp.Process(target=self.I2C_communication.start_threads())
        pI2C.start()

        if self.StopButton != None:
            p2 = mp.Process(target=self.StopButton.start_StopButton())
            p2.start()

        #Wait for StartButton to be pressed
        self.running = True
        self.waiting = True
        
        self.LogDebug("Waiting for Button to be pressed...")
        self.Display.write("Waiting for Button", "to be pressed...")
        
        self.Buzzer1.buzz(1000, 80, 0.1)
        time.sleep(0.1)
        self.Buzzer1.buzz(1000, 80, 0.1)
        
        while self.running and self.waiting:
            time.sleep(0.1)
            if self.StartButton.state() == 1:
        
                self.usb_communication.startNodeMCUs()
                self.Gyro.GyroStart = True

                self.usb_communication.sendMessage(f"D {50}", self.ESPHoldDistance)
                self.usb_communication.sendMessage(f"KP {2}", self.ESPHoldDistance)
                self.usb_communication.sendMessage(f"ED {125}", self.ESPHoldDistance)
                self.usb_communication.sendMessage(f"SPEED {50}", self.ESPHoldSpeed)
                self.usb_communication.sendMessage(f"MM {10}", self.ESPHoldSpeed)
                
                self.Starttime = time.time()
                self.LogDebug(f"Run started: {time.time()}")
                self.Display.write("Run started:", f"{time.time()}")  
                self.Buzzer1.buzz(1000, 80, 0.1) 
                
                self.waiting = False
                
    
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
                
            self.stop_run_callable = False
            self.cleanup()
          
    
    #Setup datalogging
    def setupDataLog(self):
        #Clear DataLog
        #os.remove("DataLog.log")
            
        #Create datalogger
        self.datalogger = logging.getLogger("DataLogger")
        self.datalogger.setLevel(logging.DEBUG)

        
        #Create file handler and set level to debug
        fh = logging.FileHandler("/tmp/DataLog.log", 'w')
        fh.setLevel(logging.DEBUG)
        self.datalogger.addHandler(fh)

        #Start Process to log data while the program is running
        self.DataLoggerStop = 0
        PDataLogger = mp.Process(target=self.LogData)
        PDataLogger.start()
            

    #Log Sensor values
    def LogData(self):
        self.datalogger.debug(f"Farbsensor; {0};  CPU; {psutil.cpu_percent()}; RAM; {psutil.virtual_memory().percent}; CPUTemp; {CPUTemperature().temperature}; Voltage; {self.ADC.voltage}")
    
    
    #Stop the DataLogger Process
    def StopDataLog(self):
        self.DataLoggerStop = 1
        
        
    #Setup logging
    def setupLog(self, name='Log', filename='Debug.log'):
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
        number = float(number)
        formatted_string = f"{number:.{decimal_points}f}"
        return formatted_string


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
    
    
    #Collect data from the sensors
    def data_feed(self):
        return {"D1": self.SensorDistance1, "D2": self.SensorDistance2, "angle": self.Gyro.angle, "voltage": self.ADC.voltage, "cpu_usage": psutil.cpu_percent(), "ram_usage": psutil.virtual_memory().percent}
        
        
#A class for reading a Button; A Button that instantly stops the program if pressed            
class Button(Utility):
    def __init__(self, SignalPin, Utils):
        #Variables
        self.SignalPin = SignalPin
        self.Utils = Utils
        
        #GPIO setup
        self.button_line = chip.get_line(SignalPin)
        self.button_line.request(consumer='Button', type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        
        all_lines.append(self.button_line)
        
    
    #Read the state of the Button -- 1 if pressed, 0 if not    
    def state(self): 
        #Read button state
        if self.button_line.get_value() == 0:
            return 1
        elif self.button_line.get_value() == 1:
            return 0
      
    
    #Start the Thread for reading the StopButton    
    def start_StopButton(self):
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read_StopButton, daemon=True)
        self.thread.start()
  
    
    #Function that kills the program if the StopButton is pressed    
    def read_StopButton(self):
        #Stop program if stopbutton is pressed
        while self.threadStop == 0:
            time.sleep(0.1)
            if self.state() == 1:
                self.Utils.LogError("StopButton pressed")
                self.Utils.running = False
                
                
    #Stop the Thread for reading the StopButton      
    def stop_StopButton(self):
        self.threadStop = 1

          
#A class for making sounds with a Buzzer
class Buzzer(Utility):
    def __init__(self, SignalPin, Utils):

        self.Utils = Utils
        self.SignalPin = chip.get_line(SignalPin)
        self.SignalPin.request(consumer='buzzer', type=gpiod.LINE_REQ_DIR_OUT)
        
        all_lines.append(self.SignalPin)

    def buzz(self, frequency, volume, duration):
        try:
            # Check if the volume value is greater than the frequency
            if volume > frequency:
                volume = frequency

            period = 1.0 / frequency  # Calculate the period of the frequency
            on_time = period * volume / 100  # Calculate the time the signal should be on
            off_time = period - on_time  # Calculate the time the signal should be off

            end_time = time.time() + duration
            while time.time() < end_time:
                self.SignalPin.set_value(1)
                time.sleep(on_time)
                self.SignalPin.set_value(0)
                time.sleep(off_time)
                
        except PermissionError:
            self.Utils.LogDebug("Buzzer already in use, skipping")
            return
            

#A class for detecting red and green blocks in the camera stream           
class Camera():
    def __init__(self, video_stream=False, video_source=0):
        # Variable initialization
        self.freeze = False
        self.frame = None
        self.frame_lock = threading.Lock()
        self.video_stream = video_stream
        self.picam2 = Picamera2()
        
        # Configure and start the camera
        config = self.picam2.create_still_configuration(main={"size": (1280, 720)}, raw={"size": (1280, 720)}, controls={"FrameRate": 34})
        self.picam2.configure(config)
        self.picam2.start()
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        
        # Define the color ranges for green and red in HSV color space
        self.lower_green = np.array([35, 60, 35])
        self.upper_green = np.array([85, 255, 255])

        self.lower_red1 = np.array([0, 200, 100])
        self.upper_red1 = np.array([10, 255, 180])

        self.lower_red2 = np.array([160, 200, 100])
        self.upper_red2 = np.array([180, 255, 180])

        # Define the kernel for morphological operations
        self.kernel = np.ones((5, 5), np.uint8)
        self.desired_distance_wall = -1

        
    #Get the coordinates of the blocks in the camera stream
    def get_coordinates(self):
        frame = self.picam2.capture_array()
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask of pixels within the green color range
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)

        # Create a mask of pixels within the red color range
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Dilate the masks to merge nearby areas
        mask_green = cv2.dilate(mask_green, self.kernel, iterations=1)
        mask_red = cv2.dilate(mask_red, self.kernel, iterations=1)

        # Find contours in the green mask
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find contours in the red mask
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cv2.circle(frame, (640, 720), 10, (255, 0, 0), -1)
        cv2.putText(frame, f"{self.desired_distance_wall}", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)
        cv2.putText(frame, f"Freeze: {self.freeze}", (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)
        
        block_array = []

        # Process each green contour
        for contour in contours_green:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 50:  # Only consider boxes larger than 50x50
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, 'Green Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                block_array.append({'color': 'green', 'x': x, 'y': y, 'w': w, 'h': h, 'mx': x+w/2, 'my': y+h/2, 'size': w*h})
                cv2.line(frame, (640, 720), (int(x+w/2), int(y+h/2)), (0, 255, 0), 2)

        # Process each red contour
        for contour in contours_red:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 50:  # Only consider boxes larger than 50x50
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(frame, 'Red Object', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                block_array.append({'color': 'red', 'x': x, 'y': y, 'w': w, 'h': h, 'mx': x+w/2, 'my': y+h/2, 'size': w*h})
                cv2.line(frame, (640, 720), (int(x+w/2), int(y+h/2)), (0, 0, 255), 2)
                
        return block_array, frame
        
        
    #Functrion running in a new thread that constantly updates the coordinates of the blocks in the camera stream
    def process_blocks(self):
        while True:
            self.block_array, self.frame = self.get_coordinates()
     
          
    #Start a new thread for processing the camera stream          
    def start_processing(self):
        thread = threading.Thread(target=self.process_blocks)
        thread.daemon = False
        thread.start()
      
        
    #Generate the frames for the webstream
    def video_frames(self):
        if self.video_stream:
            while True:
                with self.frame_lock:
                    if self.frame is not None:
                        (flag, encodedImage) = cv2.imencode(".jpg", self.frame)
                        yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
                    else:
                        yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' + b'\r\n')