from RobotCarClasses import AnalogDigitalConverter, DisplayOled, Utility, Functions
import time
import multiprocessing as mp
import RPi.GPIO as GPIO
import threading


class SuperSonicSensor():
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

Ultraschall = SuperSonicSensor(24, 23, 1)

ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)

Utils = Utility()
Funcs = Functions(Utils, Ultraschall, Display=Display)
Utils.transferSensorData(Ultraschall, Funcs=Funcs, Display=Display)



p = mp.Process(target=Ultraschall.start_measurement())
p.start()

while True:
    time.sleep(0.2)
    print(Ultraschall.distance)