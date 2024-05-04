##########################################################
##                                                      ##
##                 Old-Classes                          ##
##                                                      ##
##########################################################

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



#A class for reading the PixyCam ############################ WIP - may be abandoned #########################################            
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
        
    
    #Set a motor speed that gets controlled by the Speed Sensor, if it was started   
    def setMotorSpeed(self, Speed):
        self.drive("f", 100)
        time.sleep(0.01)
        self.MotorSpeed = Speed
        
           

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