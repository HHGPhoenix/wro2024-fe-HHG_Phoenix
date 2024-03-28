
    #Start a new thread for updating the Display
    def start_update(self):
        self.threadStop = 0
        self.thread1 = threading.Thread(target=self.update, daemon=True)
        self.thread1.start()
        
        if self.Gyro != None:
            self.Gyro.threadStop = 0
            self.thread2 = threading.Thread(target=self.Gyro.get_angle, daemon=True)
            self.thread2.start()
        
        if self.ColorSensor != None:
            self.ColorSensor.threadStop = 0
            self.thread3 = threading.Thread(target=self.ColorSensor.read, daemon=True)
            self.thread3.start()
            
        if self.Button != None:
            self.Button.threadStop = 0
            self.thread4 = threading.Thread(target=self.Button.read_StopButton, daemon=True)
            self.thread4.start()
            
            
    #Stop the thread for updating the Display      
    def stop_update(self):

        self.threadStop = 1
        
        if self.Gyro != None:
            self.Gyro.threadStop = 1
            
        if self.ColorSensor != None:
            self.ColorSensor.threadStop = 1
            
            
    #Start a new thread for reading the sensor        
    def start_measurement(self):
        self.sensor.active = True
        self.threadStop = 0
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()


        
# was in every class 
self.Utils = Utils

# was in DisplayOLED init with: (self, ADC=None, Gyro=None, Farbsensor=None, Button=None):
self.ADC = ADC
self.Gyro = Gyro
self.ColorSensor = Farbsensor
self.Button = Button