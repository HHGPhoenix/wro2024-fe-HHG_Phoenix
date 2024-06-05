from I2C_classes import DisplayOled, Gyroscope, AnalogDigitalConverter, ColorSensor
import threading

class I2Ccommunication:
    def __init__(self, Utils):
        self.Utils = Utils
        
        self.Display = DisplayOled(self.Utils)
        self.ADC = AnalogDigitalConverter()
        self.Gyro = Gyroscope()
        self.ColorSensor = ColorSensor()
    
        
    # start all threads for I2C communication
    def start_threads(self):
        # self.Display.threadStop = 0
        # tOled = threading.Thread(target=self.Display.update, daemon=True)
        # tOled.start()
        
        self.Gyro.threadStop = 0
        tGyro = threading.Thread(target=self.Gyro.read, daemon=True)
        tGyro.start()
        
        self.ADC.threadStop = 0
        tADC = threading.Thread(target=self.ADC.read, daemon=True)
        tADC.start()
        
        # self.ColorSensor.threadStop = 0
        # tColor = threading.Thread(target=self.ColorSensor.read, daemon=True)
        # tColor.start()
    
    
    # stop all threads for I2C communication
    def stop_threads(self):
        self.Display.threadStop = 1
        self.Gyro.threadStop = 1
        self.ADC.threadStop = 1
        # self.ColorSensor.threadStop = 1