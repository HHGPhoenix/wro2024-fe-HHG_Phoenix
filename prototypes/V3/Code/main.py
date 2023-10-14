import RPi.GPIO as GPIO
import time
from RobotCarClasses import *



##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################

global P, DISTANCE 

P = 5

DISTANCE = 15



##########################################################
##                                                      ##
##                Sensor Initalization                  ##
##                                                      ##
##########################################################
    
Farbsensor = ColorSensor()

Motor1 = Motor(1000, 27, 17, 22)
Motor1.start()

Ultraschall1 = SuperSonicSensor(24, 23, 1)
Ultraschall2 = SuperSonicSensor(8, 25, 2)
Servo1 = Servo(18, 50)
StartButton = Button(7)
StopButton = Button(26)
#Gyro = Gyroscope()
ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)


Utils = Utility()
Funcs = Functions(Utils, Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Display)
Utils.transferSensorData(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Funcs, Display)


##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################

def Variation1():
    Utils.running = True
    
    #Main loop
    while Utils.running and Funcs.rounds < 3:
        time.sleep(0.01)
        try:
            Utils.StartRun(70, 0)
            while True:
                print(Ultraschall2.distance)
            
        except Exception as e:
            print(e)
            Utils.StopRun()
            
            
def Variation2():
    Utils.running = True
    
    #Main loop
    Utils.StartRun(80, 0)
    while Utils.running and Funcs.rounds < 3:
        time.sleep(0.01)
        try:
            Funcs.HoldDistance(50, True)
            Funcs.DriveCorner("f", 80, 100, 2)
        except Exception as e:
            print(e)
            Utils.cleanup()
            
    Utils.StopRun()  
    
    
def Variation3():
    Utils.running = True
    
    #Main loop
    Utils.StartRun(80, 0)
    while Utils.running and Funcs.rounds < 3:
        time.sleep(0.01)
        try:
            Funcs.CalMiddle()
            Funcs.HoldDistance(Funcs.middledistance, True)
            Funcs.DriveCorner("f", 80, 100, 2)
        except:
            Utils.cleanup()
            
    Utils.StopRun()   
   
if __name__ == "__main__": 
    Variation1()