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
Farbsensor.start_measurement()

Motor1 = Motor(1000, 27, 17, 22)
Motor1.start()

Ultraschall1 = SuperSonicSensor(24, 23, 1)
#Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(8, 25, 2)
Ultraschall2.start_measurement()

Servo1 = Servo(18, 50)

StartButton = Button(7)

StopButton = Button(26)
StopButton.start_StopButton()

#Gyro = Gyroscope()
ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)


Utils = Utility()
Funcs = Functions(Utils, Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Display)
Utils.transferSensorData(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Funcs, Display)
with open(Utils.file_path, 'w'):
    pass  # Using 'pass' as a placeholder for no content
time.sleep(1)


Display.start_update()
time.sleep(1)


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
            Funcs.HoldDistance(40, False, 5, 70, "f", 2500)
            
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