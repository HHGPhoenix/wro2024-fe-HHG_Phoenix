import keyboard
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

Ultraschall1 = SuperSonicSensor(24, 23)
Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(12, 25)
#Ultraschall2.start_measurement()

#PixyCam1 = PixyCam()


Servo1 = Servo(18, 50)

StartButton = Button(8)

StopButton = Button(7)
StopButton.start_StopButton()

#CException = CustomException()

Utils = Utility(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton)
Funcs = Functions(Utils)



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
            Utils.StartRun(80, 0)
            Funcs.HoldDistance(50, False, 5, 0, "f", 2200)
            
        except Exception as e:
            print(e)
            Utils.cleanup()
            
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
    
    
Variation1()
