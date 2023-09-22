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

Motor1 = Motor(1000, 13, 15, 11)
Motor1.start()

Ultraschall1 = SuperSonicSensor(22, 32)
Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(16, 18)
Ultraschall2.start_measurement()

#PixyCam1 = PixyCam()

Servo1 = Servo(12, 50)

StartButton = Button(37)
StopButton = Button(33)

CException = CustomException()

Utils = Utility(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton)
Funcs = Functions(Utils)

##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################

running = True
rounds = 0
corners = 0

#Main loop
while Utils.running:
    try:
        Utils.StartRun(80, 0)
        Funcs.HoldDistance()
        
    except:
        Utils.cleanup()