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

Pixy = PixyCam()
Pixy.start_reading()

Farbsensor = ColorSensor()
Farbsensor.start_measurement()

Motor1 = Motor(1000, 27, 17, 22)
Motor1.start()

Ultraschall1 = SuperSonicSensor(24, 23)
#Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(8, 25)
Ultraschall2.start_measurement()

Servo1 = Servo(18, 50)

StartButton = Button(7)

StopButton = Button(1)
StopButton.start_StopButton()

#CException = CustomException()

Funcs = Functions(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy)
Utils = Utility(Funcs, Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy)
Funcs.information(Utils)



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################

Utils.running = True
#Utils.StartData("192.168.178.56", 12345)

while Utils.running and Funcs.rounds < 3:
    time.sleep(0.01)
    try:
        #print(Pixy.output[0].m_signature)
        Utils.StartRun(65, 0)
        Funcs.HoldLane()
    except Exception as e:
        print(e)
        Utils.cleanup()
        
Utils.StopRun()