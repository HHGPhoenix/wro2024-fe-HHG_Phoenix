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
#Farbsensor.start_measurement()

Motor1 = Motor(1000, 27, 17, 22)
#Motor1.start()

Ultraschall1 = SuperSonicSensor(24, 23)
Ultraschall1.start_measurement()

Ultraschall2 = SuperSonicSensor(12, 25)
#Ultraschall2.start_measurement()

Pixy = PixyCam()
Pixy.start_reading()

Servo1 = Servo(18, 50)

StartButton = Button(8)

StopButton = Button(7)
#StopButton.start_StopButton()

#CException = CustomException()

Utils = Utility(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Pixy)
Funcs = Functions(Utils)



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################

Utils.running = True
#clearing the "sensor_data.txt" file

with open("sensor_data.txt", "w") as data_file:
    data_file.write("")

while Utils.running and Funcs.rounds < 3:
    time.sleep(0.01)
    try:
        Utils.StartRun(80, 0)
        Funcs.HoldLane([1, 1], 1, 1, False, 5, 80, "f", 2300)
        with open("sensor_data.txt", "a") as data_file:
                    data_file.write(f"sDistance; {Ultraschall1.sDistance}; distance; {Ultraschall1.distance}\n")
    except:
        Utils.cleanup()
        
Utils.StopRun()