import time
from RobotCarClasses import *



##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
SPEED = 0
DISTANCETOWALL = 50
KP = 5
LINECOLORTEMPERATURE = 2500

#Variables
rounds = 0



##########################################################
##                                                      ##
##        Sensor / Class Initalization                  ##
##                                                      ##
##########################################################    
Utils = Utility()

Farbsensor = ColorSensor(Utils)

Motor1 = Motor(1000, 21, 20, 16, Utils)
Motor1.start()
Servo1 = Servo(27, 50, Utils)

Ultraschall1 = SuperSonicSensor(23, 24, 1, Utils)
Ultraschall2 = SuperSonicSensor(19, 13, 2, Utils)

StartButton = Button(17, Utils)
StopButton = Button(4, Utils)

#Gyro = Gyroscope()
ADC = AnalogDigitalConverter(Utils)
Display = DisplayOled(ADC, Utils)

Utils.transferSensorData(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Display, ADC)

Buzzer1 = Buzzer(18, Utils)

Buzzer1.buzz(1000, 80, 1)