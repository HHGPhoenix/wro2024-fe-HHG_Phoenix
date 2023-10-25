import time
from RobotCarClasses import *
import RPi.GPIO as GPIO



##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
SPEED = 50
DISTANCETOWALL = 30
KP = 2.5
LINECOLORTEMPERATURE = 2000
NUMBERSLOTS = 8

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

Buzzer1 = Buzzer(18, Utils)
SpeedSens = SpeedSensor(26, NUMBERSLOTS, Utils, 45)

Utils.transferSensorData(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Display, ADC, Buzzer1, SpeedSens)
Utils.setupLog()



##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldDistance(Utils, DISTANCE=50, P=5, speed=0, ColorTemperature=1, LineWaitTime=0.5):
    #Variables
    TIMEOUT = 0
    corners = 0
    rounds = 0
    
    #Start driving
    Utils.Motor1.drive("f", speed)
    
    #Hold Distance to wall
    while Utils.running and rounds < 3:
        try:
            time.sleep(0.001)
            Error = Utils.Ultraschall2.distance - DISTANCE
            #print(Utils.Ultraschall2.distance)
            Correction = P * Error
            
            #Limit steering
            if Correction > 90:
                Correction = 90
            elif Correction < -90:
                Correction = -90
                
            Utils.Servo1.steer(Correction)
            
            #Count rounds with ColorSensor
            if Utils.Farbsensor.color_temperature >= ColorTemperature - 200 and Utils.Farbsensor.color_temperature <= ColorTemperature + 200 and time.time() > TIMEOUT:
                corners = corners + 1
                Utils.Display.write(f"Rounds: {rounds}", f"Corners: {corners}")
                Utils.Buzzer1.buzz(1000, 80, 0.1)
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1 
                    print(f"Round: {rounds}")
                    
                TIMEOUT = time.time() + LineWaitTime
                
            
                
        except Exception as exception:
            print(exception)
            Utils.StopRun()



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    try:  
        GPIO.setmode(GPIO.BCM)
        Utils.StartRun(SPEED, 0, "f")
        Motor1.setMotorSpeed(3)
        HoldDistance(Utils, DISTANCETOWALL, KP, SPEED, LINECOLORTEMPERATURE, 0.3)
    
    except Exception as e:
        print(e)
        Utils.StopRun()