import time
from RobotCarClasses import Motor, Servo, ColorSensor, SuperSonicSensor, Button, AnalogDigitalConverter, DisplayOled, Utility



##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
SPEED = 70
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
Farbsensor = ColorSensor()

Motor1 = Motor(1000, 27, 17, 22)
Motor1.start()
Servo1 = Servo(18, 50)

Ultraschall1 = SuperSonicSensor(24, 23, 1)
Ultraschall2 = SuperSonicSensor(8, 25, 2)

StartButton = Button(7)
StopButton = Button(26)
#Gyro = Gyroscope()
ADC = AnalogDigitalConverter()
Display = DisplayOled(ADC)


Utils = Utility()
Utils.transferSensorData(Ultraschall1, Ultraschall2, Farbsensor, Motor1, Servo1, StartButton, StopButton, Display)



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
    Motor1.drive("f", speed)
    
    #Hold Distance to wall
    while Utils.running and rounds < 3:
        try:
            time.sleep(0.01)
            Error = Utils.Ultraschall2.distance - DISTANCE
            Correction = P * Error
            
            #Limit steering
            if Correction > 95:
                Correction = 95
            elif Correction < -95:
                Correction = -95
            
            #Count rounds with ColorSensor
            if Utils.Farbsensor.color_temperature >= ColorTemperature - 100 and Utils.Farbsensor.color_temperature <= colorTemperature + 100 and time.time() > TIMEOUT:
                corners = corners + 1
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
        Utils.StartRun(SPEED, 0, "f")
        HoldDistance(Utils, DISTANCETOWALL, KP, SPEED, LINECOLORTEMPERATURE, 0.5)
    
    except Exception as e:
        print(e)
        Utils.StopRun()