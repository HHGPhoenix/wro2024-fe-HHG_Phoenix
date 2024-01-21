import time
from RobotCarClasses import *
import RPi.GPIO as GPIO


##########################################################
##                                                      ##
##        Sensor / Class Initalization                  ##
##                                                      ##
##########################################################    
Utils = Utility()

#Farbsensor = ColorSensor(Utils)
Farbsensor = None

StartButton = Button(5, Utils)
StopButton = Button(6, Utils)

Buzzer1 = Buzzer(12, Utils)

Gyro = Gyroscope(Utils)


ADC = AnalogDigitalConverter(Utils)
Display = DisplayOled(ADC, Gyro, Farbsensor, StopButton, Utils)

Utils.transferSensorData(Farbsensor, StartButton, StopButton, Display, ADC, Buzzer1, Gyro)

Utils.setupLog()
Utils.setupDataLog()



##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
Utils.Speed = 50
Utils.KP = 3.5
LINECOLORTEMPERATURE = 2000
Utils.ED = 125 #Edge detection distance in cm



##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldDistance(Utils, colorTemperature=1, LineWaitTime=1):
    Utils.LogDebug("HoldDistance started")
    #Variables
    TIMEOUT = 0
    TIMEOUT2 = 0
    corners = 0
    corners2 = 0
    rounds = 0
    rounds2 = 0
    direction = 0
    Sensor = 0
    oldAngle = 0
    

    #Hold Lane
    while Utils.running and rounds < 3:
        try:
            time.sleep(0.001)
            #Choose distance and sensor to hold lane based on Pixy and direction
            if direction == 0:
                if Sensor != 1:
                    Sensor = 1   
                    Utils.EspHoldDistance.write(f"S1\n".encode())
            else:
                if Sensor != 2:
                    Sensor = 2
                    Utils.EspHoldDistance.write(f"S2\n".encode())
             
                
            """
            #Count rounds with ColorSensor
            if Utils.Farbsensor.color_temperature >= colorTemperature and time.time() > TIMEOUT2:
                corners2 = corners2 + 1
                Utils.LogDebug(f"Corner2: {corners2}")
                if corners2 == 4:
                    corners2 = 0
                    rounds2 = rounds2 + 1
                    
                TIMEOUT2 = time.time() + LineWaitTime
            """
            
            #Count rounds with Gyro
            Utils.angle = Gyro.angle
            if direction == 0:
                newAngle = oldAngle -90
                if Gyro.angle < newAngle and time.time() > TIMEOUT:
                    corners = corners + 1
                    Utils.LogDebug(f"Corner: {corners}")
                    Display.write(f"Corner: {corners}")
                    if corners == 4:
                        corners = 0
                        rounds = rounds + 1
                        Display.write(f"Corner: {corners}", f"Round: {rounds}")
                        
                    oldAngle = newAngle
                    TIMEOUT = time.time() + LineWaitTime
            else:
                newAngle = oldAngle + 90
                if Gyro.angle > newAngle and time.time() > TIMEOUT:
                    corners = corners + 1
                    Utils.LogDebug(f"Corner: {corners}")
                    if corners == 4:
                        corners = 0
                        rounds = rounds + 1
                        
                    oldAngle = newAngle
                    TIMEOUT = time.time() + LineWaitTime
              
            #check for direction
            if Utils.EspHoldDistance.in_waiting > 0:
                response = Utils.EspHoldDistance.read(Utils.EspHoldDistance.in_waiting).decode("utf-8")
                if "Drive direction counterclockwise" in response:
                    direction = 1
                    time.sleep(0.1)
                    Utils.EspHoldDistance.write(f"D{70}\n".encode())
                elif "Drive direction clockwise" in response:
                    direction = 0
                    time.sleep(0.1)
                    Utils.EspHoldDistance.write(f"D{70}\n".encode())
                    
                Utils.LogDebug(f"Response from EspHoldDistance: {response}")
            
            Utils.LogData()   
            
        except Exception as e:
            Utils.LogError(e)
            Utils.StopRun()
            
    Utils.LogDebug("HoldDistance finished")
    Utils.StopRun()
    


##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    try: 
        GPIO.setmode(GPIO.BCM)
        print("Init started")
        Utils.StartRun()
        print("Init finished")
        HoldDistance(Utils, colorTemperature=4000)
    
    except Exception as e:
        Utils.LogError(e)
        Utils.StopRun()