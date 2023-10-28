import time
from RobotCarClasses import *
import RPi.GPIO as GPIO
import serial



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

StartButton = Button(17, Utils)
StopButton = Button(4, Utils)

#Gyro = Gyroscope()
ADC = AnalogDigitalConverter(Utils)
Display = DisplayOled(ADC, Utils)

Buzzer1 = Buzzer(18, Utils)
SpeedSens = SpeedSensor(26, NUMBERSLOTS, Utils, 45)

Pixy = PixyCam(Utils)
Pixy.start_reading()

Utils.transferSensorData(Farbsensor=Farbsensor, Motor1=Motor1, StartButton=StartButton, StopButton=StopButton, Display=Display, ADC=ADC, Buzzer1=Buzzer1, SpeedSens=SpeedSens, Pixy=Pixy)
Utils.setupLog()



##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, YCutOffTop=200, YCutOffBottom=0, BlockWaitTime=1, WaitTime=0.01, Lane=1, SIZE=1, P=2.5, ServoWaitTime=0.015, direction="f", colorTemperature=1, LineWaitTime=1, Sensor=2):
    #Variables
    TIMEOUT = 0
    TIMEOUTPixy = 0
    ServoTimeout = 0
    corners = 0
    rounds = 0
    Sensor = 0
    
    #Hold Lane
    while Utils.running and rounds < 3:
        try:
            time.sleep(0.001)
            #Choose distance and sensor to hold lane
            if Lane == 0 and Sensor != 2:
                DISTANCE = 25
                Sensor = 2
                print("Switched to Sensor 2")
                arduino.write(f"D {DISTANCE}\n".encode())
                arduino.write(f"S2\n".encode())
                
            elif Lane == 1 and Sensor != 2:
                DISTANCE = 50
                Utils.toggle_supersonic_sensor(2)
                Sensor = 2
                print("Switched to Sensor 2")
                arduino.write(f"D {DISTANCE}\n".encode())
                arduino.write(f"S2\n".encode())
                
            elif Lane == 2 and Sensor != 1:
                DISTANCE = 25
                Utils.toggle_supersonic_sensor(1)
                Sensor = 1
                print("Switched to Sensor 1")
                arduino.write(f"D {DISTANCE}\n".encode())
                arduino.write(f"S1\n".encode())
            
            #Count rounds with ColorSensor
            if Utils.Farbsensor.color_temperature >= colorTemperature - 200 and Utils.Farbsensor.color_temperature <= colorTemperature + 200 and time.time() > TIMEOUT:
                corners = corners + 1
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1
                    
                TIMEOUT = time.time() + LineWaitTime
                
            #get Pixy objects and calculate new lane
            if time.time() > TIMEOUTPixy:
                count = Utils.Pixy.count
                if count > 0:
                    NextObject = -1
                    for x in range(count):
                        yCoord = Utils.Pixy.output[x].m_y
                        
                        if yCoord < YCutOffTop and yCoord > YCutOffBottom:
                            
                            size = Utils.Pixy.output[x].m_width * Utils.Pixy.output[x].m_height
                            
                            if size >= SIZE:
                                NextObject = x
                                break
                            
                        else:
                            NextObject = -1
                    
                    if NextObject > -1:
                        signature = Utils.Pixy.output[NextObject].m_signature
                        
                        #Green Block
                        if signature == 1:
                            Lane = 0
                        #Red Block
                        elif signature == 2:
                            Lane = 2
                        #No Block found
                        else:
                            Lane = 1

                        TIMEOUTPixy = time.time() + BlockWaitTime
                        
                    else:
                        Lane = 1
                        TIMEOUTPixy = time.time() + WaitTime

                else:
                    Lane = 1
                    TIMEOUTPixy = time.time() + WaitTime
                    
                
        except Exception as e:
            Utils.LogError(e)
            Utils.StopRun()



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    try: 
        arduino = serial.Serial('COM3', 9600)
        GPIO.setmode(GPIO.BCM)
        Utils.StartRun(SPEED, 0, "f")
        Motor1.setMotorSpeed(3.3)
        Pixy.LED(0)
        HoldLane(Utils, SIZE=500, colorTemperature=2000)
    
    except Exception as e:
        Utils.LogError(e)
        Utils.StopRun()