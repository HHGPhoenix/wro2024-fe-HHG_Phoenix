import time
from RobotCarClasses import *
import RPi.GPIO as GPIO



##########################################################
##                                                      ##
##        Sensor / Class Initalization                  ##
##                                                      ##
##########################################################    
Utils = Utility()

Farbsensor = ColorSensor(Utils)

StartButton = Button(5, Utils)
StopButton = Button(6, Utils)

Buzzer1 = Buzzer(12, Utils)

Gyro = Gyroscope(Utils)


ADC = AnalogDigitalConverter(Utils)
Display = DisplayOled(ADC, Gyro, Farbsensor, Utils)

Pixy = PixyCam(Utils)
Pixy.start_reading()

Utils.transferSensorData(Farbsensor, StartButton, StopButton, Display, ADC, Buzzer1, Gyro, Pixy)

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
Utils.ED = 125 #Edge detection distance in cm



##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, YCutOffTop=200, YCutOffBottom=0, BlockWaitTime=2, WaitTime=0.01, Lane=1, SIZE=0, colorTemperature=1, LineWaitTime=1, Sensor=2):
    #Variables
    TIMEOUT = 0
    TIMEOUTPixy = 0
    corners = 0
    rounds = 0
    Sensor = 0
    direction = 0
    oldAngle = 0
    KPNormal = True
    
    #Hold Lane
    while Utils.running and rounds < 3:
        try:
            time.sleep(0.001)
            #Choose distance and sensor to hold lane based on Pixy and direction
            if direction == 0:
                if Lane == 0 and Sensor != 1:
                    DISTANCE = 25
                    Sensor = 1
                    print("Switched to Sensor 1")
                    Utils.EspHoldDistance.write(f"D{DISTANCE}\n".encode())
                    Utils.EspHoldDistance.write(f"S1\n".encode())
                    
                elif Lane == 1 and Sensor != 11:
                    DISTANCE = 50
                    Sensor = 11
                    print("Switched to Sensor 1")
                    Utils.EspHoldDistance.write(f"D{DISTANCE}\n".encode())
                    Utils.EspHoldDistance.write(f"S1\n".encode())
                    
                elif Lane == 2 and Sensor != 2:
                    DISTANCE = 25
                    Sensor = 2
                    print("Switched to Sensor 2")
                    Utils.EspHoldDistance.write(f"D{DISTANCE}\n".encode())
                    Utils.EspHoldDistance.write(f"S2\n".encode())
            else:
                if Lane == 0 and Sensor != 2:
                    DISTANCE = 25
                    Sensor = 2
                    print("Switched to Sensor 2")
                    Utils.EspHoldDistance.write(f"D{DISTANCE}\n".encode())
                    Utils.EspHoldDistance.write(f"S2\n".encode())
                    
                elif Lane == 1 and Sensor != 22:
                    DISTANCE = 50
                    Sensor = 22
                    print("Switched to Sensor 2")
                    Utils.EspHoldDistance.write(f"D{DISTANCE}\n".encode())
                    Utils.EspHoldDistance.write(f"S2\n".encode())
                    
                elif Lane == 2 and Sensor != 1:
                    DISTANCE = 25
                    Sensor = 1
                    print("Switched to Sensor 1")
                    Utils.EspHoldDistance.write(f"D{DISTANCE}\n".encode())
                    Utils.EspHoldDistance.write(f"S1\n".encode())
                    
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
                
            #get Pixy objects and calculate new lane
            if time.time() > TIMEOUTPixy:
                if KPNormal == False:
                    Utils.EspHoldDistance.write(f"KP{3}\n".encode())
                    KPNormal = True
                time.sleep(0.1)
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
                        
                        Utils.EspHoldDistance.write(f"KP{2}\n".encode())
                        KPNormal = False
                        time.sleep(0.1)
                        Utils.LogDebug(f"Signature: {signature}")
                        
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
              
            #check for direction
            if Utils.EspHoldDistance.in_waiting > 0:
                response = Utils.EspHoldDistance.read(Utils.EspHoldDistance.in_waiting).decode()
                if "Drive direction counterclockwise" in response:
                    direction = 1
                elif "Drive direction clockwise" in response:
                    direction = 0
                    
                Utils.LogDebug(f"Response from EspHoldDistance: {response}")
            
            Utils.LogData()    
            
        except Exception as e:
            Utils.LogError(e)
            Utils.StopRun()
            
    Utils.LogDebug("HoldLane finished")
    Utils.StopRun()



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    try: 
        GPIO.setmode(GPIO.BCM)
        Utils.StartRun()
        Pixy.LED(1)
        HoldLane(Utils, SIZE=200, colorTemperature=2000)
    
    except Exception as e:
        Utils.LogError(e)
        Utils.StopRun()