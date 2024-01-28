import time
from RobotCarClasses import *
from threading import Thread
import math
import traceback


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
Display = DisplayOled(ADC, Gyro, Utils=Utils)

Cam = Camera(video_stream=True)
Cam.start_processing()

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
Utils.ED = 125 #Edge detection distance in cm


##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, YCutOffTop=1000000, YCutOffBottom=-1, SIZE=0, LineWaitTime=1, GyroWaitTime=1, Sensor=2):
    #Variables
    TIMEOUT = 0
    TIMEOUTBlock = 2
    corners = 0
    rounds = 0
    Sensor = 0
    direction = 0
    relative_angle = 0
    oldAngle = 0
    detect_new_block = True
    Last_Esp_Command = 0
    desired_distance_wall = 50
    
    old_desired_distance_wall = 50
    
    coordinates_self = (640, 720) #x, y
    middledistance = 50
    GyroCornerAngle = 90
    FreezeSize = 35000
    KP = 0.13
    
    desired_distance_to_block_red = 400
    desired_distance_to_block_green = -400
    
    #Hold Lane
    while Utils.running and rounds < 3:
        time.sleep(0.001)
                
        #Count rounds with Gyro
        Utils.angle = Gyro.angle
        if direction == 0:
            relative_angle = Gyro.angle - oldAngle
            newAngle = oldAngle - GyroCornerAngle
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
            relative_angle = Gyro.angle - oldAngle
            newAngle = oldAngle + GyroCornerAngle
            if Gyro.angle > newAngle and time.time() > TIMEOUT:
                corners = corners + 1
                Utils.LogDebug(f"Corner: {corners}")
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1
                    
                oldAngle = newAngle
                TIMEOUT = time.time() + LineWaitTime
        
        
        #get objects and calculate new distance
        if detect_new_block:
            block_array = Cam.block_array
            if len(block_array) > 0:
                #Delete blocks not meeting requirements
                for block in block_array:
                    if block['my'] < YCutOffTop and block['my'] > YCutOffBottom and block["size"] > SIZE:
                        pass
                    else:
                        block_array.remove(block)
                    
            if len(block_array) > 0:
                #Utils.LogDebug(f"Lenght of block_array: {len(block_array)}")
                #Sort blocks by size
                
                block_array.sort(key=lambda x: x['size'], reverse=True)
                
                nextBlock = block_array[0]
                
                if nextBlock['w'] * nextBlock['h'] > FreezeSize or old_desired_distance_wall < 25 or old_desired_distance_wall > 75:
                    detect_new_block = False
                    Cam.freeze = True
                    Time_detect_new_block = time.time()
                    Utils.LogInfo(f"Freeze")

                else:
                    nextBlock['distancex'] = -(coordinates_self[0] - nextBlock['mx'])
                    nextBlock['distancey'] = coordinates_self[1] - nextBlock['y']
                    #nextBlock['distance'] = math.sqrt(nextBlock['distancex']**2 + nextBlock['distancey']**2)
                    
                    if nextBlock['color'] == "red":
                        desired_distance_to_block = desired_distance_to_block_red
                    elif nextBlock['color'] == "green":
                        desired_distance_to_block = desired_distance_to_block_green
                        
                    distance_divider = (nextBlock['y'] / coordinates_self[1]) * 1.3
                    #print("Distance Divider: ", distance_divider)
                    
                    error = (desired_distance_to_block - nextBlock['distancex']) * distance_divider
                    desired_distance_wall =  middledistance - error * KP
                    desired_distance_wall = int(desired_distance_wall)
                    Cam.desired_distance_wall = desired_distance_wall
                    
                    if desired_distance_wall < 25:
                        desired_distance_wall =  25 
                    elif desired_distance_wall > 75:
                        desired_distance_wall = 75

                    if desired_distance_wall > middledistance:
                        desired_distance_wall_other_direction = (100 - desired_distance_wall)
                            
                        #Send ESPHoldDistance new Distance
                        if abs(desired_distance_wall_other_direction - old_desired_distance_wall) > 3:
                            Utils.EspHoldDistance.write(f"D{desired_distance_wall_other_direction}\n".encode())
                            Utils.LogInfo(f"New Distance {desired_distance_wall_other_direction}, Current Sensor: {Sensor}")
                            old_desired_distance_wall = desired_distance_wall
                        
                        #Send ESPHoldDistance new Sensor
                        if Sensor != 1:
                            Sensor = 1
                            Utils.EspHoldDistance.write(f"S1\n".encode())
                            Utils.LogInfo(f"Switched to Sensor 1")
                            
                    else:
                        if abs(desired_distance_wall - old_desired_distance_wall) > 3:
                            Utils.EspHoldDistance.write(f"D{desired_distance_wall}\n".encode())
                            Utils.LogInfo(f"New Distance {desired_distance_wall}, Current Sensor: {Sensor}")
                            old_desired_distance_wall = desired_distance_wall
                        
                        if Sensor != 2:
                            Sensor = 2
                            Utils.EspHoldDistance.write(f"S2\n".encode())
                            Utils.LogInfo(f"Switched to Sensor 2")
                            
            else:
                if time.time() > Last_Esp_Command + 1:
                    time.sleep(0.1)
                    Utils.LogInfo(f"Desired Distance: {desired_distance_wall}, Current Sensor: {Sensor}")
                    if Sensor != 2:
                        Utils.EspHoldDistance.write(f"S2\n".encode())
                        Sensor = 2
                        time.sleep(0.05)
                        
                    if desired_distance_wall > middledistance + 2:
                        desired_distance_wall -= 4
                        Utils.EspHoldDistance.write(f"D{desired_distance_wall}\n".encode())

                        Last_Esp_Command = time.time()
                        
                    elif desired_distance_wall < middledistance - 2:
                        desired_distance_wall += 4
                        Utils.EspHoldDistance.write(f"D{desired_distance_wall}\n".encode())

                        Last_Esp_Command = time.time()
                        
                    elif desired_distance_wall != middledistance:
                        desired_distance_wall = middledistance
                        Utils.EspHoldDistance.write(f"D{desired_distance_wall}\n".encode())
                        
                        Last_Esp_Command = time.time()
                        
                    Cam.desired_distance_wall = desired_distance_wall
                    if desired_distance_wall <= 30:
                        desired_distance_wall = 31
                    elif desired_distance_wall >= 70:
                        desired_distance_wall = 69
                    old_desired_distance_wall = desired_distance_wall
                        
        else:
            if time.time() > Time_detect_new_block + TIMEOUTBlock:
                detect_new_block = True
                Cam.freeze = False
                Utils.LogInfo("Detect new block")
                    
        #check for direction
        if Utils.EspHoldDistance.in_waiting > 0:
            time.sleep(0.1)
            response = Utils.EspHoldDistance.read(Utils.EspHoldDistance.in_waiting).decode()
            if "Drive direction counterclockwise" in response:
                direction = 1
            elif "Drive direction clockwise" in response:
                direction = 0
                
            Utils.LogDebug(f"Response from EspHoldDistance: {response}")
        
        Utils.LogData()    



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    try: 
        #start flask server if needed
        if Cam.video_stream:
            app = Flask(__name__)
            
            @app.route('/')
            def index():
                return render_template('index.html')


            @app.route('/video_feed')
            def video_feed():
                return Response(Cam.video_frames(),
                                mimetype='multipart/x-mixed-replace; boundary=frame')

            # Run the server in a separate thread
            server_thread = Thread(target=app.run, kwargs={'host':'0.0.0.0', 'threaded':True})
            server_thread.start()
                
        Utils.StartRun()
        HoldLane(Utils)
    
    except:
        Utils.LogError(traceback.format_exc())
        
    finally:
        Utils.StopRun()