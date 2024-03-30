import time
from RobotCarClasses import *
from threading import Thread
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

Cam = Camera(video_stream=True)
Cam.start_processing()

global ESPHoldDistance, ESPHoldSpeed
ESPHoldDistance, ESPHoldSpeed = Utils.transferSensorData(Farbsensor, StartButton, StopButton, Buzzer1)

Utils.setupDataLog()



##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
Utils.Speed = 40
Utils.KP = 3.5
Utils.ED = 125 #Edge detection distance in cm


##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, YCutOffTop=165, YCutOffBottom=-1, SIZE=0, LineWaitTime=1, GyroWaitTime=1, Sensor=2):
    #Variables
    TIMEOUT = 0
    TIMEOUTBlock = 0.8
    corners = 0
    rounds = 0
    Sensor = 0
    direction = 0
    relative_angle = 0
    oldAngle = 0
    detect_new_block = True
    Last_Esp_Command = 0
    desired_distance_wall = 50
    timelastcorner = time.time()
    
    old_desired_distance_wall = 50
    
    coordinates_self = (640, 720) #x, y
    middledistance = 50
    GyroCornerAngle = 90

    FreezeSize = 150
    FreezeY = 350
    KP = 0.20
    
    desired_distance_to_block_red = 650
    desired_distance_to_block_green = -650
    
    #Hold Lane
    while Utils.running and rounds < 3:
        time.sleep(0.01)
                
        if direction == 0:
            relative_angle = Utils.Gyro.angle - oldAngle
            newAngle = oldAngle - GyroCornerAngle
            if Utils.Gyro.angle < newAngle and time.time() > TIMEOUT:
                corners = corners + 1
                timelastcorner = time.time()
                Utils.LogDebug(f"Corner: {corners}")
                Utils.Display.write(f"Corner: {corners}")
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1
                    Utils.Display.write(f"Corner: {corners}", f"Round: {rounds}")
                    
                oldAngle = newAngle
                TIMEOUT = time.time() + LineWaitTime
                
        else:
            newAngle = oldAngle + GyroCornerAngle
            if Utils.Gyro.angle > newAngle and time.time() > TIMEOUT:
                corners = corners + 1
                timelastcorner = time.time()
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
                    if block['y'] > YCutOffTop and block["size"] > SIZE:
                        pass
                    else:
                        block_array.remove(block)
                    
            if len(block_array) > 0:
                #Utils.LogDebug(f"Lenght of block_array: {len(block_array)}")
                #Sort blocks by size
                
                block_array.sort(key=lambda x: x['size'], reverse=True)
                
                nextBlock = block_array[0]

                if nextBlock['w'] > FreezeSize or nextBlock['y'] > FreezeY or old_desired_distance_wall < 15 or old_desired_distance_wall > 75:
                    detect_new_block = False
                    Cam.freeze = True
                    Time_detect_new_block = time.time()
                    Utils.LogInfo(f"Freeze")
                    
                    if timelastcorner + 1 > time.time():
                        nextBlock['position'] = "1"
                    elif timelastcorner + 2 > time.time():
                        nextBlock['position'] = "2"
                    elif timelastcorner + 3 > time.time():
                        nextBlock['position'] = "3"

                else:
                    nextBlock['distancex'] = -(coordinates_self[0] - nextBlock['mx'])
                    nextBlock['distancey'] = coordinates_self[1] - nextBlock['y']
                    #nextBlock['distance'] = math.sqrt(nextBlock['distancex']**2 + nextBlock['distancey']**2)
                    
                    if nextBlock['color'] == "red":
                        desired_distance_to_block = desired_distance_to_block_red
                    elif nextBlock['color'] == "green":
                        desired_distance_to_block = desired_distance_to_block_green
                        
                    distance_divider = (nextBlock['y'] / coordinates_self[1]) * 0.8

                    #print("Distance Divider: ", distance_divider)
                    # Calculation of desired distance to wall
                    error = (desired_distance_to_block - nextBlock['distancex']) * distance_divider
                    desired_distance_wall =  middledistance - error * KP
                    desired_distance_wall = int(desired_distance_wall)
                    Cam.desired_distance_wall = desired_distance_wall
                    
                    if desired_distance_wall < 15:
                        desired_distance_wall =  15 
                    elif desired_distance_wall > 75:
                        desired_distance_wall = 75

                    if desired_distance_wall > middledistance:
                        desired_distance_wall_other_direction = (100 - desired_distance_wall)
                        
                        #Send ESPHoldDistance new Sensor
                        if Sensor != 1:
                            Sensor = 1
                            Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                            Utils.LogInfo(f"Switched to Sensor 1")
                            
                        #Send ESPHoldDistance new Distance
                        if abs(desired_distance_wall_other_direction - old_desired_distance_wall) > 3:
                            Utils.usb_communication.sendMessage(f"D{desired_distance_wall_other_direction}", ESPHoldDistance)
                            Utils.LogInfo(f"New Distance {desired_distance_wall_other_direction}, Current Sensor: {Sensor}")
                            old_desired_distance_wall = desired_distance_wall_other_direction
                        
                    else:
                        if Sensor != 2:
                            Sensor = 2
                            Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                            Utils.LogInfo(f"Switched to Sensor 2")
                            
                        if abs(desired_distance_wall - old_desired_distance_wall) > 3:
                            Utils.usb_communication.sendMessage(f"D{desired_distance_wall}", ESPHoldDistance)
                            Utils.LogInfo(f"New Distance {desired_distance_wall}, Current Sensor: {Sensor}")
                            old_desired_distance_wall = desired_distance_wall
                        
            else:
                if time.time() > Last_Esp_Command + 1.5:
                    # Change this part based on the position of the block
                    """
                    if nextBlock['position'] == "1":
                        desired_distance_wall = 50
                    elif nextBlock['position'] == "2":
                        desired_distance_wall = 50
                    elif nextBlock['position'] == "3":
                        desired_distance_wall = 50
                    """
                    
                    time.sleep(0.1)
                    Utils.LogInfo(f"Desired Distance: {desired_distance_wall}, Current Sensor: {Sensor}")
                    if Sensor != 2:
                        Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                        Sensor = 2
                        
                    if desired_distance_wall > middledistance + 2:
                        desired_distance_wall -= 4
                        Utils.usb_communication.sendMessage(f"D{desired_distance_wall}", ESPHoldDistance)

                        Last_Esp_Command = time.time()
                        
                    elif desired_distance_wall < middledistance - 2:
                        desired_distance_wall += 4
                        Utils.usb_communication.sendMessage(f"D{desired_distance_wall}", ESPHoldDistance)

                        Last_Esp_Command = time.time()
                        
                    elif desired_distance_wall != middledistance:
                        desired_distance_wall = middledistance
                        Utils.usb_communication.sendMessage(f"D{desired_distance_wall}", ESPHoldDistance)
                        
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
          
                    
        responses = Utils.usb_communication.getResponses(ESPHoldDistance)
        if (responses != None and responses != []):
            Utils.LogDebug(f"Responses: {responses}")
            for response in responses:
                if "Drive direction counterclockwise" in responses:
                    direction = 1
                if "Drive direction clockwise" in responses:
                    direction = 0
                if "SD1:" in responses:
                    Utils.SensorDistance1 = float(response.split(":")[1].strip())
                if "SD2:" in responses:
                    Utils.SensorDistance2 = float(response.split(":")[1].strip())
        
        Utils.LogData()
        
    #time.sleep(2)



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
                
            @app.route('/data_feed')
            def data_feed():
                return jsonify(Utils.data_feed())

            # Run the server in a separate thread
            server_thread = Thread(target=app.run, kwargs={'host':'0.0.0.0', 'threaded':True})
            server_thread.start()
                
        Utils.StartRun()
        HoldLane(Utils)
    
    except:
        Utils.LogError(traceback.format_exc())
        
    finally:
        Utils.StopRun()