import time
from RobotCarClasses import *
from threading import Thread
import traceback
import multiprocessing as mp


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

global ESPHoldDistance, ESPHoldSpeed
ESPHoldDistance, ESPHoldSpeed = Utils.transferSensorData(Farbsensor, StartButton, StopButton, Buzzer1, 50, Cam)

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

middledistance = 50
coordinates_self = (640, 720) #x, y
desired_distance_to_block_red = -700
desired_distance_to_block_green = 700
KP = 0.22
DISTANCE_DIVIDER_VALUE = 0.8

##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, YCutOffTop=0, YCutOffBottom=-1, SIZE=0, LineWaitTime=1, GyroWaitTime=1, Sensor=2):
    #Variables
    TIMEOUT = 0
    TIMEOUTBlock = 0.5
    corners = 0
    rounds = 0
    Sensor = 2
    direction = -1
    relative_angle = 0
    oldAngle = 0
    detect_new_block = True
    Last_Esp_Command = 0
    desired_distance_wall = 50
    timelastcorner = time.time()
    Inverted = False
    
    old_desired_distance_wall = 50
    
    GyroCornerAngle = 70

    FreezeSize = 150
    FreezeY = 350
    FreezeBlock = False
    
    SensorFirstCornerChanged = False
    active_block_drive = False
    ESP_adjusted = False

    #Hold Lane
    while Utils.running and rounds < 3:
        StartTime = time.time()
        time.sleep(0.001)
                
                
        #Utils.LogDebug(f"direction: {direction}, Sensor: {Sensor}, Inverted: {Inverted}, SensorFirstCornerChanged: {SensorFirstCornerChanged}")
        if direction == 0:
            Inverted = True
            if not SensorFirstCornerChanged:
                Sensor = 1
                Utils.usb_communication.sendMessage("S1", ESPHoldDistance)
                SensorFirstCornerChanged = True
            
            relative_angle = Utils.Gyro.angle - oldAngle
            print(relative_angle)
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
                
        elif direction == 1:
            Inverted = False
            if not SensorFirstCornerChanged:
                Sensor = 2
                Utils.usb_communication.sendMessage("S2", ESPHoldDistance)
                SensorFirstCornerChanged = True
            
            relative_angle = Utils.Gyro.angle - oldAngle
            newAngle = oldAngle + GyroCornerAngle
            if Utils.Gyro.angle > newAngle and time.time() > TIMEOUT:
                corners = corners + 1
                timelastcorner = time.time()
                Utils.LogDebug(f"Corner: {corners}")
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1
                    
                oldAngle = newAngle
                Utils.Gyro.angle = Utils.Gyro.angle - 20
                TIMEOUT = time.time() + LineWaitTime
                
        #get objects and calculate new distance
        if detect_new_block:
            block_array = Utils.Cam.block_array.copy()
            if len(block_array) > 0:
                #Delete blocks not meeting requirements
                for block in block_array:
                    if block['y'] > YCutOffTop and block["size"] > SIZE:
                        pass
                    else:
                        block_array.remove(block)
                    
            #print(block_array)
            if len(block_array) > 0 or active_block_drive:
                #Utils.LogDebug(f"Lenght of block_array: {len(block_array)}")
                #Sort blocks by size
                if not active_block_drive:
                    block_array.sort(key=lambda x: x['size'], reverse=True)

                    nextBlock = block_array[0]
                
                #print(Utils.Cam.avg_edge_distance)
                if (Utils.Cam.avg_edge_distance < 200) and not active_block_drive:
                    active_block_drive = get_block_pos(nextBlock, direction, corners, relative_angle)
                            
                if active_block_drive:     
                    print(Utils.blockPositions)       
                    active_block_drive, ESP_adjusted = drive_special_case(corners, active_block_drive, ESP_adjusted)
                    
                elif (nextBlock['w'] > FreezeSize or nextBlock['y'] > FreezeY or old_desired_distance_wall < 15 or old_desired_distance_wall > 75) and not FreezeBlock:
                    detect_new_block = False
                    Utils.Cam.freeze = True
                    Time_detect_new_block = time.time()
                    Utils.LogInfo(f"Freeze")
                    
                    Utils.usb_communication.sendMessage("MANUAL", ESPHoldDistance)
                    Utils.usb_communication.sendMessage("ANG90", ESPHoldDistance)

                else:
                    block_correction(nextBlock)
                        
            else:
                if time.time() > Last_Esp_Command + 1:  
                   # time.sleep(0.1)
                    #Utils.LogInfo(f"Desired Distance: {desired_distance_wall}, Current Sensor: {Sensor}")
                    if Inverted:
                        if Sensor != 1:
                            Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                            Sensor = 1
                    else:
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
                        
                    Utils.Cam.desired_distance_wall = desired_distance_wall
                    if desired_distance_wall <= 30:
                        desired_distance_wall = 31
                    elif desired_distance_wall >= 70:
                        desired_distance_wall = 69
                    old_desired_distance_wall = desired_distance_wall
                        
        else:
            if time.time() > Time_detect_new_block + TIMEOUTBlock:
                Utils.usb_communication.sendMessage("MANUAL", ESPHoldDistance)
                
                time.sleep(0.5)
                detect_new_block = True
                
                
                Utils.Cam.freeze = False

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
        
        StopTime = time.time()
        #Utils.LogDebug(f"Time: {StopTime - StartTime}, block_array: {Utils.Cam.block_array}")
        
        
# Get block position and save it in dictionary  
def get_block_pos(block, direction, corners, relative_angle=0):
    block_distance_to_wall = Utils.Cam.avg_edge_distance - block['distance']
    #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
    if direction == 0:
        if (90 < Utils.Cam.avg_edge_distance < 180) and block['x'] > 800 and block['y'] > 200:
            block['position'] = "1"
            BlockPos = corners + 1 if corners < 3 else 0
        elif 80 < block_distance_to_wall < 190:
            block['position'] = "3"
            BlockPos = corners
        elif 190 < block_distance_to_wall < 240:# or block_distance_to_wall < 80:
            block['position'] = "2"
            if abs(relative_angle) > 40:
                BlockPos = corners + 1 if corners < 3 else 0
            else:
                BlockPos = corners
        else:
            block['position'] = "0"
        
        if block['position'] != "0":# and BlockPos not in Utils.blockPositions:
            active_block_drive = True
            Utils.blockPositions.update({BlockPos: {"position": block['position'], "color": block['color']}})
    
    elif direction == 1:
        if (90 < Utils.Cam.avg_edge_distance < 200) and block['x'] < 250 and block['y'] > 200:
            block['position'] = "1"
            BlockPos = corners + 1 if corners < 3 else 0
        elif 80 < block_distance_to_wall < 190:
            block['position'] = "3"
            BlockPos = corners
        elif 190 < block_distance_to_wall < 240: #or block_distance_to_wall < 80:
            block['position'] = "2"
            if abs(relative_angle) > 30:
                BlockPos = corners + 1 if corners < 3 else 0
            else:
                BlockPos = corners

        else:
            block['position'] = "0"
        
        if block['position'] != "0":
            active_block_drive = True
            Utils.blockPositions.update({BlockPos: {"position": block['position'], "color": block['color']}})
            
    return active_block_drive


# Correct to drive around specified block
def block_correction(block):
    block['distancex'] = -(coordinates_self[0] - block['mx'])
    block['distancey'] = coordinates_self[1] - block['y']
    
    if block['color'] == "red":
        desired_distance_to_block = desired_distance_to_block_red
    elif block['color'] == "green":
        desired_distance_to_block = desired_distance_to_block_green
        
    distance_divider = (block['y'] / coordinates_self[1]) * DISTANCE_DIVIDER_VALUE
    
    # Calculation of desired distance to wall
    error = (desired_distance_to_block - block['distancex']) * distance_divider
    desired_distance_wall =  middledistance - error * KP
    desired_distance_wall = int(desired_distance_wall)
    Utils.Cam.desired_distance_wall = desired_distance_wall
    
    if desired_distance_wall < 15:
        desired_distance_wall =  15 
    elif desired_distance_wall > 85:
        desired_distance_wall = 85

    if desired_distance_wall > middledistance:
        desired_distance_wall_other_direction = (100 - desired_distance_wall)
        
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
            
            
# check for special case and adjust for it      
def drive_special_case(corners, active_block_drive, ESP_adjusted):
    next_corners = corners + 1 if corners < 3 else 0
    if next_corners in Utils.blockPositions:
        if Utils.blockPositions[next_corners]["position"] != "3" and Utils.blockPositions[next_corners]["color"] == "red":
            if not ESP_adjusted:
                Utils.LogInfo(f"Position 1 Red")
                Utils.usb_communication.sendMessage("S1", ESPHoldDistance)
                Utils.usb_communication.sendMessage("D35", ESPHoldDistance) 
                ESP_adjusted = True
            
            if Utils.Cam.avg_edge_distance < 90:
                Utils.LogWarning(f"Block too close to wall")
                Utils.usb_communication.sendMessage("S2", ESPHoldDistance)
                Utils.usb_communication.sendMessage("D50", ESPHoldDistance)
                active_block_drive = False
                ESP_adjusted = False
                
    return active_block_drive, ESP_adjusted
    

# Main function
if __name__ == "__main__":
    try: 
        #start flask server if needed
        if Utils.Cam.video_stream:
            app = Flask(__name__)
            
            @app.route('/')
            def index():
                return render_template('index.html')


            @app.route('/video_feed')
            def video_feed():
                return Response(Utils.Cam.video_frames(),
                                mimetype='multipart/x-mixed-replace; boundary=frame')
                
            @app.route('/data_feed')
            def data_feed():
                return jsonify(Utils.data_feed())

            # Run the server in a separate thread
            server_thread = Thread(target=app.run, kwargs={'host':'0.0.0.0', 'threaded':True})
            server_thread.start()
            
            @app.route('/positions')
            def positions():
                return render_template('positions.html')
            
            @app.route('/positions_feed')
            def positions_feed():
                return jsonify(Utils.blockPositions)
                
        Utils.StartRun()
        HoldLane(Utils)
    
    except:
        Utils.LogError(traceback.format_exc())
        
    finally:
        Utils.StopRun()