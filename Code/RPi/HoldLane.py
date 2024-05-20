import time
from RobotCarClasses import *
from threading import Thread
import traceback
from flask import Flask, render_template, Response, jsonify

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

Cam = Camera(video_stream=True, Utils=Utils)

global ESPHoldDistance, ESPHoldSpeed
ESPHoldDistance, ESPHoldSpeed = Utils.transferSensorData(Farbsensor, StartButton, StopButton, Buzzer1, Cam)

Utils.setupDataLog()


##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
Utils.StartSpeed = 120
Utils.Distance = 50
Utils.Kp = 0.7
Utils.Ed = 125 # Edge detection distance in cm
Utils.StartSensor = 2
Utils.Mm = 10
Utils.AngR = 43
Utils.AngL = 46


##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, CornerWaitTime=1):
    #Variables
    TIMEOUT = 0
    corner = 0
    rounds = 0
    direction = -1
    oldAngle = 0
    GyroCornerAngle = 70
    Sensor = 0
    relative_angle = 0
    desired_distance_wall = 50
    smooth_to_middle = False
    smoothing_counter = 0
    
    timelastcorner = 0
    timelastgreenpos1 = 0
    timelastredpos1 = 0
    
    #Cutoffs
    YCutOffTop = 0
    SIZE = 0
    
    ESPAdjustedCorner = False
    ESPAdjusted = False
    detect_new_block = True
    block_wide_corner = False
    drive_corner = False
    
    coordinates_self = (640, 720) #x, y
    middledistance = 50
    KP = 0.4
    desired_distance_to_block_red = -700
    desired_distance_to_block_green = 700
    old_desired_distance_wall = 50
    
    #Hold Lane
    while Utils.running and rounds < 3:
        time.sleep(0.0001)
        if direction == 0:
            if Sensor != 1:
                Sensor = 1
                desired_distance_wall = 50
                Utils.LogInfo("Switched to Sensor 1")
                Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
               
            relative_angle = Utils.Gyro.angle - oldAngle 
            newAngle = oldAngle - GyroCornerAngle
            if Utils.Gyro.angle < newAngle and time.time() > TIMEOUT:
                corner = corner + 1
                timelastcorner = time.time()
                Utils.LogDebug(f"Corner: {corner}")
                Utils.Display.write(f"Corner: {corner}")
                if corner == 4:
                    corner = 0
                    rounds = rounds + 1
                    Utils.Display.write(f"Corner: {corner}", f"Round: {rounds}")
                    
                oldAngle = newAngle
                drive_corner = False
                Utils.Gyro.angle = Utils.Gyro.angle + 20
                TIMEOUT = time.time() + CornerWaitTime
                
        elif direction == 1:
            if Sensor != 2:
                Sensor = 2
                desired_distance_wall = 50
                Utils.LogInfo("Switched to Sensor 2")
                Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
            
            relative_angle = Utils.Gyro.angle - oldAngle
            newAngle = oldAngle + GyroCornerAngle
            if Utils.Gyro.angle > newAngle and time.time() > TIMEOUT:
                corner = corner + 1
                timelastcorner = time.time()
                Utils.LogDebug(f"Corner: {corner}")
                if corner == 4:
                    corner = 0
                    rounds = rounds + 1
        
                oldAngle = newAngle
                drive_corner = False
                Utils.Gyro.angle = Utils.Gyro.angle - 20
                TIMEOUT = time.time() + CornerWaitTime
                
        # get objects and calculate new distance
        if detect_new_block:
            block_array = Utils.Cam.block_array.copy()
            # if len(block_array) > 0:
                #Delete blocks not meeting requirements
            for block in block_array:
                if block['y'] > YCutOffTop and block["size"] > SIZE:
                    pass
                else:
                    block_array.remove(block)
                
            next_corner = corner + 1 if corner < 3 else 0   
            #print(block_array)
            if len(block_array) > 0:
                #Utils.LogDebug(f"Lenght of block_array: {len(block_array)}")
                #Sort blocks by size
                block_array.sort(key=lambda x: x['size'], reverse=True)

                nextBlock = block_array[0]
                    
                nextBlock["distance"] = Utils.Cam.get_distance_to_block(nextBlock)
                
                #print(nextBlock["distance"])
                block_distance_to_wall = Utils.Cam.avg_edge_distance - nextBlock['distance']
                Utils.LogDebug(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                
                if (Utils.Cam.avg_edge_distance < 220) and -15 < relative_angle < 40 and direction == 1:
                    #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                    nextBlock['position'] = "0"
                    if (120 < Utils.Cam.avg_edge_distance < 150) and nextBlock['x'] < 300 and 50 < nextBlock["distance"] < 90 and timelastcorner + 1.5 < time.time(): # and next_corner not in Utils.blockPositions:
                        nextBlock['position'] = "1"
                        BlockPos = next_corner
                               
                        # elif 60 < nextBlock['distance'] < 80 and timelastcorner + 1.5 < time.time():
                        #     nextBlock['position'] = "2"
                        #     BlockPos = next_corner
                                                
                    elif 85 < block_distance_to_wall < 110 and 45 < nextBlock["distance"] < 70 and timelastcorner + 1.5 < time.time(): # and corner not in Utils.blockPositions
                        nextBlock['position'] = "3"
                        BlockPos = corner
                        
                    # elif 45 < nextBlock['distance'] < 70 and timelastcorner + 1.5 < time.time():
                    #     nextBlock['position'] = "2"
                    #     BlockPos = corner
                    
                    if nextBlock['position'] != "0":
                        Utils.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                            
                elif (Utils.Cam.avg_edge_distance < 200) and -35 < relative_angle < 5 and direction == 0:
                    #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                    if (120 < Utils.Cam.avg_edge_distance < 150) and nextBlock['x'] > 980 and 70 < nextBlock["distance"] < 110:
                        nextBlock['position'] = "1"
                        BlockPos = corner + 1 if corner < 3 else 0
                        
                    elif 85 < block_distance_to_wall < 110 and 45 < nextBlock["distance"] < 70 and timelastcorner + 1.5 < time.time() and corner not in Utils.blockPositions:
                        nextBlock['position'] = "3"
                        BlockPos = corner
                        
                    # elif corner not in Utils.blockPositions and 45 < nextBlock['distance'] < 70:
                    #     nextBlock['position'] = "2"
                    #     BlockPos = corner

                    else:
                        nextBlock['position'] = "0"
                    
                    if nextBlock['position'] != "0":
                        Utils.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                      
                        
                if 55 < nextBlock["distance"] < 80 and not ESPAdjusted and not ESPAdjustedCorner and timelastcorner + 5 > time.time() and not timelastcorner + 1 < time.time():
                    nextBlock['distancex'] = -(coordinates_self[0] - nextBlock['mx'])
                    nextBlock['distancey'] = coordinates_self[1] - nextBlock['y']
                    
                    if nextBlock['color'] == "red":
                        desired_distance_to_block = desired_distance_to_block_red
                    elif nextBlock['color'] == "green":
                        desired_distance_to_block = desired_distance_to_block_green
                        
                    distance_divider = 15 / nextBlock['distance']
                    
                    # Calculation of desired distance to wall
                    error = (desired_distance_to_block - nextBlock['distancex']) * distance_divider
                    desired_distance_wall =  middledistance - error * KP
                    desired_distance_wall = int(desired_distance_wall)
                    Utils.Cam.desired_distance_wall = desired_distance_wall
                    
                    if desired_distance_wall < 10:
                        desired_distance_wall =  10 
                    elif desired_distance_wall > 90:
                        desired_distance_wall = 90

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
            
        if corner in Utils.blockPositions:
            if direction == 0:
                if Utils.blockPositions[corner]["color"] == "green" and timelastgreenpos1 + 1.5 < time.time() and not drive_corner:
                    if not ESPAdjustedCorner:
                        Utils.LogInfo("green direction 0 start")
                        Utils.usb_communication.sendMessage("D 15", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                        desired_distance_wall = 30
                        ESPAdjustedCorner = True
                        block_wide_corner = False
                        
                elif Utils.blockPositions[corner]["color"] == "red" and timelastredpos1 + 1.5 < time.time() and not drive_corner:
                    if not ESPAdjustedCorner:
                        Utils.usb_communication.sendMessage("D 15", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 30
                        ESPAdjustedCorner = True
                        block_wide_corner = False
                        
            elif direction == 1:
                if Utils.blockPositions[corner]["color"] == "red" and timelastredpos1 + 1.5 < time.time() and timelastcorner + 1 < time.time() and not drive_corner:
                    if not ESPAdjustedCorner:
                        Utils.usb_communication.sendMessage("D 15", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 30
                        ESPAdjustedCorner = True
                        block_wide_corner = False
                        
                elif Utils.blockPositions[corner]["color"] == "green" and timelastgreenpos1 + 1.5 < time.time() and not drive_corner:
                    if not ESPAdjustedCorner:
                        Utils.usb_communication.sendMessage("D 15", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                        desired_distance_wall = 30
                        ESPAdjustedCorner = True
                        block_wide_corner = False
        
        if next_corner in Utils.blockPositions:
            if direction == 0:
                if Utils.blockPositions[next_corner]["position"] == "1" and Utils.blockPositions[next_corner]["color"] == "green":
                    if not ESPAdjusted and 70 < Utils.Cam.avg_edge_distance < 150 and timelastgreenpos1 + 3 < time.time():
                        Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                        desired_distance_wall = 50
                        ESPAdjusted = True
                        block_wide_corner = True
                        ESPAdjustedCorner = False
                        Utils.LogInfo("Pos 1 green direction 0 start")
                        
                    elif ESPAdjusted and Utils.Cam.avg_edge_distance < 70:
                        Utils.usb_communication.sendMessage("D 15", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 15
                        ESPAdjusted = False
                        block_wide_corner = False
                        Utils.LogInfo("Pos 1 green direction 0 end")
                        timelastgreenpos1 = time.time()
                        
                elif Utils.blockPositions[next_corner]["position"] == "1" and Utils.blockPositions[next_corner]["color"] == "red":
                    if not ESPAdjusted and 140 < Utils.Cam.avg_edge_distance < 180 and timelastredpos1 + 3 < time.time():
                        Utils.usb_communication.sendMessage("D 70", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 70
                        ESPAdjusted = True
                        block_wide_corner = True
                        ESPAdjustedCorner = False
                        Utils.LogInfo("Pos 1 red direction 0 start")
                        
                    elif ESPAdjusted and Utils.Cam.avg_edge_distance < 140:
                        Utils.usb_communication.sendMessage("D 10", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 10
                        ESPAdjusted = False
                        block_wide_corner = False
                        Utils.LogInfo("Pos 1 red direction 0 end")
                        timelastredpos1 = time.time()
                        
            elif direction == 1:
                if Utils.blockPositions[next_corner]["position"] == "1" and Utils.blockPositions[next_corner]["color"] == "red":
                    if not ESPAdjusted and 70 < Utils.Cam.avg_edge_distance < 150:
                        Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 50
                        ESPAdjusted = True
                        block_wide_corner = True
                        
                    elif ESPAdjusted and Utils.Cam.avg_edge_distance < 70:
                        Utils.usb_communication.sendMessage("D 15", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                        desired_distance_wall = 15
                        ESPAdjusted = False
                        block_wide_corner = False
                        
                elif Utils.blockPositions[next_corner]["position"] == "1" and Utils.blockPositions[next_corner]["color"] == "green":
                    if not ESPAdjusted and 140 < Utils.Cam.avg_edge_distance < 180 and timelastgreenpos1 + 3 < time.time():
                        Utils.usb_communication.sendMessage("D 70", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                        desired_distance_wall = 70
                        ESPAdjusted = True
                        block_wide_corner = True
                        timelastredpos1 = time.time()
                        
                    elif ESPAdjusted and Utils.Cam.avg_edge_distance < 140:
                        Utils.usb_communication.sendMessage("D 10", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        desired_distance_wall = 10
                        ESPAdjusted = False
                        block_wide_corner = False
                        timelastgreenpos1 = time.time()
        
        # Drive to the middle of the lane
        if smooth_to_middle:
            if smoothing_counter >= 10:
                if desired_distance_wall > 52:
                    desired_distance_wall = desired_distance_wall - 4
                    
                elif desired_distance_wall < 48:
                    desired_distance_wall = desired_distance_wall + 4
                    
                else:
                    desired_distance_wall = 50
                    smooth_to_middle = False
                    
                smoothing_counter = 0
                Utils.usb_communication.sendMessage(f"D {desired_distance_wall}", Utils.ESPHoldDistance)
                    
            else:
                smoothing_counter += 1
            
        
        # Drive wide around corners
        # print(Utils.Cam.avg_edge_distance)
        if direction == 1 and 100 < Utils.Cam.avg_edge_distance < 150 and -15 < relative_angle < 50 and timelastcorner + 2 < time.time() and not block_wide_corner:
            if not ESPAdjustedCorner:
                print("wide corner")
                Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                desired_distance_wall = 50
                ESPAdjustedCorner = True
                
        elif direction == 0 and 100 < Utils.Cam.avg_edge_distance < 150 and -35 < relative_angle < 15 and timelastcorner + 2.5 < time.time() and timelastredpos1 + 3 < time.time() and not block_wide_corner:
            if not ESPAdjustedCorner:
                print("wide corner start")
                Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                desired_distance_wall = 50
                ESPAdjustedCorner = True
                
        elif Utils.Cam.avg_edge_distance < 100 and timelastcorner + 3 < time.time():
            if ESPAdjustedCorner:
                if direction == 0:
                    Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                    Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                    desired_distance_wall = 50
                    ESPAdjustedCorner = False
                    drive_corner = True
                    print("wide corner end ", Utils.Cam.avg_edge_distance)
                elif direction == 1:
                    Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                    Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                    desired_distance_wall = 50
                    ESPAdjustedCorner = False
                    drive_corner = True
                                  
        responses = Utils.usb_communication.getResponses(ESPHoldDistance)
        if responses != None:
            # print(responses)
            for _ in responses:
                if "Drive direction counterclockwise" in responses:
                    direction = 1
                if "Drive direction clockwise" in responses:
                    direction = 0
        
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
                return render_template('videofeed.html')


            @app.route('/video_feed')
            def video_feed():
                return Response(Cam.video_frames(),
                                mimetype='multipart/x-mixed-replace; boundary=frame')
                
            @app.route('/data_feed')
            def data_feed():
                return jsonify(Utils.data_feed())
            
            @app.route('/positions')
            def positions():
                return render_template('positions.html')
            
            @app.route('/positions_feed')
            def positions_feed():
                return jsonify(Utils.blockPositions)

            # Run the server in a separate thread
            server_thread = Thread(target=app.run, kwargs={'host':'0.0.0.0', 'threaded':True})
            server_thread.start()
                
        Utils.StartRun()
        HoldLane(Utils)
    
    except:
        Utils.LogError(traceback.format_exc())
        
    finally:
        Utils.StopRun()