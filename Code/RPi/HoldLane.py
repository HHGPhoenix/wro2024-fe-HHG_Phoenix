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
Utils.StartSpeed = 60
Utils.Distance = 50
Utils.Kp = 0.7
Utils.Ed = 125 #Edge detection distance in cm
Utils.StartSensor = 2
Utils.Mm = 10
Utils.AngR = 40
Utils.AngL = 50


##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
def HoldLane(Utils, CornerWaiTTime=1):
    #Variables
    TIMEOUT = 0
    corners = 0
    rounds = 0
    direction = -1
    oldAngle = 0
    GyroCornerAngle = 70
    Sensor = 0
    relative_angle = 0
    timelastcorner = 0
    
    #Cutoffs
    YCutOffTop = 0
    SIZE = 0
    
    ESPAdjustedCorner = False
    ESPAdjusted = False
    detect_new_block = True
    block_wide_corner = False
    
    #Hold Lane
    while Utils.running and rounds < 3:
        time.sleep(0.001)
                
        if direction == 0:
            if Sensor != 1:
                Sensor = 1
                Utils.LogInfo("Switched to Sensor 1")
                Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
               
            relative_angle = Utils.Gyro.angle - oldAngle 
            newAngle = oldAngle - GyroCornerAngle
            if Utils.Gyro.angle < newAngle and time.time() > TIMEOUT:
                corners = corners + 1
                Utils.LogDebug(f"Corner: {corners}")
                Utils.Display.write(f"Corner: {corners}")
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1
                    Utils.Display.write(f"Corner: {corners}", f"Round: {rounds}")
                    
                oldAngle = newAngle
                Utils.Gyro.angle = Utils.Gyro.angle + 20
                TIMEOUT = time.time() + CornerWaiTTime
                
        elif direction == 1:
            if Sensor != 2:
                Sensor = 2
                Utils.LogInfo("Switched to Sensor 2")
                Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
            
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
                TIMEOUT = time.time() + CornerWaiTTime
                
        # Drive wide around corners
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
            if len(block_array) > 0:
                #Utils.LogDebug(f"Lenght of block_array: {len(block_array)}")
                #Sort blocks by size
                block_array.sort(key=lambda x: x['size'], reverse=True)

                nextBlock = block_array[0]
                    
                nextBlock["distance"] = Utils.Cam.get_distance_to_block(nextBlock)
                
                #print(nextBlock["distance"])
                block_distance_to_wall = Utils.Cam.avg_edge_distance - nextBlock['distance']
                Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                
                
                if (Utils.Cam.avg_edge_distance < 200) and -10 < relative_angle < 40 and direction == 1:
                    #block_distance_to_wall = Utils.Cam.avg_edge_distance - nextBlock['distance']
                    #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                    if (120 < Utils.Cam.avg_edge_distance < 150) and nextBlock['x'] < 300 and 50 < nextBlock["distance"] < 110:
                        nextBlock['position'] = "1"
                        BlockPos = corners + 1 if corners < 3 else 0
                        
                        
                    # elif 80 < block_distance_to_wall < 130 and nextBlock["distance"] < 80:
                    #     nextBlock['position'] = "3"
                    #     #print("3")
                    #     BlockPos = corners
                    # elif 30 < block_distance_to_wall < 60 and nextBlock["distance"] < 80: #or block_distance_to_wall < 80:
                    #     nextBlock['position'] = "2"
                    #     if abs(relative_angle) > 30:
                    #         BlockPos = corners + 1 if corners < 3 else 0
                    #     else:
                    #         BlockPos = corners

                    else:
                        nextBlock['position'] = "0"
                    
                    if nextBlock['position'] != "0" and nextBlock["position"] != "3":
                        Utils.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                    
                    elif nextBlock['position'] != "0" and nextBlock["position"] == "3":# and BlockPos not in Utils.blockPositions:
                        Utils.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                            
                elif (Utils.Cam.avg_edge_distance < 200) and - 40 < relative_angle < 10 and direction == 0:
                    #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {nextBlock['distance']}, block_distance_to_wall: {block_distance_to_wall}, nextblock['x']: {nextBlock['x']}, nextblock['y']: {nextBlock['y']}")
                    if (120 < Utils.Cam.avg_edge_distance < 180) and nextBlock['x'] > 780 and nextBlock['y'] > 200 and 70 < nextBlock["distance"] < 110:
                        nextBlock['position'] = "1"
                        BlockPos = corners + 1 if corners < 3 else 0
                    elif 80 < block_distance_to_wall < 130 and nextBlock["distance"] < 80:
                        nextBlock['position'] = "3"
                        BlockPos = corners
                    elif 130 < block_distance_to_wall < 180 and nextBlock["distance"] < 80:# or block_distance_to_wall < 80:
                        nextBlock['position'] = "2"
                        if abs(relative_angle) > 40:
                            BlockPos = corners + 1 if corners < 3 else 0
                        else:
                            BlockPos = corners
                    else:
                        nextBlock['position'] = "0"
                    
                    if nextBlock['position'] != "0" and nextBlock["position"] != "3":
                        active_block_drive = True
                        Utils.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
                    
                    elif nextBlock['position'] != "0" and nextBlock["position"] == "3":# and BlockPos not in Utils.blockPositions:
                        Utils.blockPositions.update({BlockPos: {"position": nextBlock['position'], "color": nextBlock['color']}})
        
        next_corners = corners + 1 if corners < 3 else 0
            
        if corners in Utils.blockPositions:
            if direction == 0:
                pass
            elif direction == 1:
                if Utils.blockPositions[corners]["color"] == "red":
                    if not ESPAdjustedCorner:
                        Utils.usb_communication.sendMessage("D 30", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        ESPAdjustedCorner = True
                        block_wide_corner = False
        
        if next_corners in Utils.blockPositions:
            if direction == 0:
                pass
            elif direction == 1:
                if Utils.blockPositions[next_corners]["position"] == "1" and Utils.blockPositions[next_corners]["color"] == "red":
                    if not ESPAdjusted and 65 < Utils.Cam.avg_edge_distance < 150:
                        Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                        ESPAdjusted = True
                        block_wide_corner = True
                        
                    elif ESPAdjusted and Utils.Cam.avg_edge_distance < 65:
                        Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                        Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                        ESPAdjusted = False
                        block_wide_corner = False
                
        # print(Utils.Cam.avg_edge_distance)
        if 100 < Utils.Cam.avg_edge_distance < 150 and abs(relative_angle) < 15 and timelastcorner + 1 < time.time() and not block_wide_corner:
            print("Drive wide around corner", Utils.Cam.avg_edge_distance)
            if not ESPAdjustedCorner:
                if direction == 0:
                    Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                    Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                    ESPAdjustedCorner = True
                elif direction == 1:
                    Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                    Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                    ESPAdjustedCorner = True
                    
        elif Utils.Cam.avg_edge_distance < 100:
            if ESPAdjustedCorner:
                if direction == 0:
                    Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                    Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                    ESPAdjustedCorner = False
                elif direction == 1:
                    Utils.usb_communication.sendMessage("D 50", Utils.ESPHoldDistance)
                    Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                    ESPAdjustedCorner = False
                    
                    
        responses = Utils.usb_communication.getResponses(ESPHoldDistance)
        if (responses != None):
            # print(responses)
            for response in responses:
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
                return render_template('index.html')


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