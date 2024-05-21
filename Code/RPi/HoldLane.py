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



##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
class HoldLane():
    def __init__(self, Utils):
        self.Utils = Utils
        
        Utils.StartSpeed = 120
        Utils.Distance = 50
        Utils.Kp = 0.7
        Utils.Ed = 125 # Edge detection distance in cm
        Utils.StartSensor = 2
        Utils.Mm = 10
        Utils.AngR = 43
        Utils.AngL = 46
        
        #Variables
        self.TIMEOUT = 0
        self.corner = 0
        self.rounds = 0
        self.direction = -1
        self.oldAngle = 0
        self.GyroCornerAngle = 70
        self.Sensor = 0
        self.relative_angle = 0
        self.desired_distance_wall = 50
        self.smooth_to_middle = False
        self.smoothing_counter = 0
        self.CornerWaitTime = 1
        self.nextBlock = None
        self.block_distance_to_wall = 0
        
        self.timelastcorner = 0
        self.timelastgreenpos1 = 0
        self.timelastredpos1 = 0
        
        #Cutoffs
        self.YCutOffTop = 0
        self.SIZE = 0
        
        self.ESPAdjustedCorner = False
        self.ESPAdjusted = False
        self.detect_new_block = True
        self.block_wide_corner = False
        self.drive_corner = False
        
        self.coordinates_self = (640, 720) #x, y
        self.middledistance = 50
        self.KP = 0.4
        self.desired_distance_to_block_red = -700
        self.desired_distance_to_block_green = 700
        self.old_desired_distance_wall = 50
        
    def avoidBlocks(self):
        #Hold Lane
        while self.Utils.running and self.rounds < 3:
            time.sleep(0.0001)
            self.cornerStuff()
                    
            # get objects and calculate new distance
            if self.detect_new_block:
                block_array = self.Utils.Cam.block_array.copy()
                for block in block_array:
                    if block['y'] > self.YCutOffTop and block["size"] > self.SIZE:
                        pass
                    else:
                        block_array.remove(block)
                    
                self.next_corner = self.corner + 1 if self.corner < 3 else 0   
                #print(block_array)
                if len(block_array) > 0:
                    #Sort blocks by size
                    block_array.sort(key=lambda x: x['size'], reverse=True)

                    self.nextBlock = block_array[0]
                        
                    self.nextBlock["distance"] = self.Utils.Cam.get_distance_to_block(self.nextBlock)
                    
                    #print(self.nextBlock["distance"])
                    self.block_distance_to_wall = self.Utils.Cam.avg_edge_distance - self.nextBlock['distance']
                    self.Utils.LogDebug(f"avg_edge_distance: {self.Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
                    
                    self.detectBlockPos()
                               
                    if 55 < self.nextBlock["distance"] < 80 and not self.ESPAdjusted and not self.ESPAdjustedCorner and self.timelastcorner + 5 > time.time() and not self.timelastcorner + 1 < time.time():
                        self.smartSteer()
                
            if self.corner in self.Utils.blockPositions:
                self.currentCornerCases()
            
            if self.next_corner in Utils.blockPositions:
                self.nextCornerCases()
            
            # Drive to the middle of the lane
            if smooth_to_middle:
                if smoothing_counter >= 10:
                    if self.desired_distance_wall > 52:
                        self.desired_distance_wall = self.desired_distance_wall - 4
                        
                    elif self.desired_distance_wall < 48:
                        self.desired_distance_wall = self.desired_distance_wall + 4
                        
                    else:
                        self.desired_distance_wall = 50
                        smooth_to_middle = False
                        
                    smoothing_counter = 0
                    self.Utils.usb_communication.sendMessage(f"D {self.desired_distance_wall}", self.Utils.ESPHoldDistance)
                        
                else:
                    smoothing_counter += 1
                
            # Drive wide around corners
            self.driveWideCorner()
                                    
            responses = self.Utils.usb_communication.getResponses(ESPHoldDistance)
            if responses != None:
                # print(responses)
                for _ in responses:
                    if "Drive direction counterclockwise" in responses:
                        self.direction = 1
                    if "Drive direction clockwise" in responses:
                        self.direction = 0
            
            self.Utils.LogData()
           
            
    def smartSteer(self):
        self.nextBlock['distancex'] = -(self.coordinates_self[0] - self.nextBlock['mx'])
        self.nextBlock['distancey'] = self.coordinates_self[1] - self.nextBlock['y']
        
        if self.nextBlock['color'] == "red":
            desired_distance_to_block = self.desired_distance_to_block_red
        elif self.nextBlock['color'] == "green":
            desired_distance_to_block = self.desired_distance_to_block_green
            
        distance_divider = 15 / self.nextBlock['distance']
        
        # Calculation of desired distance to wall
        error = (desired_distance_to_block - self.nextBlock['distancex']) * distance_divider
        self.desired_distance_wall = self.middledistance - error * self.KP
        self.desired_distance_wall = int(self.desired_distance_wall)
        self.Utils.Cam.self.desired_distance_wall = self.desired_distance_wall
        
        if self.desired_distance_wall < 10:
            self.desired_distance_wall =  10 
        elif self.desired_distance_wall > 90:
            self.desired_distance_wall = 90

        if self.desired_distance_wall > self.middledistance:
            desired_distance_wall_other_direction = (100 - self.desired_distance_wall)
            
            if Sensor != 1:
                Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to Sensor 1")
                
            #Send ESPHoldDistance new Distance
            if abs(desired_distance_wall_other_direction - old_desired_distance_wall) > 3:
                self.Utils.usb_communication.sendMessage(f"D{desired_distance_wall_other_direction}", ESPHoldDistance)
                self.Utils.LogInfo(f"New Distance {desired_distance_wall_other_direction}, Current Sensor: {Sensor}")
                old_desired_distance_wall = desired_distance_wall_other_direction
            
        else:
            if Sensor != 2:
                Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to Sensor 2")
                
            if abs(self.desired_distance_wall - old_desired_distance_wall) > 3:
                self.Utils.usb_communication.sendMessage(f"D{self.desired_distance_wall}", ESPHoldDistance)
                self.Utils.LogInfo(f"New Distance {self.desired_distance_wall}, Current Sensor: {Sensor}")
                old_desired_distance_wall = self.desired_distance_wall
        
        
    def detectBlockPos(self):
        if (self.Utils.Cam.avg_edge_distance < 220) and -15 < self.relative_angle < 40 and self.direction == 1:
            #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
            self.nextBlock['position'] = "0"
            if (120 < self.Utils.Cam.avg_edge_distance < 150) and self.nextBlock['x'] < 300 and 50 < self.nextBlock["distance"] < 90 and self.timelastcorner + 1.5 < time.time(): # and next_corner not in Utils.blockPositions:
                self.nextBlock['position'] = "1"
                BlockPos = self.next_corner
                                        
            elif 85 < self.block_distance_to_wall < 110 and 45 < self.nextBlock["distance"] < 70 and self.timelastcorner + 1.5 < time.time(): # and corner not in Utils.blockPositions
                self.nextBlock['position'] = "3"
                BlockPos = self.corner
                
            # elif 45 < self.nextBlock['distance'] < 70 and self.timelastcorner + 1.5 < time.time():
            #     self.nextBlock['position'] = "2"
            #     BlockPos = corner
            
            if self.nextBlock['position'] != "0":
                self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                    
        elif (self.Utils.Cam.avg_edge_distance < 200) and -35 < self.relative_angle < 5 and self.direction == 0:
            #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
            if (120 < self.Utils.Cam.avg_edge_distance < 150) and self.nextBlock['x'] > 980 and 70 < self.nextBlock["distance"] < 110:
                self.nextBlock['position'] = "1"
                BlockPos = self.corner + 1 if self.corner < 3 else 0
                
            elif 85 < self.block_distance_to_wall < 110 and 45 < self.nextBlock["distance"] < 70 and self.timelastcorner + 1.5 < time.time() and self.corner not in self.Utils.blockPositions:
                self.nextBlock['position'] = "3"
                BlockPos = self.corner
                
            # elif corner not in Utils.blockPositions and 45 < self.nextBlock['distance'] < 70:
            #     self.nextBlock['position'] = "2"
            #     BlockPos = corner

            else:
                self.nextBlock['position'] = "0"
            
            if self.nextBlock['position'] != "0":
                self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                
                
    def currentCornerCases(self):
        if self.direction == 0:
            if self.Utils.blockPositions[self.corner]["color"] == "green" and self.timelastgreenpos1 + 1.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.LogInfo("green direction 0 start")
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 30
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    
            elif self.Utils.blockPositions[self.corner]["color"] == "red" and self.timelastredpos1 + 1.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 30
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    
        elif self.direction == 1:
            if self.Utils.blockPositions[self.corner]["color"] == "red" and self.timelastredpos1 + 1.5 < time.time() and self.timelastcorner + 1 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 30
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    
            elif self.Utils.blockPositions[self.corner]["color"] == "green" and self.timelastgreenpos1 + 1.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 30
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    
                    
    def nextCornerCases(self):
        if self.direction == 0:
            if self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "green":
                if not self.ESPAdjusted and 70 < self.Utils.Cam.avg_edge_distance < 150 and self.timelastgreenpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    self.Utils.LogInfo("Pos 1 green direction 0 start")
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 70:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    self.Utils.LogInfo("Pos 1 green direction 0 end")
                    self.timelastgreenpos1 = time.time()
                    
            elif self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "red":
                if not self.ESPAdjusted and 140 < self.Utils.Cam.avg_edge_distance < 180 and self.timelastredpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 70", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 70
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    self.Utils.LogInfo("Pos 1 red direction 0 start")
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 140:
                    self.Utils.usb_communication.sendMessage("D 10", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 10
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    self.Utils.LogInfo("Pos 1 red direction 0 end")
                    self.timelastredpos1 = time.time()
                    
        elif self.direction == 1:
            if self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "red":
                if not self.ESPAdjusted and 70 < self.Utils.Cam.avg_edge_distance < 150:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 70:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    
            elif self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "green":
                if not self.ESPAdjusted and 140 < self.Utils.Cam.avg_edge_distance < 180 and self.timelastgreenpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 70", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 70
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.timelastredpos1 = time.time()
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 140:
                    self.Utils.usb_communication.sendMessage("D 10", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 10
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    self.timelastgreenpos1 = time.time()
                    
                    
    def driveWideCorner(self):
        if self.direction == 1 and 100 < self.Utils.Cam.avg_edge_distance < 150 and -15 < self.relative_angle < 50 and self.timelastcorner + 2 < time.time() and not self.block_wide_corner:
            if not self.ESPAdjustedCorner:
                print("wide corner")
                self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                self.desired_distance_wall = 50
                self.ESPAdjustedCorner = True
                
        elif self.direction == 0 and 100 < self.Utils.Cam.avg_edge_distance < 150 and -35 < self.relative_angle < 15 and self.timelastcorner + 2.5 < time.time() and self.timelastredpos1 + 3 < time.time() and not self.block_wide_corner:
            if not self.ESPAdjustedCorner:
                print("wide corner start")
                self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                self.desired_distance_wall = 50
                self.ESPAdjustedCorner = True
                
        elif self.Utils.Cam.avg_edge_distance < 100 and self.timelastcorner + 3 < time.time():
            if self.ESPAdjustedCorner:
                if self.direction == 0:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.ESPAdjustedCorner = False
                    self.drive_corner = True
                    print("wide corner end ", self.Utils.Cam.avg_edge_distance)
                elif self.direction == 1:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.ESPAdjustedCorner = False
                    self.drive_corner = True
                    
                    
    def cornerStuff(self):
        if self.direction == 0:
            if Sensor != 1 and not self.ESPAdjusted and not self.ESPAdjustedCorner:
                Sensor = 1
                self.desired_distance_wall = 50
                self.Utils.LogInfo("Switched to Sensor 1")
                self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
            
            self.relative_angle = self.Utils.Gyro.angle - self.oldAngle 
            newAngle = self.oldAngle - self.GyroCornerAngle
            if self.Utils.Gyro.angle < newAngle and time.time() > TIMEOUT:
                self.corner = self.corner + 1
                self.timelastcorner = time.time()
                self.Utils.LogDebug(f"Corner: {self.corner}")
                self.Utils.Display.write(f"Corner: {self.corner}")
                if self.corner == 4:
                    self.corner = 0
                    self.rounds = self.rounds + 1
                    self.Utils.Display.write(f"Corner: {self.corner}", f"Round: {self.rounds}")
                    
                self.oldAngle = newAngle
                self.drive_corner = False
                self.Utils.Gyro.angle = self.Utils.Gyro.angle + 20
                TIMEOUT = time.time() + self.CornerWaitTime
                
        elif self.direction == 1:
            if Sensor != 2:
                Sensor = 2
                self.desired_distance_wall = 50
                self.Utils.LogInfo("Switched to Sensor 2")
                self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
            
            self.relative_angle = self.Utils.Gyro.angle - self.oldAngle
            newAngle = self.oldAngle + self.GyroCornerAngle
            if self.Utils.Gyro.angle > newAngle and time.time() > TIMEOUT:
                self.corner = self.corner + 1
                self.timelastcorner = time.time()
                self.Utils.LogDebug(f"Corner: {self.corner}")
                if self.corner == 4:
                    self.corner = 0
                    self.rounds = self.rounds + 1
        
                self.oldAngle = newAngle
                self.drive_corner = False
                self.Utils.Gyro.angle = self.Utils.Gyro.angle - 20
                TIMEOUT = time.time() + self.CornerWaitTime



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    Holdlane = HoldLane(Utils)
    
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
        Holdlane.avoidBlocks()
        
    except:
        Utils.LogError(traceback.format_exc())
        
    finally:
        Utils.StopRun()