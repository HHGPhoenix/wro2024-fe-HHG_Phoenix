import time
from RobotCarClasses import *
from threading import Thread
import traceback
from flask import Flask, render_template, Response, jsonify

##########################################################
##                                                      ##
##        self.Sensor / Class Initalization             ##
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
Utils.StartSpeed = 115
Utils.Distance = 50
Utils.Kp = 3
Utils.Kd = 0.7
Utils.Ed = 125 # Edge detection distance in cm
Utils.StartSensor = 2
Utils.Mm = 0.5
Utils.AngR = 43
Utils.AngL = 46

##########################################################
##                                                      ##
##                     Functions                        ##
##                                                      ##
##########################################################
class HoldLane():
    def __init__(self, Utils):
        self.Utils = Utils
        
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
        self.sensorAdjustedCorner = False
        self.active_block_drive = False
        
        self.coordinates_self = (640, 720) #x, y
        self.middledistance = 50
        self.KP = 0.3
        self.desired_distance_to_block_red = -650
        self.desired_distance_to_block_green = 650
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
                    self.nextBlock2 = block_array[1] if len(block_array) > 1 else None
                        
                    self.nextBlock["distance"] = self.Utils.Cam.get_distance_to_block(self.nextBlock)
                    
                    #print(self.nextBlock["distance"])
                    self.block_distance_to_wall = self.Utils.Cam.avg_edge_distance - self.nextBlock['distance']
                    # self.Utils.LogDebug(f"avg_edge_distance: {self.Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
                    
                    self.detectBlockPos()
                               
                    if 60 < self.nextBlock["distance"] < 100 and not self.ESPAdjusted and not self.ESPAdjustedCorner and self.corner not in self.Utils.blockPositions:
                        self.smartSteer()
                
            if not self.ESPAdjusted and not self.ESPAdjustedCorner:
                self.avoid_sharp_angle()
                
            if self.corner in self.Utils.blockPositions:
                self.currentCornerCases()
            
            if self.next_corner in Utils.blockPositions:
                self.nextCornerCases()
            
            # Drive to the middle of the lane
            if self.smooth_to_middle:
                if smoothing_counter >= 10:
                    if self.desired_distance_wall > 52:
                        self.desired_distance_wall = self.desired_distance_wall - 4
                        
                    elif self.desired_distance_wall < 48:
                        self.desired_distance_wall = self.desired_distance_wall + 4
                        
                    else:
                        self.desired_distance_wall = 50
                        self.smooth_to_middle = False
                        
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
            # self.Utils.LogDebug(f"self.ESPAdjusted: {self.ESPAdjusted}, self.ESPAdjustedCorner: {self.ESPAdjustedCorner}, self.desired_distance_wall: {self.desired_distance_wall}, relative_angle: {self.relative_angle}, direction: {self.direction}")
           
            
    def smartSteer(self):
        self.nextBlock['distancex'] = -(self.coordinates_self[0] - self.nextBlock['mx'])
        self.nextBlock['distancey'] = self.coordinates_self[1] - self.nextBlock['y']
        
        if self.nextBlock['color'] == "red":
            desired_distance_to_block = self.desired_distance_to_block_red
        elif self.nextBlock['color'] == "green":
            desired_distance_to_block = self.desired_distance_to_block_green
            
        distance_divider = 20 / self.nextBlock['distance']
        
        # Calculation of desired distance to wall
        error = (desired_distance_to_block - self.nextBlock['distancex']) * distance_divider
        self.desired_distance_wall = self.middledistance - error * self.KP
        self.desired_distance_wall = int(self.desired_distance_wall)
        self.Utils.Cam.desired_distance_wall = self.desired_distance_wall
        
        if self.desired_distance_wall < 15:
            self.desired_distance_wall =  15 
        elif self.desired_distance_wall > 85:
            self.desired_distance_wall = 85
            

        # self.Utils.LogDebug(f"self.ESPAdjusted: {self.ESPAdjusted}, self.ESPAdjustedCorner: {self.ESPAdjustedCorner}, self.desired_distance_wall: {self.desired_distance_wall}, relative_angle: {self.relative_angle}, direction: {self.direction}")
        if self.desired_distance_wall > self.middledistance:
            if self.direction == 1 and not (-20 < self.relative_angle < 40):
                # self.Utils.LogInfo(f"Special Case: {self.relative_angle}, {self.direction}")
                # if self.Sensor != 2:
                #     self.Sensor = 2
                #     self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                #     self.Utils.LogInfo(f"Switched to self.Sensor 2 Special")
                    
                # if abs(self.desired_distance_wall - self.old_desired_distance_wall) > 3:
                #     self.Utils.usb_communication.sendMessage(f"D{self.desired_distance_wall}", ESPHoldDistance)
                #     self.Utils.LogInfo(f"New Distance {self.desired_distance_wall}, Current self.Sensor: {self.Sensor}")
                #     self.old_desired_distance_wall = self.desired_distance_wall
                return
                
            desired_distance_wall_other_direction = (100 - self.desired_distance_wall)
            
            if self.Sensor != 1:
                self.Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 1 Smart Steer")
                
            #Send ESPHoldDistance new Distance
            if abs(desired_distance_wall_other_direction - self.old_desired_distance_wall) > 3:
                self.Utils.usb_communication.sendMessage(f"D{desired_distance_wall_other_direction}", ESPHoldDistance)
                self.Utils.LogInfo(f"New Distance {desired_distance_wall_other_direction}, Current self.Sensor: {self.Sensor}")
                self.old_desired_distance_wall = desired_distance_wall_other_direction
            
        else:
            if self.direction == 0 and not (10 > self.relative_angle > -50):
                self.Utils.LogInfo(f"Special Case: {self.relative_angle}, {self.direction}")
                if self.Sensor != 1:
                    self.Sensor = 1
                    self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                    self.Utils.LogInfo(f"Switched to self.Sensor 1 Special")
                    
                if abs(self.desired_distance_wall - self.old_desired_distance_wall) > 3:
                    self.Utils.usb_communication.sendMessage(f"D{self.desired_distance_wall}", ESPHoldDistance)
                    self.Utils.LogInfo(f"New Distance {self.desired_distance_wall}, Current self.Sensor: {self.Sensor}")
                    self.old_desired_distance_wall = self.desired_distance_wall
                return
            
            elif self.direction == 1 and self.Utils.Cam.avg_edge_distance < 200:
                self.Utils.LogInfo(f"Special Case: {self.relative_angle}, {self.direction}")
                if self.Sensor != 2:
                    self.Sensor = 2
                    self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                    self.Utils.LogInfo(f"Switched to self.Sensor 2 Special")
                    
                desired_distance_wall_other_direction = (100 - self.desired_distance_wall)
                    
                if abs(desired_distance_wall_other_direction - self.old_desired_distance_wall) > 3:
                    self.Utils.usb_communication.sendMessage(f"D{desired_distance_wall_other_direction}", ESPHoldDistance)
                    self.Utils.LogInfo(f"New Distance {desired_distance_wall_other_direction}, Current self.Sensor: {self.Sensor}")
                    self.old_desired_distance_wall = desired_distance_wall_other_direction
                return
            
            if self.Sensor != 2:
                self.Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 2 Smart Steer")
                
            if abs(self.desired_distance_wall - self.old_desired_distance_wall) > 3:
                self.Utils.usb_communication.sendMessage(f"D{self.desired_distance_wall}", ESPHoldDistance)
                self.Utils.LogInfo(f"New Distance {self.desired_distance_wall}, Current self.Sensor: {self.Sensor}")
                self.old_desired_distance_wall = self.desired_distance_wall

        
    def detectBlockPos(self):
        if self.Utils.blockPositions[self.corner] and self.Utils.blockPositions[self.corner]["position"] == "3" and self.Utils.blockPositions[self.corner]["color"] == "red" and 90 < self.Utils.Cam.avg_edge_distance < 150:
            if not self.active_block_drive:
                self.Utils.LogDebug("active_block_drive")
                
            self.active_block_drive = True
        else:
            self.active_block_drive = False

        if not self.active_block_drive:
            if (self.Utils.Cam.avg_edge_distance < 220) and -15 < self.relative_angle < 40 and self.direction == 1:
                #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
                self.nextBlock['position'] = "0"
                if (120 < self.Utils.Cam.avg_edge_distance < 150) and self.nextBlock['x'] < 300 and 50 < self.nextBlock["distance"] < 90 and self.timelastcorner + 1.5 < time.time(): # and next_corner not in Utils.blockPositions:
                    self.nextBlock['position'] = "1"
                    BlockPos = self.next_corner
                                            
                elif 75 < self.block_distance_to_wall < 130 and 45 < self.nextBlock["distance"] < 120: # and self.timelastcorner + 1 < time.time(): # and corner not in Utils.blockPositions
                    self.nextBlock['position'] = "3"
                    BlockPos = self.corner
                    
                # elif 45 < self.nextBlock['distance'] < 70 and self.timelastcorner + 1.5 < time.time():
                #     self.nextBlock['position'] = "2"
                #     BlockPos = corner
                
                if self.nextBlock['position'] != "0":
                    self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                        
            elif (self.Utils.Cam.avg_edge_distance < 200) and -35 < self.relative_angle < 5 and self.direction == 0:
                # Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
                if (120 < self.Utils.Cam.avg_edge_distance < 150) and self.nextBlock['x'] > 980 and 70 < self.nextBlock["distance"] < 110:
                    self.nextBlock['position'] = "1"
                    BlockPos = self.corner + 1 if self.corner < 3 else 0
                    
                elif 85 < self.block_distance_to_wall < 130 and 45 < self.nextBlock["distance"] < 120 and self.timelastcorner + 1.5 < time.time() and self.corner not in self.Utils.blockPositions:
                    self.nextBlock['position'] = "3"
                    BlockPos = self.corner
                    
                # elif corner not in Utils.blockPositions and 45 < self.nextBlock['distance'] < 70:
                #     self.nextBlock['position'] = "2"
                #     BlockPos = corner

                else:
                    self.nextBlock['position'] = "0"
                
                if self.nextBlock['position'] != "0":
                    self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                    
        elif self.nextBlock2:
            print("nextBlock2" + self.nextBlock2)
                
                
    def currentCornerCases(self):
        if self.direction == 0:
            if self.Utils.blockPositions[self.corner]["color"] == "green" and self.timelastgreenpos1 + 1.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.LogInfo("green direction 0 start")
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 2
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("green direction 0 start")
                    
            elif self.Utils.blockPositions[self.corner]["color"] == "red" and self.timelastredpos1 + 1.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 1
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("red direction 0 start")
                    
        elif self.direction == 1:
            if self.Utils.blockPositions[self.corner]["color"] == "red" and self.timelastredpos1 + 1.5 < time.time() and self.timelastcorner + 1 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 1
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("red direction 1 start")
                    
            elif self.Utils.blockPositions[self.corner]["color"] == "green" and self.timelastgreenpos1 + 1.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 2
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("green direction 1 start")
                    
                    
    def nextCornerCases(self):
        if self.direction == 0:
            if self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "green":
                if not self.ESPAdjusted and 70 < self.Utils.Cam.avg_edge_distance < 150 and self.timelastgreenpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.Sensor = 2
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    self.Utils.LogInfo("Pos 1 green direction 0 start")
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 70:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 1
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    self.Utils.LogInfo("Pos 1 green direction 0 end")
                    self.timelastgreenpos1 = time.time()
                    
            elif self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "red":
                if not self.ESPAdjusted and 140 < self.Utils.Cam.avg_edge_distance < 180 and self.timelastredpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 70", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 70
                    self.Sensor = 1
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    self.Utils.LogInfo("Pos 1 red direction 0 start")
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 140:
                    self.Utils.usb_communication.sendMessage("D 10", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 10
                    self.Sensor = 1
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
                    self.Sensor = 1
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 70:
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 2
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    
            elif self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "green":
                if not self.ESPAdjusted and 140 < self.Utils.Cam.avg_edge_distance < 180 and self.timelastgreenpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 70", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 70
                    self.Sensor = 2
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.timelastredpos1 = time.time()
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 140:
                    self.Utils.usb_communication.sendMessage("D 10", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 10
                    self.Sensor = 1
                    self.ESPAdjusted = False
                    self.block_wide_corner = False
                    self.timelastgreenpos1 = time.time()
                    
                    
    def driveWideCorner(self):
        # self.Utils.LogDebug(f"avg_edge_distance: {self.Utils.Cam.avg_edge_distance}, relative_angle: {self.relative_angle}, direction: {self.direction}, self.block_wide_corner: {self.block_wide_corner}, self.ESPAdjustedCorner: {self.ESPAdjustedCorner}, lastcorner: {self.timelastcorner + 2 < time.time()}")
        if self.direction == 1 and 90 < self.Utils.Cam.avg_edge_distance < 130 and -15 < self.relative_angle < 50 and self.timelastcorner + 2 < time.time() and not self.block_wide_corner:
            if not self.ESPAdjustedCorner:
                print("wide corner start")
                # self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                self.Sensor = 1
                self.ESPAdjustedCorner = True
                
        elif self.direction == 0 and 90 < self.Utils.Cam.avg_edge_distance < 130 and -50 < self.relative_angle < 15 and self.timelastcorner + 2 < time.time() and self.timelastredpos1 + 3 < time.time() and not self.block_wide_corner:
            if not self.ESPAdjustedCorner:
                print("wide corner start")
                # self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                self.Sensor = 2
                self.ESPAdjustedCorner = True
                
        elif self.Utils.Cam.avg_edge_distance < 90 and self.timelastcorner + 2 < time.time():
            if self.ESPAdjustedCorner:
                if self.direction == 0:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.Sensor = 1
                    self.ESPAdjustedCorner = False
                    self.drive_corner = True
                    print("wide corner end ", self.Utils.Cam.avg_edge_distance)
                elif self.direction == 1 and -10 < self.relative_angle < 50:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.Sensor = 2
                    self.ESPAdjustedCorner = False
                    self.drive_corner = True
                    print("wide corner end ", self.Utils.Cam.avg_edge_distance)
                    
                    
    def cornerStuff(self):
        if self.direction == 0:            
            self.relative_angle = self.Utils.Gyro.angle - self.oldAngle 
            newAngle = self.oldAngle - self.GyroCornerAngle
            if self.Utils.Gyro.angle < newAngle and time.time() > self.TIMEOUT:
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
                self.TIMEOUT = time.time() + self.CornerWaitTime
                
        elif self.direction == 1:
            self.relative_angle = self.Utils.Gyro.angle - self.oldAngle
            
            newAngle = self.oldAngle + self.GyroCornerAngle
            if self.Utils.Gyro.angle > newAngle and time.time() > self.TIMEOUT:
                self.corner = self.corner + 1
                self.timelastcorner = time.time()
                
                self.Utils.LogDebug(f"Corner: {self.corner}")
                
                if self.corner == 4:
                    self.corner = 0
                    self.rounds = self.rounds + 1
                    self.Utils.Display.write(f"Corner: {self.corner}", f"Round: {self.rounds}")
        
                self.oldAngle = newAngle
                self.drive_corner = False
                self.sensorAdjustedCorner = False
                self.Utils.Gyro.angle = self.Utils.Gyro.angle - 20
                
                self.TIMEOUT = time.time() + self.CornerWaitTime
                
            if self.timelastcorner + 0.5 < time.time() and not self.ESPAdjustedCorner and not self.ESPAdjusted and not self.sensorAdjustedCorner:
                self.Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                self.Utils.usb_communication.sendMessage(f"D50", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 1 Corner")
                self.sensorAdjustedCorner = True
                
                
    def avoid_sharp_angle(self):
        if self.direction == 0 and self.relative_angle < -45:
            if self.Sensor == 1:
                new_distance = self.desired_distance_wall
            elif self.Sensor == 2:
                new_distance = 100 - self.desired_distance_wall
            
            if self.Sensor != 1:
                self.Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 1")
                
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            
        elif self.direction == 1 and self.relative_angle > 45:
            if self.Sensor == 2:
                new_distance = self.desired_distance_wall
            elif self.Sensor == 1:
                new_distance = 100 - self.desired_distance_wall
            
            if self.Sensor != 2:
                self.Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 2")
                
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            
            



##########################################################
##                                                      ##
##                     Main Code                        ##
##                                                      ##
##########################################################
if __name__ == "__main__":
    try:
        Holdlane = HoldLane(Utils)
        
        #start flask server if needed
        if Cam.video_stream:
            app = Flask(__name__)
            
            @app.route('/')
            def index():
                return render_template('videofeed.html')


            @app.route('/video_feed_1')
            def video_feed_1():
                return Response(Cam.video_frames("type1"),
                                mimetype='multipart/x-mixed-replace; boundary=frame')
                
            @app.route('/video_feed_2')
            def video_feed_2():
                return Response(Cam.video_frames("type2"),
                                mimetype='multipart/x-mixed-replace; boundary=frame')
                
            @app.route('/status')
            def status():
                return Response("Running", mimetype='text/plain')
                                
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