import time
from RobotCarClasses import *
from threading import Thread
import traceback
from flask import Flask, render_template, Response, jsonify
from math import atan2
from copy import deepcopy

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

Cam = Camera(video_stream=True, enable_video_writer=False, Utils=Utils)

global ESPHoldDistance, ESPHoldSpeed
ESPHoldDistance, ESPHoldSpeed = Utils.transferSensorData(StartButton, StopButton, Buzzer1, Cam)

Utils.setupDataLog()


##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
Utils.StartSpeed = 110
Utils.Distance = 50
Utils.Kp = 3
Utils.Kd = 1
Utils.Ed = 125 # Edge detection distance in cm
Utils.StartSensor = 2
Utils.Mm = 10
Utils.AngR = 41
Utils.AngL = 46
Utils.startMode = 2

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
        self.before_safety_state = [self.Sensor, self.desired_distance_wall]
        self.desired_distance_wall_smart = 50
        
        self.timelastcorner = 0
        self.timelastgreenpos1 = 0
        self.timelastredpos1 = 0
        self.timelastwidecorner = 0
        
        #Cutoffs
        self.YCutOffTop = 0
        self.SIZE = 0
        
        self.ESPAdjustedCorner = False
        self.ESPAdjusted = False
        self.ESPAdjusted_2 = False
        self.detect_new_block = True
        self.block_wide_corner = False
        self.drive_corner = False
        self.sensorAdjustedCorner = False
        self.active_block_drive = False
        self.safetyEnabled = False
        self.safetyEnabled_inside = False
        
        self.coordinates_self = (640, 720) #x, y
        self.middledistance = 50
        self.KP = 0.23
        self.desired_distance_to_block_red = -650
        self.desired_distance_to_block_green = 650
        self.old_desired_distance_wall = 50


    def avoidBlocks(self):        
        while not self.Utils.Cam.frames:
            time.sleep(0.0001)
        
        case = -1
        if self.Utils.Cam.avg_edge_distance > 150:
            case = 0
        
        elif self.Utils.Cam.avg_edge_distance > 100:
            case = 1
            
        if case != -1:
            drive_directions = []
            for _ in range(3):
                drive_direction = self.get_drive_direction(self.Utils.Cam.frames[2], case)
                
                if drive_direction != -1:
                    drive_directions.append(drive_direction)
                                
            self.direction = round(np.mean(drive_directions))
            
            # print("drive_directions", drive_directions)
            # print("self.direction", self.direction)
            # print("case", case)
            
        
        else:
            self.Utils.LogError("Edge detection could not decide on case, maybe not working correctly")

        self.Utils.usb_communication.sendMessage(f"START", ESPHoldSpeed)
        self.Utils.usb_communication.sendMessage(f"SPEED {self.Utils.StartSpeed}", ESPHoldSpeed)
        #Hold Lane
        while self.Utils.running:
            time.sleep(0.0001)
            # print(self.relative_angle)
            
            self.cornerStuff()
            # self.avoid_sharp_angle()
            # print(self.safetyEnabled)
            
            self.Utils.Cam.desired_distance_wall = self.desired_distance_wall
            self.Utils.relative_angle = self.relative_angle
                    
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
                    # self.block_distance_to_wall = self.Utils.Cam.avg_edge_distance - self.nextBlock['distance']
                    # self.Utils.LogDebug(f"avg_edge_distance: {self.Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['x']: {self.nextBlock['x']}, self.nextBlock['y']: {self.nextBlock['y']}")
                    
                    self.detectBlockPos()
                    
                    blockPos1 = False
                    if self.next_corner in self.Utils.blockPositions:
                        if self.Utils.blockPositions[self.next_corner]["position"] == "1":
                            blockPos1 = True
                               
                    if (15 < self.nextBlock["distance"] < 90 and not self.ESPAdjusted and not self.ESPAdjustedCorner and self.corner not in self.Utils.blockPositions and 
                        not self.safetyEnabled and not self.safetyEnabled_inside and not blockPos1 and not self.drive_corner):
                        self.smartSteer()
                        
                else:
                    self.nextBlock = None
              
            self.avoid_sharp_angle()
                
            if self.corner in self.Utils.blockPositions:
                self.currentCornerCases()
            
            if self.next_corner in Utils.blockPositions:
                self.nextCornerCases()
                
            if (self.corner in self.Utils.blockPositions and 
                ((self.Utils.blockPositions[self.corner]["position"] == "3" and self.Utils.blockPositions[self.corner]["color"] == "red") or 
                (self.Utils.blockPositions[self.corner]["position"] == "3" and self.Utils.blockPositions[self.corner]["color"] == "green")) and 
                90 < self.Utils.Cam.avg_edge_distance < 160):
                
                if not self.active_block_drive:
                    self.Utils.LogDebug("active_block_drive")
                    
                self.active_block_drive = True
            else:
                self.active_block_drive = False
            
            # # Drive to the middle of the lane
            # if self.smooth_to_middle:
            #     if smoothing_counter >= 10:
            #         if self.desired_distance_wall > 52:
            #             self.desired_distance_wall = self.desired_distance_wall - 4
                        
            #         elif self.desired_distance_wall < 48:
            #             self.desired_distance_wall = self.desired_distance_wall + 4
                        
            #         else:
            #             self.desired_distance_wall = 50
            #             self.smooth_to_middle = False
                        
            #         smoothing_counter = 0
            #         self.Utils.usb_communication.sendMessage(f"D {self.desired_distance_wall}", self.Utils.ESPHoldDistance)
                        
            #     else:
            #         smoothing_counter += 1
                
            # Drive wide around corners
            self.driveWideCorner()
                                    
            responses = self.Utils.usb_communication.getResponses(ESPHoldDistance)
            if responses != None:
                # print(responses)
                for response in responses:
                    if "Drive direction counterclockwise" in responses:
                        self.direction = 1
                    if "Drive direction clockwise" in responses:
                        self.direction = 0
                    if response.startswith("RD"):
                        self.Utils.LogDebug(f"Response: {response}")
                    if response.startswith("RS"):
                        self.Utils.LogDebug(f"Response: {response}")
            
            if self.rounds == 3 and self.timelastcorner + 2 < time.time():
                if self.Utils.Cam.avg_edge_distance < 165:
                    self.Utils.running = False
                    self.Utils.LogInfo(f"End of path reached {self.Utils.Cam.avg_edge_distance}")
            
            self.Utils.LogData()
            # self.Utils.LogDebug(f"self.ESPAdjusted: {self.ESPAdjusted}, self.ESPAdjustedCorner: {self.ESPAdjustedCorner}, self.desired_distance_wall: {self.desired_distance_wall}, relative_angle: {self.relative_angle}, direction: {self.direction}")
           
            
    def smartSteer(self):
        self.nextBlock['distancex'] = -(self.coordinates_self[0] - self.nextBlock['mx'])
        # self.nextBlock['distancey'] = self.coordinates_self[1] - self.nextBlock['y']
        
        if self.nextBlock['color'] == "red":
            desired_distance_to_block = self.desired_distance_to_block_red
        elif self.nextBlock['color'] == "green":
            desired_distance_to_block = self.desired_distance_to_block_green
            
        distance_divider = 30 / self.nextBlock['distance']
        
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
            if self.Sensor != 2:
                self.Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 2 Smart Steer")
                
            # self.Utils.LogInfo(f"desired_distance_wall {self.desired_distance_wall}, old_desired_distance_wall {self.old_desired_distance_wall}")
            if abs(self.desired_distance_wall - self.old_desired_distance_wall) > 3:
                self.Utils.usb_communication.sendMessage(f"D{self.desired_distance_wall}", ESPHoldDistance)
                self.Utils.LogInfo(f"New Distance {self.desired_distance_wall}, Current self.Sensor: {self.Sensor}")
                self.old_desired_distance_wall = self.desired_distance_wall
                
        self.desired_distance_wall_smart = self.desired_distance_wall

        
    def detectBlockPos(self):
        pos1_x_area = [0, 0]
        if self.direction == 1:
            if self.desired_distance_wall <= 30 and self.Sensor == 2:
                pos1_x_area = [230, 550]
            elif 30 < self.desired_distance_wall < 70:
                pos1_x_area = [50, 320]
            elif self.desired_distance_wall <= 30 and self.Sensor == 1:
                pos1_x_area = [0, 300]
        
        elif self.direction == 0:
            if self.desired_distance_wall <= 30 and self.Sensor == 1:
                pos1_x_area = [730, 1050]
            elif 30 < self.desired_distance_wall < 70:
                pos1_x_area = [960, 1280]
            elif self.desired_distance_wall <= 30 and self.Sensor == 2:
                pos1_x_area = [980, 1280]
            
        
        if not self.active_block_drive:
            #Utils.LogInfo(f"avg_edge_distance: {Utils.Cam.avg_edge_distance}, distance: {self.nextBlock['distance']}, self.block_distance_to_wall: {self.block_distance_to_wall}, self.nextBlock['mx']: {self.nextBlock['mx']}, self.nextBlock['y']: {self.nextBlock['y']}")
            
            self.block_distance_to_wall = self.Utils.Cam.avg_edge_distance - self.nextBlock['distance']
            
            # Try to make the x area for pos1 somewhat dynamic
            # self.Utils.LogInfo(f"nextblock[x]: {self.nextBlock['mx']}")
            

            if (self.Utils.Cam.avg_edge_distance < 220) and -15 < self.relative_angle < 40 and self.direction == 1:
                self.nextBlock['position'] = "0"
                if (130 < self.Utils.Cam.avg_edge_distance < 160) and self.block_distance_to_wall < 85 and pos1_x_area[0] < self.nextBlock['mx'] < pos1_x_area[1] and 50 < self.nextBlock["distance"] < 105 and self.timelastcorner + 1.5 < time.time() and not self.drive_corner: # and next_corner not in Utils.blockPositions:
                    self.nextBlock['position'] = "1"
                    BlockPos = self.next_corner
                                            
                elif 95 < self.block_distance_to_wall < 120 and 70 < self.nextBlock["distance"] < 100 and self.timelastcorner + 1.5 < time.time(): # and corner not in Utils.blockPositions
                    self.nextBlock['position'] = "3"
                    BlockPos = self.corner
                
                if self.nextBlock['position'] != "0":
                    self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                        
            elif (self.Utils.Cam.avg_edge_distance < 220) and -35 < self.relative_angle < 15 and self.direction == 0:
                self.nextBlock['position'] = "0"
                if (130 < self.Utils.Cam.avg_edge_distance < 180) and self.block_distance_to_wall < 95 and pos1_x_area[0] < self.nextBlock['mx'] < pos1_x_area[1] and 50 < self.nextBlock["distance"] < 105 and self.timelastcorner + 1.5 < time.time():
                    self.nextBlock['position'] = "1"
                    BlockPos = self.corner + 1 if self.corner < 3 else 0
                    
                elif 95 < self.block_distance_to_wall < 120 and 70 < self.nextBlock["distance"] < 100 and self.timelastcorner + 1.5 < time.time():# and self.corner not in self.Utils.blockPositions:
                    self.nextBlock['position'] = "3"
                    BlockPos = self.corner

                if self.nextBlock['position'] != "0":
                    self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                    
        elif self.nextBlock2:
            print(self.nextBlock2['color'], self.nextBlock2['distance'], self.nextBlock2['mx'])
            
            self.block_distance_to_wall = self.Utils.Cam.avg_edge_distance - self.nextBlock2['distance']
            
            if (self.Utils.Cam.avg_edge_distance < 220) and -15 < self.relative_angle < 40 and self.direction == 1:
                self.nextBlock['position'] = "0"
                if (120 < self.Utils.Cam.avg_edge_distance < 200) and pos1_x_area[0] < self.nextBlock['mx'] < pos1_x_area[1] and 50 < self.nextBlock["distance"] < 120 and self.timelastcorner + 1.5 < time.time(): # and next_corner not in Utils.blockPositions:
                    self.nextBlock['position'] = "1"
                    BlockPos = self.next_corner

                if self.nextBlock['position'] != "0":
                    self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                        
            elif (self.Utils.Cam.avg_edge_distance < 200) and -35 < self.relative_angle < 5 and self.direction == 0:
                self.nextBlock['position'] = "0"
                if (120 < self.Utils.Cam.avg_edge_distance < 150) and pos1_x_area[0] < self.nextBlock['mx'] < pos1_x_area[1] and 50 < self.nextBlock["distance"] < 90 and self.timelastcorner + 1.5 < time.time():
                    self.nextBlock['position'] = "1"
                    BlockPos = self.corner + 1 if self.corner < 3 else 0

                if self.nextBlock['position'] != "0":
                    self.Utils.blockPositions.update({BlockPos: {"position": self.nextBlock['position'], "color": self.nextBlock['color']}})
                
                
    def currentCornerCases(self):
        if self.direction == 0:
            if self.Utils.blockPositions[self.corner]["color"] == "green" and self.timelastgreenpos1 + 1 < time.time() and self.timelastcorner + 1 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.LogInfo("green direction 0 start")
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 2
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("green direction 0 start")
                    
            elif self.Utils.blockPositions[self.corner]["color"] == "red" and self.timelastredpos1 + 1.5 < time.time() and self.timelastcorner + 0.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner and self.Utils.Cam.avg_edge_distance < 120 and self.timelastcorner + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 90", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 90
                    self.Sensor = 2
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo(f"red direction 0 start corner {self.relative_angle}")
                    
                elif not self.ESPAdjusted:
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 1
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.Utils.LogInfo("red direction 0 start")
                    
        elif self.direction == 1:
            if self.Utils.blockPositions[self.corner]["color"] == "red" and self.timelastredpos1 + 1 < time.time() and self.timelastcorner + 1 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner:
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 1
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("red direction 1 start")
                    
            elif self.Utils.blockPositions[self.corner]["color"] == "green" and self.timelastgreenpos1 + 1.5 < time.time() and self.timelastcorner + 0.5 < time.time() and not self.drive_corner:
                if not self.ESPAdjustedCorner and self.Utils.Cam.avg_edge_distance < 110 and self.timelastcorner + 2.5 < time.time():
                    self.Utils.usb_communication.sendMessage("D 80", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 80
                    self.Sensor = 1
                    self.ESPAdjustedCorner = True
                    self.block_wide_corner = False
                    self.Utils.LogInfo("green direction 1 start corner")
                    
                elif not self.ESPAdjusted:
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 15", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 15
                    self.Sensor = 2
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.Utils.LogInfo("green direction 1 start")
                    
                    
    def nextCornerCases(self):
        if self.direction == 0:
            if self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "green" and self.timelastcorner + 1.5 < time.time():
                if not self.ESPAdjusted and 70 < self.Utils.Cam.avg_edge_distance < 150 and self.timelastgreenpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.Sensor = 2
                    self.ESPAdjusted_2 = True
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    self.Utils.LogInfo("Pos 1 green direction 0 start")
                    
                elif self.ESPAdjusted_2 and self.Utils.Cam.avg_edge_distance < 60:
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 0", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 1
                    self.Sensor = 1
                    self.ESPAdjusted_2 = False
                    self.ESPAdjusted = True
                    self.timelastwidecorner = time.time()
                    self.block_wide_corner = True
                    self.drive_corner = True
                    self.Utils.LogInfo("Pos 1 green direction 0 end")
                    
            elif self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "red":
                if not self.ESPAdjusted and 140 < self.Utils.Cam.avg_edge_distance < 180 and self.timelastredpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("D 70", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 70
                    self.Sensor = 1
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.timelastgreenpos1 = time.time()
                    self.Utils.LogInfo("Pos 1 red direction 0 start")
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 140:
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 10", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 10
                    self.Sensor = 1
                    self.ESPAdjusted = False
                    self.block_wide_corner = True
                    self.Utils.LogInfo("Pos 1 red direction 0 end")
                    self.timelastredpos1 = time.time()
                    self.drive_corner = True
                    
        elif self.direction == 1:
            if self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "red" and self.timelastcorner + 1 < time.time():
                if not self.ESPAdjusted_2 and 65 < self.Utils.Cam.avg_edge_distance < 150:
                    # self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.Sensor = 1
                    self.ESPAdjusted_2 = True
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.ESPAdjustedCorner = False
                    self.Utils.LogInfo("Pos 1 red direction 1 start")
                    
                elif self.ESPAdjusted_2 and self.Utils.Cam.avg_edge_distance < 65:
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 0", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 1
                    self.Sensor = 2
                    self.ESPAdjusted_2 = False
                    self.ESPAdjusted = True
                    self.timelastwidecorner = time.time()
                    self.block_wide_corner = True
                    self.timelastredpos1 = time.time()
                    self.drive_corner = True
                    self.Utils.LogInfo("Pos 1 red direction 1 end")
                    
            elif self.Utils.blockPositions[self.next_corner]["position"] == "1" and self.Utils.blockPositions[self.next_corner]["color"] == "green" and self.timelastcorner + 1 < time.time():
                if not self.ESPAdjusted and 140 < self.Utils.Cam.avg_edge_distance < 180 and self.timelastgreenpos1 + 3 < time.time():
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 70", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 70
                    self.Sensor = 2
                    self.ESPAdjusted = True
                    self.block_wide_corner = True
                    self.Utils.LogInfo("Pos 1 green direction 1 start")
                    
                elif self.ESPAdjusted and self.Utils.Cam.avg_edge_distance < 140:
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("D 10", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 10
                    self.Sensor = 1
                    self.ESPAdjusted = False
                    self.block_wide_corner = True
                    self.timelastgreenpos1 = time.time()
                    self.drive_corner = True
                    self.Utils.LogInfo("Pos 1 green direction 1 end")
                    
                    
    def driveWideCorner(self):
        # self.Utils.LogDebug(f"avg_edge_distance: {self.Utils.Cam.avg_edge_distance}, relative_angle: {self.relative_angle}, direction: {self.direction}, self.block_wide_corner: {self.block_wide_corner}, self.ESPAdjustedCorner: {self.ESPAdjustedCorner}, lastcorner: {self.timelastcorner + 2 < time.time()}")
        if (self.direction == 1 and 90 < self.Utils.Cam.avg_edge_distance < 110 and -15 < self.relative_angle < 50 and self.timelastcorner + 2 < time.time() and not self.block_wide_corner and 
            self.timelastwidecorner + 2 < time.time()):
            if not self.ESPAdjustedCorner:
                print("wide corner start")
                self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)
                
                if self.desired_distance_wall_smart < 50:
                    desired_distance_wall_other_direction = 100 - self.desired_distance_wall_smart
                    self.Utils.usb_communication.sendMessage(f"D {desired_distance_wall_other_direction}", self.Utils.ESPHoldDistance)
                    
                self.Sensor = 1
                self.ESPAdjustedCorner = True
                
        elif self.direction == 0 and 90 < self.Utils.Cam.avg_edge_distance < 110 and -50 < self.relative_angle < 15 and self.timelastcorner + 2 < time.time() and not self.block_wide_corner and self.timelastwidecorner + 2 < time.time():
            if not self.ESPAdjustedCorner:
                print("wide corner start")
                self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                
                if self.desired_distance_wall_smart > 50:
                    desired_distance_wall_other_direction = 100 - self.desired_distance_wall_smart
                    self.Utils.usb_communication.sendMessage(f"D {desired_distance_wall_other_direction}", self.Utils.ESPHoldDistance)
                    
                self.Sensor = 2
                self.ESPAdjustedCorner = True
                
        elif self.Utils.Cam.avg_edge_distance < 90 and self.timelastcorner + 2.5 < time.time():
            if self.ESPAdjustedCorner:
                if self.direction == 0 and -50 < self.relative_angle < 20 and not self.active_block_drive and not self.nextBlock:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S1", self.Utils.ESPHoldDistance)      
                                  
                    self.desired_distance_wall = 50
                    self.Sensor = 1
                    self.ESPAdjustedCorner = False
                    self.drive_corner = True
                    print("wide corner end ", self.Utils.Cam.avg_edge_distance)
                    self.timelastwidecorner = time.time()
                    
                elif self.direction == 1 and -10 < self.relative_angle < 50 and not self.active_block_drive and not self.nextBlock:
                    self.Utils.usb_communication.sendMessage("D 50", self.Utils.ESPHoldDistance)
                    self.Utils.usb_communication.sendMessage("S2", self.Utils.ESPHoldDistance)
                    self.desired_distance_wall = 50
                    self.Sensor = 2
                    self.ESPAdjustedCorner = False
                    self.drive_corner = True
                    print("wide corner end ", self.Utils.Cam.avg_edge_distance)
                    self.timelastwidecorner = time.time()
                    
                    
    def cornerStuff(self):
        if self.direction == 0:            
            self.relative_angle = self.Utils.Gyro.angle - self.oldAngle 
            
            newAngle = self.oldAngle - self.GyroCornerAngle
            if self.Utils.Gyro.angle < newAngle and time.time() > self.TIMEOUT:
                self.corner = self.corner + 1
                self.Utils.Gyro.angle = self.Utils.Gyro.angle + 22
                self.timelastcorner = time.time()
                
                self.Utils.LogDebug(f"Corner: {self.corner}")
                self.Utils.Display.write(f"Corner: {self.corner}")
                
                if self.corner == 4:
                    self.corner = 0
                    self.rounds = self.rounds + 1
                    self.Utils.Display.write(f"Corner: {self.corner}", f"Round: {self.rounds}")
                    
                self.oldAngle = newAngle
                self.drive_corner = False
                self.sensorAdjustedCorner = False
                self.ESPAdjusted = False
                self.block_wide_corner = False
                self.ESPAdjustedCorner = False
                self.desired_distance_wall_smart = 50
                
                self.TIMEOUT = time.time() + self.CornerWaitTime
                
            if (self.timelastcorner + 0.75 < time.time() and not self.ESPAdjustedCorner and not self.ESPAdjusted and 
                not self.sensorAdjustedCorner and not self.safetyEnabled and not self.safetyEnabled_inside):
                if (self.nextBlock and 30 < self.nextBlock["distance"] < 90) or self.drive_corner:
                    self.sensorAdjustedCorner = True
                    return
                
                self.Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                self.Utils.usb_communication.sendMessage(f"D50", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 2 Corner")
                self.sensorAdjustedCorner = True
                
        elif self.direction == 1:
            self.relative_angle = self.Utils.Gyro.angle - self.oldAngle
            
            newAngle = self.oldAngle + self.GyroCornerAngle
            if self.Utils.Gyro.angle > newAngle and time.time() > self.TIMEOUT:
                self.Utils.Gyro.angle = self.Utils.Gyro.angle - 23
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
                self.sensorAdjustedCorner = False
                self.ESPAdjusted = False
                self.block_wide_corner = False
                self.ESPAdjustedCorner = False
                self.desired_distance_wall_smart = 50
                
                self.TIMEOUT = time.time() + self.CornerWaitTime
                
            if (self.timelastcorner + 0.75 < time.time() and not self.ESPAdjustedCorner and not self.ESPAdjusted and 
                not self.sensorAdjustedCorner and not self.safetyEnabled and not self.safetyEnabled_inside):
                if (self.nextBlock and 30 < self.nextBlock["distance"] < 90) or self.drive_corner:
                    self.sensorAdjustedCorner = True
                    return
                
                self.Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
                self.Utils.usb_communication.sendMessage(f"D50", ESPHoldDistance)
                self.Utils.LogInfo(f"Switched to self.Sensor 1 Corner")
                self.sensorAdjustedCorner = True
                
                
    def avoid_sharp_angle(self):
        if self.drive_corner:
            return
        
        if self.direction == 0 and self.relative_angle > 31 and not self.safetyEnabled and not self.drive_corner and self.timelastcorner + 0.5 < time.time():
            self.before_safety_state = [self.Sensor, self.desired_distance_wall]
            
            if self.Sensor == 1:
                new_distance = self.desired_distance_wall
            elif self.Sensor == 2:
                new_distance = 100 - self.desired_distance_wall
            
            if self.Sensor != 1:
                self.Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
            self.Utils.LogInfo(f"Switched to self.Sensor 1 Safety")
                
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            self.safetyEnabled = True
            
        elif self.direction == 1 and self.relative_angle < -31 and not self.safetyEnabled and not self.drive_corner and self.timelastcorner + 0.5 < time.time():
            self.before_safety_state = [self.Sensor, self.desired_distance_wall]
            
            if self.Sensor == 2:
                new_distance = self.desired_distance_wall
            elif self.Sensor == 1:
                new_distance = 100 - self.desired_distance_wall
            
            if self.Sensor != 2:
                self.Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
                
            self.Utils.LogInfo(f"Switched to self.Sensor 2 Safety")
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            self.safetyEnabled = True
            
        elif self.direction == 0 and self.safetyEnabled and self.relative_angle < 31:
            self.Utils.usb_communication.sendMessage(f"S{self.before_safety_state[0]}", ESPHoldDistance)
            self.Utils.usb_communication.sendMessage(f"D{self.before_safety_state[1]}", ESPHoldDistance)
            self.Sensor = self.before_safety_state[0]
            self.desired_distance_wall = self.before_safety_state[1]
            self.safetyEnabled = False
            
            self.Utils.LogInfo(f"Switched to self.Sensor {self.before_safety_state[0]} Safety end")
            
        elif self.direction == 1 and self.safetyEnabled and self.relative_angle > -31:
            self.Utils.usb_communication.sendMessage(f"S{self.before_safety_state[0]}", ESPHoldDistance)
            
            new_distance = self.before_safety_state[1] * 1
            if new_distance < 15:
                new_distance = 15
            
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            self.Sensor = self.before_safety_state[0]
            self.desired_distance_wall = new_distance
            self.safetyEnabled = False
            
            self.Utils.LogInfo(f"Switched to self.Sensor {self.before_safety_state[0]} Safety end")
            
        elif self.direction == 1 and self.relative_angle > 30 and not self.safetyEnabled_inside and self.timelastcorner + 0.5 < time.time():
            self.before_safety_state = [self.Sensor, self.desired_distance_wall]
            
            if self.Sensor == 1:
                new_distance = self.desired_distance_wall
            elif self.Sensor == 2:
                new_distance = 100 - self.desired_distance_wall
            
            if self.Sensor != 1:
                self.Sensor = 1
                self.Utils.usb_communication.sendMessage(f"S1", ESPHoldDistance)
            self.Utils.LogInfo(f"Switched to self.Sensor 1 Safety")
                
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            self.safetyEnabled_inside = True       
            
        elif self.direction == 1 and self.relative_angle < 30 and self.safetyEnabled_inside:
            self.Utils.usb_communication.sendMessage(f"S{self.before_safety_state[0]}", ESPHoldDistance)
            self.Utils.usb_communication.sendMessage(f"D{self.before_safety_state[1]}", ESPHoldDistance)
            self.Sensor = self.before_safety_state[0]
            self.desired_distance_wall = self.before_safety_state[1]
            self.safetyEnabled_inside = False
            
            self.Utils.LogInfo(f"Switched to self.Sensor {self.before_safety_state[0]} Safety end")
            
        elif self.direction == 0 and self.relative_angle < -45 and not self.safetyEnabled_inside and self.timelastcorner + 0.5 < time.time():
            self.before_safety_state = [self.Sensor, self.desired_distance_wall]
            
            if self.Sensor == 2:
                new_distance = self.desired_distance_wall
            elif self.Sensor == 1:
                new_distance = 100 - self.desired_distance_wall
            
            if self.Sensor != 2:
                self.Sensor = 2
                self.Utils.usb_communication.sendMessage(f"S2", ESPHoldDistance)
            self.Utils.LogInfo(f"Switched to self.Sensor 2 Safety")
                
            self.Utils.usb_communication.sendMessage(f"D{new_distance}", ESPHoldDistance)
            self.safetyEnabled_inside = True
            
        elif self.direction == 0 and self.relative_angle > -45 and self.safetyEnabled_inside:
            self.Utils.usb_communication.sendMessage(f"S{self.before_safety_state[0]}", ESPHoldDistance)
            self.Utils.usb_communication.sendMessage(f"D{self.before_safety_state[1]}", ESPHoldDistance)
            self.Sensor = self.before_safety_state[0]
            self.desired_distance_wall = self.before_safety_state[1]
            self.safetyEnabled_inside = False
            
            self.Utils.LogInfo(f"Switched to self.Sensor {self.before_safety_state[0]} Safety end")


    def get_drive_direction(self, inputFrame, case):
        frame = deepcopy(inputFrame)
        frame_width = frame.shape[1]
        middle_third_start = frame_width//3
        middle_third_end = (2*frame_width)//3

        self.kernel = self.Utils.Cam.kernel

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.dilate(gray, self.kernel, iterations=1)

        # Threshold the grayscale image to get a binary image
        binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]

        binary = cv2.dilate(binary, self.kernel, iterations=2)

        if case == 0:
            binary = binary[300:, :]
            frame = frame[300:, :]
            edges = cv2.Canny(binary, 20, 30, apertureSize=7)

            # Perform Probabilistic Hough Line Transform
            lines = cv2.HoughLinesP(edges, 4, np.pi/180, 30, minLineLength=50, maxLineGap=30)                

            target_angle = 90  # The angle we want to find the closest to
            closest_angle_diff = float('inf')  # Initialize the closest angle difference to infinity
            vertical_line = None  # Initialize the vertical line

            if lines is not None:
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        average_x = (x1 + x2) / 2
                        # Skip the line if it's in the middle third of the frame
                        if middle_third_start <= average_x <= middle_third_end:
                            continue
                        
                        if abs(y2 - y1) < 20:
                            continue

                        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        # Calculate the angle of the line with respect to the vertical axis
                        angle = degrees(atan2(y2 - y1, x2 - x1))
                        angle = abs(angle)  # Adjust the range to [0, 180]

                        # Calculate the difference between this angle and the target angle
                        angle_diff = abs(target_angle - angle)

                        # If this line is closer to the target angle than the previous closest
                        if angle_diff < closest_angle_diff:
                            closest_angle_diff = angle_diff
                            vertical_line = line
                            
                        cv2.imwrite("frame.jpg", frame)

            if vertical_line is not None:
                x1, y1, x2, y2 = vertical_line[0]
                average_x = (x1 + x2) / 2

                cv2.line(frame, (x1, y1), (x2, y2), (244, 255, 0), 2)

                if average_x < 640:
                    return 1
                else:
                    return 0

            return -1
        
        elif case == 1:
            new_binary = deepcopy(binary)
            new_binary = new_binary[250:, :]
            # Get the average pixel value of the left and right side of the image
            left_avg = np.mean(new_binary[:, :new_binary.shape[1]//2])
            right_avg = np.mean(new_binary[:, new_binary.shape[1]//2:])
            
            return 1 if left_avg > right_avg else 0
        
        else:
            raise ValueError("Invalid case number")

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
            
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            
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