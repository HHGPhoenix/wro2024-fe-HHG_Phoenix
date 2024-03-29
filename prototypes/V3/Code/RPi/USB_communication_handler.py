import subprocess
import time
import serial
import threading
import logging
import gpiod

global chip, all_lines

chip = gpiod.Chip('gpiochip4')

all_lines = []

class USBCommunication:
    def __init__(self, Utils):
        self.Utils = Utils
        
        self.HoldSpeedCounter = 0
        self.HoldDistanceCounter = 0
        self.responsesHoldDistance = []
        self.responsesHoldSpeed = []
        self.messageArrayHoldDistance = []
        self.messageArrayHoldSpeed = []
        
        """
        self.EspHoldDistancePin = chip.get_line(24)
        self.EspHoldSpeedPin = chip.get_line(16)
        self.EspHoldDistancePin.request(consumer='USBCommunication', type=gpiod.LINE_REQ_DIR_OUT)
        self.EspHoldSpeedPin.request(consumer='USBCommunication', type=gpiod.LINE_REQ_DIR_OUT)
        all_lines.append(self.EspHoldDistancePin)
        all_lines.append(self.EspHoldSpeedPin)
        self.EspHoldDistancePin.set_value(1)
        self.EspHoldSpeedPin.set_value(1)
        """
        
        
    def handleHeartbeat(self):
        self.sendMessage("H", self.EspHoldSpeed)
        time.sleep(0.01)
        responseHoldSpeed = self.getResponses(self.EspHoldSpeed, "HBProcess")
        if responseHoldSpeed == False:
            self.HoldSpeedCounter += 1
            self.Utils.LogWarning(f"NodeMCU for HoldSpeed is not responding to heartbeat: '{responseHoldSpeed}' x{self.HoldSpeedCounter}")
            
        time.sleep(0.1)

        self.sendMessage("H", self.EspHoldDistance)
        time.sleep(0.01)
        responseHoldDistance = self.getResponses("HoldDistance", "HBProcess")
        if responseHoldDistance == False:
            self.HoldDistanceCounter += 1
            self.Utils.LogWarning(f"NodeMCU for HoldDistance is not responding to heartbeat: '{responseHoldDistance}' x{self.HoldDistanceCounter}")
        
        
    def handleSendMessage(self):
        totalSizeHoldDistance = 0
        for message in self.messageArrayHoldDistance:
            print(message)
            messageSize = len(f"{message}\n".encode())
            if totalSizeHoldDistance + messageSize > 64:
                break
            self.EspHoldDistance.write(f"{message}\n".encode())
            time.sleep(0.01)
            totalSizeHoldDistance += messageSize
            self.messageArrayHoldDistance.remove(message)

        time.sleep(0.05)

        totalSizeHoldSpeed = 0
        for message in self.messageArrayHoldSpeed:
            print(message)
            messageSize = len(f"{message}\n".encode())
            if totalSizeHoldSpeed + messageSize > 64:
                break
            self.EspHoldSpeed.write(f"{message}\n".encode())
            time.sleep(0.01)
            totalSizeHoldSpeed += messageSize
            self.messageArrayHoldSpeed.remove(message)
        
        
    def handleGetResponse(self):
        try:
            responseHoldDistance = self.EspHoldDistance.read(self.EspHoldDistance.inWaiting())
            responseHoldDistance = responseHoldDistance.decode("utf-8").strip().replace('\r', '')
            responsesHoldDistance = responseHoldDistance.split('\n')
            
            for response in responsesHoldDistance:
                self.responsesHoldDistance.append(response)
            
        except UnicodeDecodeError as e:
            self.Utils.LogWarning(f"UnicodeDecodeError for HoldDistance: {e}")
            return None

        try:
            responseHoldSpeed = self.EspHoldSpeed.read(self.EspHoldSpeed.inWaiting())
            responseHoldSpeed = responseHoldSpeed.decode("utf-8").strip().replace('\r', '')
            responsesHoldSpeed = responseHoldSpeed.split('\n')
            
            for response in responsesHoldSpeed:
                self.responsesHoldSpeed.append(response)
            
        
        except UnicodeDecodeError as e:
            self.Utils.LogWarning(f"UnicodeDecodeError for HoldSpeed: {e}")
            return None
        
        
    def handleThreadedFunctions(self):
        counter = 0
        while True:
            self.handleSendMessage()
            self.handleGetResponse()
            if counter == 20:
                self.handleHeartbeat()
                counter = 0
            else:
                counter += 1
        
        
    def sendMessage(self, message, ESP):
        if ESP == self.EspHoldDistance:
            self.messageArrayHoldDistance.append(message)
        
        elif ESP == self.EspHoldSpeed:
            self.messageArrayHoldSpeed.append(message)
        
            
    def getResponses(self, ESP, caller=""):    
        if ESP == self.EspHoldDistance:
            for i, response in enumerate(self.responsesHoldDistance):
                if '' == response:
                    self.responsesHoldDistance.pop(i)
                    
                if caller == "HBProcess":
                    if 'HB' in self.responsesHoldDistance:
                        for i, response in enumerate(self.responsesHoldDistance):
                            if 'HB' == response:
                                self.responsesHoldDistance.pop(i)
                        
                        return True
                    else:
                        return False  
                    
                else:
                    responses = self.responsesHoldDistance
                    self.responsesHoldDistance = []
                    return responses
                      
        elif ESP == self.EspHoldSpeed:
            for i, response in enumerate(self.responsesHoldSpeed):
                if '' == response:
                    self.responsesHoldSpeed.pop(i)
                    
            if caller == "HBProcess":
                if 'HB' in self.responsesHoldSpeed:
                    for i, response in enumerate(self.responsesHoldSpeed):
                        if 'HB' == response:
                            self.responsesHoldSpeed.pop(i)
                    
                    return True
                else:
                    return False
            
            else:
                responses = self.responsesHoldSpeed
                self.responsesHoldSpeed = []
                return responses
        
        
    #Init both NodeMCUs
    def initNodeMCUs(self):
        usb_devices = []
        # Run the 'ls /dev/tty*' command using a shell and capture the output
        result = subprocess.run('ls /dev/tty*', shell=True, stdout=subprocess.PIPE, text=True, check=True)
        
        # Split the output into lines and print each line
        devices = result.stdout.split('\n')
        for device in devices:
            if "/dev/ttyUSB" in device:
                usb_devices.append(device)

        if len(usb_devices) != 2: # Maybe use Constant instead of 2
            self.Utils.LogError(f"Could not find both NodeMCUs: {usb_devices}")
            self.StopRun()
            return
            
        #Identify both NodeMCUs
        for device in usb_devices:
            ESP = serial.Serial(device, baudrate=1000000, timeout=1)
            ESP.write(f"IDENT\n".encode())
            time.sleep(0.1)
            
            #wait for response
            Timeout = time.time() + 5 # 5 seconds timeout
            self.Utils.LogInfo(f"Waiting for response from {device} ...")
            while not ESP.in_waiting and time.time() < Timeout:
                time.sleep(0.01)
            response = ESP.read(ESP.inWaiting())
            self.Utils.LogInfo(f"Received response from {device}")
            
            responseDecoded = response.decode("utf-8")

            if "HoldDistance" in responseDecoded:
                ESP.close()
                self.EspHoldDistance = serial.Serial(device,baudrate=1000000,timeout=1)
            elif "HoldSpeed" in responseDecoded:
                ESP.close()
                self.EspHoldSpeed = serial.Serial(device,baudrate=1000000,timeout=1)  
            else:
                self.Utils.LogError(f"Could not identify NodeMCU on: {device}")
                self.StopRun()
            
            time.sleep(0.1)

        tUSBcomm = threading.Thread(target=self.handleThreadedFunctions)
        tUSBcomm.start()
        
        return self.EspHoldDistance, self.EspHoldSpeed
        
    #Start both NodeMCUs and wait for responses
    def startNodeMCUs(self):
        #Start both NodeMCUs
        for ESP in [self.EspHoldDistance, self.EspHoldSpeed]:
            ESP.write(f"START\n".encode())
            time.sleep(0.2)
            waitingForResponse = True
            responseTimeout = time.time() + 5 # 5 seconds timeout

            while waitingForResponse:
                response = self.getResponses(ESP)
                if "Received START command. Performing action..." in response:
                    waitingForResponse = False
                elif time.time() > responseTimeout:
                    self.Utils.LogError("No response from NodeMCU")
                    self.StopRun()
                else:
                    time.sleep(0.01)

            time.sleep(0.1)
          
            
    #Stop both NodeMCUs and wait for responses
    def stopNodeMCUs(self):
        for ESP in [self.EspHoldDistance, self.EspHoldSpeed]:
            ESP.flush()
            ESP.write(f"STOP\n".encode())
            waitingForResponse = True
            responseTimeout = time.time() + 5
            
            time.sleep(0.2)

            while waitingForResponse:
                response = self.getResponses(ESP)
                if "Received STOP command. Performing action..." in response:
                    waitingForResponse = False
                elif time.time() > responseTimeout:
                    self.Utils.LogError("No response from NodeMCU")
                    self.StopRun()
                else:
                    time.sleep(0.01)
        
        time.sleep(0.1)
        
        
    def closeNodeMCUs(self):
        time.sleep(0.1)
        self.EspHoldDistance.close()
        time.sleep(0.1)
        self.EspHoldSpeed.close()


    # WIP // no clue why resetting bugs out sometimes, pins have to be set high on boot somehow or inverted
    def resetNodeMCU(self, ESP):
        if ESP == self.EspHoldDistance:
            self.EspHoldDistancePin.set_value(0)
            time.sleep(0.1)
            self.EspHoldDistancePin.set_value(1)
            self.Utils.LogWarning("Reset NodeMCU for HoldDistance")
            
        elif ESP == self.EspHoldSpeed:
            self.EspHoldSpeedPin.set_value(0)
            time.sleep(0.1)
            self.EspHoldSpeedPin.set_value(1)
            self.Utils.LogWarning("Reset NodeMCU for HoldSpeed")
            
        
if __name__ == "__main__":
    try:
        usb_communication = USBCommunication()
        usb_communication.LogWarning("HB test started")
        usb_communication.startNodeMCUs()
        
        while True:
            time.sleep(1)
            
    finally:
        usb_communication.stopNodeMCUs()
        exit()