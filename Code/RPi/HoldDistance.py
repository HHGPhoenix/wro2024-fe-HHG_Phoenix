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
ESPHoldDistance, ESPHoldSpeed = Utils.transferSensorData(StartButton, StopButton, Buzzer1, Cam)

Utils.setupDataLog()




##########################################################
##                                                      ##
##                      Variables                       ##
##                                                      ##
##########################################################
#Constants
Utils.StartSpeed = 100
Utils.Distance = 50
Utils.Kp = 0.7
Utils.Ed = 125 #Edge detection distance in cm
Utils.StartSensor = 2
Utils.Mm = 10
Utils.AngR = 37
Utils.AngL = 42
Utils.startMode = 1


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
    GyroCornerAngle = 90
    Sensor = 0
    
    #Hold Lane
    while Utils.running:
        time.sleep(0.001)
                
        if direction == 0:
            if Sensor != 1:
                Sensor = 1
                Utils.LogInfo("Switched to Sensor 1")
                Utils.usb_communication.sendMessage("S1", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("D35", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("SPEED 180", Utils.ESPHoldSpeed)
                
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
                TIMEOUT = time.time() + CornerWaiTTime
                
        elif direction == 1:
            if Sensor != 2:
                Sensor = 2
                Utils.LogInfo("Switched to Sensor 2")
                Utils.usb_communication.sendMessage("S2", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("D35", Utils.ESPHoldDistance)
                Utils.usb_communication.sendMessage("SPEED 180", Utils.ESPHoldSpeed)
            
            newAngle = oldAngle + GyroCornerAngle
            if Utils.Gyro.angle > newAngle and time.time() > TIMEOUT:
                corners = corners + 1
                timelastcorner = time.time()
                Utils.LogDebug(f"Corner: {corners}")
                Utils.Display.write(f"Corner: {corners}")
                if corners == 4:
                    corners = 0
                    rounds = rounds + 1
                    Utils.Display.write(f"Corner: {corners}", f"Round: {rounds}")
                    
                oldAngle = newAngle
                TIMEOUT = time.time() + CornerWaiTTime
          
                    
        responses = Utils.usb_communication.getResponses(ESPHoldDistance)
        if (responses != None):
            # print(responses)
            for response in responses:
                if "Drive direction counterclockwise" in responses:
                    direction = 1
                if "Drive direction clockwise" in responses:
                    direction = 0
                if "SD1:" in responses:
                    Utils.SensorDistance1 = float(response.split(":")[1].strip())
                if "SD2:" in responses:
                    Utils.SensorDistance2 = float(response.split(":")[1].strip())
                    
        if rounds == 3 and timelastcorner + 0.5 < time.time():
            if Utils.Cam.avg_edge_distance < 165:
                Utils.running = False
                Utils.LogInfo(f"End of path reached {Utils.Cam.avg_edge_distance}")
        
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
        HoldLane(Utils)
    
    except:
        Utils.LogError(traceback.format_exc())
        
    finally:
        Utils.StopRun()