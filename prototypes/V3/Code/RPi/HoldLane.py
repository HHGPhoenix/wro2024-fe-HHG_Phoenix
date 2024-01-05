import time
from RobotCarClasses import *
from threading import Thread
import math


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
def HoldLane(Utils, YCutOffTop=1000000, YCutOffBottom=-1, SIZE=0, LineWaitTime=1, Sensor=2):
    #Variables
    TIMEOUT = 0
    TIMEOUTBlock = 0
    corners = 0
    rounds = 0
    Sensor = 0
    direction = 0
    oldAngle = 0
    BlockKP = 0.05
    old_desired_distance_wall = 0
    
    coordinates_self = (320, 480) #x, y
    
    #Hold Lane
    while Utils.running and rounds < 3:
        try:
            time.sleep(0.001)
                    
            #Count rounds with Gyro
            Utils.angle = Gyro.angle
            if direction == 0:
                newAngle = oldAngle -90
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
                newAngle = oldAngle + 90
                if Gyro.angle > newAngle and time.time() > TIMEOUT:
                    corners = corners + 1
                    Utils.LogDebug(f"Corner: {corners}")
                    if corners == 4:
                        corners = 0
                        rounds = rounds + 1
                        
                    oldAngle = newAngle
                    TIMEOUT = time.time() + LineWaitTime
                
                
            #get objects and calculate new distance
            if time.time() > TIMEOUTBlock:
                block_array = Cam.block_array
                if len(block_array) > 0:
                    #Delete blocks not meeting requirements
                    for block in block_array:
                        if block['my'] < YCutOffTop and block['my'] > YCutOffBottom and block["size"] > SIZE:
                            pass
                        else:
                            block_array.remove(block)
                        
                if len(block_array) > 0:
                    #Sort blocks by size
                    
                    block_array.sort(key=lambda x: x['size'], reverse=True)
                    
                    nextBlock = block_array[0]
                    
                    nextBlock['distancex'] = nextBlock['mx'] - coordinates_self[0]
                    nextBlock['distancey'] = nextBlock['my'] - coordinates_self[1]
                    nextBlock['distance'] = math.sqrt(nextBlock['distancex']**2 + nextBlock['distancey']**2)
                    if nextBlock['distancex'] < 0:
                        nextBlock['distance'] = nextBlock['distance'] * -1
                    
                    if nextBlock['color'] == "red":
                        desired_distance_to_block = -300      
                    elif nextBlock['color'] == "green":
                        desired_distance_to_block = 300
                        
                    error = desired_distance_to_block - nextBlock['distance']
                    desired_distance_wall = 50 - error * BlockKP
                    
                    if desired_distance_wall < 5:
                        desired_distance_wall = 5  
                    elif desired_distance_wall > 95:
                        desired_distance_wall = 95
                        
                        print(desired_distance_wall)

                    if desired_distance_wall > 50:
                        desired_distance_wall = (desired_distance_wall - 100) * -1
                        print(desired_distance_wall)
                            
                        #Send ESPHoldDistance new Distance
                        if desired_distance_wall != old_desired_distance_wall:
                            Utils.EspHoldDistance.write(f"D{desired_distance_wall}\n".encode())
                            print(f"New Distance {desired_distance_wall}")
                            old_desired_distance_wall = desired_distance_wall
                        
                        #Send ESPHoldDistance new Sensor
                        if Sensor != 2:
                            Sensor = 2
                            Utils.EspHoldDistance.write(f"S1\n".encode())
                            print("Switched to Sensor 1")
                            
                    else:
                        if desired_distance_wall != old_desired_distance_wall:
                            Utils.EspHoldDistance.write(f"D{desired_distance_wall}\n".encode())
                            print(f"New Distance {desired_distance_wall}")
                            old_desired_distance_wall = desired_distance_wall
                        
                        if Sensor != 1:
                            Sensor = 1
                            Utils.EspHoldDistance.write(f"S2\n".encode())
                            print("Switched to Sensor 2")
                        
                        
            #check for direction
            if Utils.EspHoldDistance.in_waiting > 0:
                response = Utils.EspHoldDistance.read(Utils.EspHoldDistance.in_waiting).decode()
                if "Drive direction counterclockwise" in response:
                    direction = 1
                elif "Drive direction clockwise" in response:
                    direction = 0
                    
                Utils.LogDebug(f"Response from EspHoldDistance: {response}")
            
            Utils.LogData()    
            
        except Exception as e:
            Utils.LogError(e)
            Utils.StopRun()
            
    Utils.LogDebug("HoldLane finished")
    Utils.StopRun()



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
    
    except Exception as e:
        Utils.LogError(e)
        Utils.StopRun()